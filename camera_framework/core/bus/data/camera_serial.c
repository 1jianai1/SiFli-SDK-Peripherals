#include "camera_serial.h"

#include "camera_serial_hw.h"
#include "rthw.h"

#define DBG_TAG "camera.serial"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define GC032A_SERIAL_WIDTH        640U
#define GC032A_SERIAL_HEIGHT       480U
#define GC032A_SERIAL_FRAME_SIZE   \
    (GC032A_SERIAL_WIDTH * GC032A_SERIAL_HEIGHT * 2U)
#define CAMERA_SERIAL_BLOCK_COUNT  2U
#define CAMERA_SERIAL_PACK_RATIO   4U
#define CAMERA_SERIAL_THREAD_TICK  2U
#define CAMERA_SERIAL_STOP_WAIT_MS 100U
#define CAMERA_SERIAL_MAX_BLOCKS_PER_WAKE 16U

#if !defined(SF32LB52X)
#error "The GC032A serial GPIO-DMA adapter currently supports SF32LB52X only"
#endif

#if (CAMERA_SERIAL_SAMPLE_BUFFER_SIZE % 8) != 0
#error "CAMERA_SERIAL_SAMPLE_BUFFER_SIZE must be a multiple of 8"
#endif

static uint8_t
    s_camera_serial_samples[CAMERA_SERIAL_SAMPLE_BUFFER_SIZE]
        __attribute__((aligned(4)));
static uint8_t
    s_camera_serial_packed[CAMERA_SERIAL_SAMPLE_BUFFER_SIZE /
                           CAMERA_SERIAL_BLOCK_COUNT /
                           CAMERA_SERIAL_PACK_RATIO +
                           3U]
        __attribute__((aligned(4)));
static camera_serial_handle_t s_camera_serial_handle;

static const bus_adapter_ops_t s_camera_serial_ops;
static bus_adapter_t s_camera_serial_adapter =
{
    .name = CAMERA_SERIAL_BUS_ADAPTER_NAME,
    .type = BUS_TYPE_SPI,
    .ops = &s_camera_serial_ops,
    .priv = &s_camera_serial_handle,
};

static void camera_serial_apply_defaults(camera_serial_handle_t *handle)
{
    handle->config.mode = BUS_CAPTURE_MODE_RGB565;
    handle->config.sample_buffer = s_camera_serial_samples;
    handle->config.sample_buffer_size =
        CAMERA_SERIAL_SAMPLE_BUFFER_SIZE;
    handle->config.sample_buffer_pool_size =
        (uint32_t)sizeof(s_camera_serial_samples);
    handle->config.packed_buffer = s_camera_serial_packed;
    handle->config.packed_buffer_size =
        (uint32_t)sizeof(s_camera_serial_packed);
}

static void camera_serial_reset_decoder(camera_serial_handle_t *handle)
{
    camera_serial_packer_reset(&handle->packer);
    if (handle->state.capture_enabled &&
        handle->config.frame_buffer != RT_NULL)
    {
        camera_serial_decoder_start(&handle->decoder,
                                    handle->config.frame_buffer,
                                    handle->config.frame_buffer_size);
    }
    else
    {
        camera_serial_decoder_reset(&handle->decoder);
    }
}

static void camera_serial_process_block(camera_serial_handle_t *handle,
                                        uint32_t offset,
                                        uint32_t sequence)
{
    bus_frame_notify_callback_t callback = RT_NULL;
    void *callback_data = RT_NULL;
    uint8_t *frame_buffer = RT_NULL;
    size_t packed_size = 0;
    size_t frame_length = 0;
    uint32_t block_size = handle->config.sample_buffer_size /
                          CAMERA_SERIAL_BLOCK_COUNT;
    uint8_t *samples = handle->config.sample_buffer + offset;
    int result;

#ifdef PSRAM_CACHE_WB
    mpu_dcache_invalidate(samples, block_size);
#endif
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        handle->state.dma_errors++;
        return;
    }
    if (!handle->state.capture_enabled)
    {
        rt_mutex_release(handle->lock);
        return;
    }

    result = camera_serial_packer_feed(&handle->packer,
                                       samples,
                                       block_size,
                                       handle->config.packed_buffer,
                                       handle->config.packed_buffer_size,
                                       &packed_size);
    if (result != 0)
    {
        handle->state.bad_frames++;
        handle->state.capture_enabled = 0;
        LOG_E("packer stopped: %d", result);
        rt_mutex_release(handle->lock);
        return;
    }
    if (handle->state.producer_sequence != sequence)
    {
        handle->state.overruns++;
        handle->state.bad_frames++;
        camera_serial_reset_decoder(handle);
        rt_mutex_release(handle->lock);
        return;
    }
    result = camera_serial_decoder_feed(&handle->decoder,
                                        handle->config.packed_buffer,
                                        packed_size,
                                        &frame_length);
    if (result == CAMERA_SERIAL_DECODE_BAD_FRAME)
    {
        handle->state.bad_frames++;
        rt_mutex_release(handle->lock);
        return;
    }
    if (result == CAMERA_SERIAL_DECODE_OVERFLOW ||
        result == CAMERA_SERIAL_DECODE_INVALID)
    {
        handle->state.bad_frames++;
        handle->state.capture_enabled = 0;
        LOG_E("decoder stopped: %d", result);
        rt_mutex_release(handle->lock);
        return;
    }
    if (result == CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        handle->state.capture_enabled = 0;
        handle->state.completed_frames++;
#ifdef PSRAM_CACHE_WB
        mpu_dcache_clean(handle->config.frame_buffer, frame_length);
#endif
        callback = handle->callback;
        callback_data = handle->callback_data;
        frame_buffer = handle->config.frame_buffer;
    }
    rt_mutex_release(handle->lock);

    if (callback != RT_NULL)
    {
        callback(frame_buffer,
                 (uint32_t)frame_length,
                 callback_data);
    }
}

static void camera_serial_hw_block_ready(uint32_t offset,
                                         uint32_t size,
                                         void *context)
{
    camera_serial_handle_t *handle = (camera_serial_handle_t *)context;
    uint32_t next_sequence;
    uint32_t expected_offset;
    uint32_t block_size;

    if (handle == RT_NULL || !handle->state.running)
    {
        return;
    }
    block_size = handle->config.sample_buffer_size /
                 CAMERA_SERIAL_BLOCK_COUNT;
    next_sequence = handle->state.producer_sequence + 1U;
    expected_offset = ((next_sequence - 1U) & 1U) != 0U
                          ? block_size
                          : 0U;
    if (size != block_size || offset != expected_offset)
    {
        handle->state.dma_errors++;
        return;
    }

    handle->state.producer_sequence = next_sequence;
    handle->state.dma_callbacks++;
    handle->state.dma_bytes += size;
    rt_sem_release(handle->block_sem);
}

static void camera_serial_worker(void *parameter)
{
    camera_serial_handle_t *handle =
        (camera_serial_handle_t *)parameter;

    while (!handle->state.worker_exit)
    {
        uint32_t producer;
        uint32_t processed_blocks = 0U;

        if (rt_sem_take(handle->block_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }
        if (handle->state.worker_exit)
        {
            break;
        }

        while (handle->state.consumer_sequence !=
                   handle->state.producer_sequence &&
               processed_blocks < CAMERA_SERIAL_MAX_BLOCKS_PER_WAKE)
        {
            uint32_t next_sequence;
            uint32_t block_size;
            uint32_t offset;

            producer = handle->state.producer_sequence;
            if ((uint32_t)(producer -
                           handle->state.consumer_sequence) > 1U)
            {
                if (rt_mutex_take(handle->lock,
                                  RT_WAITING_FOREVER) == RT_EOK)
                {
                    handle->state.overruns++;
                    handle->state.bad_frames++;
                    camera_serial_reset_decoder(handle);
                    handle->state.consumer_sequence = producer - 1U;
                    rt_mutex_release(handle->lock);
                }
            }

            next_sequence = handle->state.consumer_sequence + 1U;
            block_size = handle->config.sample_buffer_size /
                         CAMERA_SERIAL_BLOCK_COUNT;
            offset = ((next_sequence - 1U) & 1U) != 0U
                         ? block_size
                         : 0U;

            camera_serial_process_block(handle, offset, next_sequence);
            handle->state.consumer_sequence = next_sequence;
            processed_blocks++;
        }
        if (handle->state.consumer_sequence !=
            handle->state.producer_sequence)
        {
            rt_thread_mdelay(1);
        }
    }

    handle->worker = RT_NULL;
}

static int camera_serial_create_worker(camera_serial_handle_t *handle)
{
    handle->block_sem = rt_sem_create("camser", 0, RT_IPC_FLAG_FIFO);
    if (handle->block_sem == RT_NULL)
    {
        return BUS_ERR_HW;
    }
    handle->lock = rt_mutex_create("camslk", RT_IPC_FLAG_PRIO);
    if (handle->lock == RT_NULL)
    {
        rt_sem_delete(handle->block_sem);
        handle->block_sem = RT_NULL;
        return BUS_ERR_HW;
    }

    handle->worker = rt_thread_create("camser",
                                      camera_serial_worker,
                                      handle,
                                      CAMERA_SERIAL_WORKER_STACK_SIZE,
                                      RT_THREAD_PRIORITY_HIGH + 2,
                                      CAMERA_SERIAL_THREAD_TICK);
    if (handle->worker == RT_NULL)
    {
        rt_mutex_delete(handle->lock);
        rt_sem_delete(handle->block_sem);
        handle->lock = RT_NULL;
        handle->block_sem = RT_NULL;
        return BUS_ERR_HW;
    }
    rt_thread_startup(handle->worker);
    return BUS_OK;
}

static void camera_serial_destroy_worker(camera_serial_handle_t *handle)
{
    uint32_t waited_ms = 0U;

    if (handle->worker != RT_NULL)
    {
        handle->state.worker_exit = 1U;
        rt_sem_release(handle->block_sem);
        while (handle->worker != RT_NULL &&
               waited_ms < CAMERA_SERIAL_STOP_WAIT_MS)
        {
            rt_thread_mdelay(1);
            waited_ms++;
        }
        if (handle->worker != RT_NULL)
        {
            rt_thread_delete(handle->worker);
            handle->worker = RT_NULL;
        }
    }
    if (handle->lock != RT_NULL)
    {
        rt_mutex_delete(handle->lock);
        handle->lock = RT_NULL;
    }
    if (handle->block_sem != RT_NULL)
    {
        rt_sem_delete(handle->block_sem);
        handle->block_sem = RT_NULL;
    }
}

int camera_serial_configure(bus_adapter_t *self,
                            const bus_adapter_config_t *config)
{
    if (self == RT_NULL || self->priv == RT_NULL || config == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    if (config->mode != BUS_CAPTURE_MODE_RGB565)
    {
        return BUS_ERR_NOT_SUPPORTED;
    }

    ((camera_serial_handle_t *)self->priv)->config.mode = config->mode;
    return BUS_OK;
}

int camera_serial_init(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;
    bus_frame_notify_callback_t callback;
    void *callback_data;
    int result;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }

    handle = (camera_serial_handle_t *)self->priv;
    callback = handle->callback;
    callback_data = handle->callback_data;
    rt_memset(handle, 0, sizeof(*handle));
    handle->callback = callback;
    handle->callback_data = callback_data;
    camera_serial_apply_defaults(handle);

    camera_serial_decoder_init(&handle->decoder,
                               GC032A_SERIAL_WIDTH,
                               GC032A_SERIAL_HEIGHT);
    camera_serial_packer_reset(&handle->packer);
    result = camera_serial_create_worker(handle);
    if (result != BUS_OK)
    {
        return result;
    }
    if (camera_serial_hw_init(camera_serial_hw_block_ready, handle) !=
        CAMERA_SERIAL_HW_OK)
    {
        camera_serial_destroy_worker(handle);
        return BUS_ERR_HW;
    }

    handle->state.initialized = 1U;
    LOG_I("initialized: GPIO-DMA CLK=PA39 D1=PA38 D0=PA37 samples=%u",
          (unsigned int)handle->config.sample_buffer_size);
    return BUS_OK;
}

int camera_serial_deinit(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;
    int result;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    result = camera_serial_stop(self);
    camera_serial_hw_deinit();
    camera_serial_destroy_worker(handle);
    camera_serial_decoder_reset(&handle->decoder);
    camera_serial_packer_reset(&handle->packer);
    handle->state.initialized = 0U;
    LOG_I("deinitialized");
    return result;
}

int camera_serial_start(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    if (!handle->state.initialized)
    {
        return BUS_ERR_INVALID;
    }
    if (handle->state.running)
    {
        return BUS_OK;
    }

    handle->state.producer_sequence = 0U;
    handle->state.consumer_sequence = 0U;
    handle->state.running = 1U;
    if (camera_serial_hw_start(handle->config.sample_buffer,
                               handle->config.sample_buffer_size) !=
        CAMERA_SERIAL_HW_OK)
    {
        handle->state.running = 0U;
        LOG_E("GPIO-DMA start failed");
        return BUS_ERR_HW;
    }
    return BUS_OK;
}

int camera_serial_stop(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;
    int hw_result;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    if (!handle->state.running)
    {
        return BUS_OK;
    }

    hw_result = camera_serial_hw_stop();
    handle->state.running = 0U;
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        handle->state.capture_enabled = 0U;
        camera_serial_reset_decoder(handle);
        rt_mutex_release(handle->lock);
    }
    if (hw_result != CAMERA_SERIAL_HW_OK)
    {
        LOG_E("GPIO-DMA stop failed");
        return BUS_ERR_HW;
    }
    return BUS_OK;
}

int camera_serial_set_frame_notify_callback(
    bus_adapter_t *self,
    bus_frame_notify_callback_t callback,
    void *user_data)
{
    camera_serial_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    if (handle->lock != RT_NULL)
    {
        rt_mutex_take(handle->lock, RT_WAITING_FOREVER);
    }
    handle->callback = callback;
    handle->callback_data = user_data;
    if (handle->lock != RT_NULL)
    {
        rt_mutex_release(handle->lock);
    }
    return BUS_OK;
}

static int camera_serial_arm(camera_serial_handle_t *handle,
                             void *buffer,
                             uint32_t size)
{
    int result;

    if (buffer == RT_NULL || size < GC032A_SERIAL_FRAME_SIZE ||
        handle->lock == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return BUS_ERR_HW;
    }

    result = camera_serial_decoder_start(&handle->decoder, buffer, size);
    if (result != CAMERA_SERIAL_DECODE_MORE)
    {
        rt_mutex_release(handle->lock);
        return BUS_ERR_INVALID;
    }
    camera_serial_packer_reset(&handle->packer);
    handle->config.frame_buffer = (uint8_t *)buffer;
    handle->config.frame_buffer_size = size;
    handle->state.consumer_sequence =
        handle->state.producer_sequence;
    handle->state.dma_callbacks = 0U;
    handle->state.dma_bytes = 0U;
    handle->state.capture_enabled = 1U;
    rt_mutex_release(handle->lock);
    return BUS_OK;
}

int camera_serial_start_capture(bus_adapter_t *self,
                                void *buffer,
                                uint32_t size)
{
    camera_serial_handle_t *handle;
    int result;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    result = camera_serial_arm(handle, buffer, size);
    if (result != BUS_OK)
    {
        return result;
    }
    if (!handle->state.running)
    {
        result = camera_serial_start(self);
        if (result != BUS_OK)
        {
            handle->state.capture_enabled = 0U;
        }
    }
    return result;
}

int camera_serial_rearm_capture(bus_adapter_t *self,
                                void *buffer,
                                uint32_t size)
{
    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    return camera_serial_arm((camera_serial_handle_t *)self->priv,
                             buffer,
                             size);
}

int camera_serial_abort_capture(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    if (rt_mutex_take(handle->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return BUS_ERR_HW;
    }
    handle->state.capture_enabled = 0U;
    handle->state.consumer_sequence =
        handle->state.producer_sequence;
    camera_serial_reset_decoder(handle);
    rt_mutex_release(handle->lock);
    return BUS_OK;
}

int camera_serial_set_pingpong_size(bus_adapter_t *self, uint32_t size)
{
    camera_serial_handle_t *handle;
    uint32_t sample_size;
    int was_running;
    int result;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return BUS_ERR_INVALID;
    }
    handle = (camera_serial_handle_t *)self->priv;
    if (size == 0U ||
        size > handle->config.sample_buffer_pool_size /
                   CAMERA_SERIAL_PACK_RATIO)
    {
        return BUS_ERR_INVALID;
    }
    sample_size = size * CAMERA_SERIAL_PACK_RATIO;
    if (sample_size < CAMERA_SERIAL_SAMPLE_BUFFER_SIZE)
    {
        sample_size = CAMERA_SERIAL_SAMPLE_BUFFER_SIZE;
    }
    was_running = handle->state.running;
    if (was_running)
    {
        result = camera_serial_stop(self);
        if (result != BUS_OK)
        {
            return result;
        }
    }
    handle->config.sample_buffer_size = sample_size;
    handle->config.packed_buffer_size =
        sample_size / CAMERA_SERIAL_BLOCK_COUNT /
        CAMERA_SERIAL_PACK_RATIO + 3U;
    LOG_I("sample ping-pong resized to %u bytes",
          (unsigned int)sample_size);
    if (was_running)
    {
        return camera_serial_start(self);
    }
    return BUS_OK;
}

int camera_serial_set_mode(bus_adapter_t *self, bus_capture_mode_t mode)
{
    if (mode != BUS_CAPTURE_MODE_RGB565)
    {
        return BUS_ERR_NOT_SUPPORTED;
    }
    return camera_serial_configure(
        self,
        &(bus_adapter_config_t){.mode = BUS_CAPTURE_MODE_RGB565});
}

void camera_serial_dump_state(bus_adapter_t *self)
{
    camera_serial_handle_t *handle;

    if (self == RT_NULL || self->priv == RT_NULL)
    {
        return;
    }
    handle = (camera_serial_handle_t *)self->priv;
    LOG_W("bus=gpio-dma running=%u capture=%u frames=%u bad=%u "
          "dma_err=%u overrun=%u hw_err=%u",
          handle->state.running,
          handle->state.capture_enabled,
          (unsigned int)handle->state.completed_frames,
          (unsigned int)handle->state.bad_frames,
          (unsigned int)handle->state.dma_errors,
          (unsigned int)handle->state.overruns,
          (unsigned int)camera_serial_hw_error_count());
    LOG_W("dma_callbacks=%u dma_bytes=%u producer=%u consumer=%u",
          (unsigned int)handle->state.dma_callbacks,
          (unsigned int)handle->state.dma_bytes,
          (unsigned int)handle->state.producer_sequence,
          (unsigned int)handle->state.consumer_sequence);
}

static const bus_adapter_ops_t s_camera_serial_ops =
{
    .config = camera_serial_configure,
    .init = camera_serial_init,
    .deinit = camera_serial_deinit,
    .start = camera_serial_start,
    .stop = camera_serial_stop,
    .set_frame_notify_callback = camera_serial_set_frame_notify_callback,
    .start_capture = camera_serial_start_capture,
    .rearm_capture = camera_serial_rearm_capture,
    .abort_capture = camera_serial_abort_capture,
    .set_pingpong_size = camera_serial_set_pingpong_size,
    .set_mode = camera_serial_set_mode,
    .dump_state = camera_serial_dump_state,
};

static int camera_serial_register(void)
{
    int result = bus_adapter_register(&s_camera_serial_adapter);

    if (result != BUS_OK)
    {
        LOG_E("adapter register failed: %d", result);
    }
    return result;
}
INIT_BOARD_EXPORT(camera_serial_register);
