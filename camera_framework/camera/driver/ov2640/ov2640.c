/**
 * @file    ov2640.c
 * @brief   OV2640 camera sensor driver implementation
 *
 * This module implements the OV2640 sensor initialization, register
 * configuration, image parameter control, single-shot capture, and streaming
 * operations exposed through the camera handle's driver ops interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "ov2640.h"
#include "../camera_driver_desc.h"
#include "ov2640_regs.h"
#include "ov2640_settings.h"
#include "rtthread.h"

#define DBG_TAG "ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <string.h>

static sensor_device_t  *s_active_device = RT_NULL;
static sensor_device_t  s_device;

static const pixformat_t g_camera_pixformats[] = {
    PIXFORMAT_JPEG,
    PIXFORMAT_RGB565,
    PIXFORMAT_YUV422,
};

static const framesize_t g_camera_framesizes[] = {
    FRAMESIZE_96X96,
    FRAMESIZE_QQVGA,
    FRAMESIZE_128X128,
    FRAMESIZE_QCIF,
    FRAMESIZE_HQVGA,
    FRAMESIZE_240X240,
    FRAMESIZE_QVGA,
    FRAMESIZE_320X320,
    FRAMESIZE_CIF,
    FRAMESIZE_HVGA,
    FRAMESIZE_VGA,
    FRAMESIZE_SVGA,
    FRAMESIZE_XGA,
    FRAMESIZE_HD,
    FRAMESIZE_SXGA,
    FRAMESIZE_UXGA,
};

/* max_buffer_size: UXGA (1600×1200) in RGB565 = 1600*1200*2 bytes */
static const camera_capabilities_t g_camera_caps = {
    .pixformats      = g_camera_pixformats,
    .num_pixformats  = sizeof(g_camera_pixformats) / sizeof(g_camera_pixformats[0]),
    .framesizes      = g_camera_framesizes,
    .num_framesizes  = sizeof(g_camera_framesizes) / sizeof(g_camera_framesizes[0]),
    .max_buffer_size = 1600U * 1200U * 2U,
};

static const camera_capture_config_t g_default_config = {
    .pixformat = PIXFORMAT_JPEG,
    .framesize = FRAMESIZE_VGA,
    .quality = 10,
};

typedef struct {
    sccb_config_t sccb;
    camera_sensor_runtime_config_t runtime;
} sensor_hw_config_t;

static const sensor_hw_config_t g_hw_config = {
    .sccb = {
        .bus_name               = CAMERA_SCCB_I2C_BUS_NAME,
        .timeout_ms             = CAMERA_SCCB_TIMEOUT_MS,
        .max_hz                 = CAMERA_SCCB_MAX_HZ,
    },
    .runtime = {
        .adapter_name = CAMERA_DATA_BUS_ADAPTER_NAME,
        .adapter_type = BUS_TYPE_DVP,
        .frame_timeout_ms = CAMERA_READ_TIMEOUT_MS,
        .default_config = &g_default_config,
    },
};

static int sensor_open(void);
static int sensor_close(void);
static int sensor_apply_pixformat(pixformat_t pixformat);
static int sensor_apply_framesize(framesize_t framesize);
static int sensor_apply_quality(uint8_t quality);
static rt_size_t sensor_capture(void *buffer, rt_size_t buffer_size);
static int sensor_capture_async(void *buffer,
                                rt_size_t buffer_size,
                                camera_capture_done_callback_t callback,
                                void *context);
static int sensor_start_stream(const camera_stream_start_args_t *args);
static int sensor_stop_stream(void);

const camera_device_ops_t ov2640_ops = {
    .capabilities   = &g_camera_caps,
    .default_config = &g_default_config,
    .open           = sensor_open,
    .close          = sensor_close,
    .set_pixformat  = sensor_apply_pixformat,
    .set_framesize  = sensor_apply_framesize,
    .set_quality    = sensor_apply_quality,
    .capture        = sensor_capture,
    .capture_async  = sensor_capture_async,
    .start_stream   = sensor_start_stream,
    .stop_stream    = sensor_stop_stream,
};

CAMERA_DRIVER_EXPORT(ov2640, &ov2640_ops);


typedef enum {
    ASPECT_RATIO_4X3,
    ASPECT_RATIO_3X2,
    ASPECT_RATIO_16X10,
    ASPECT_RATIO_5X3,
    ASPECT_RATIO_16X9,
    ASPECT_RATIO_21X9,
    ASPECT_RATIO_5X4,
    ASPECT_RATIO_1X1,
    ASPECT_RATIO_9X16
} aspect_ratio_t;

static const aspect_ratio_t s_aspect_ratios[FRAMESIZE_INVALID] = {
    ASPECT_RATIO_1X1,  /* 96x96 */
    ASPECT_RATIO_4X3,  /* QQVGA */
    ASPECT_RATIO_1X1,  /* 128x128 */
    ASPECT_RATIO_5X4,  /* QCIF  */
    ASPECT_RATIO_4X3,  /* HQVGA */
    ASPECT_RATIO_1X1,  /* 240x240 */
    ASPECT_RATIO_4X3,  /* QVGA  */
    ASPECT_RATIO_1X1,  /* 320x320 */
    ASPECT_RATIO_4X3,  /* CIF   */
    ASPECT_RATIO_3X2,  /* HVGA  */
    ASPECT_RATIO_4X3,  /* VGA   */
    ASPECT_RATIO_4X3,  /* SVGA  */
    ASPECT_RATIO_4X3,  /* XGA   */
    ASPECT_RATIO_16X9, /* HD    */
    ASPECT_RATIO_5X4,  /* SXGA  */
    ASPECT_RATIO_4X3,  /* UXGA  */
};




/**
 * @brief Invalidate the cached bank so the next access forces a BANK_SEL write.
 *
 * Call on deinit to ensure the driver state is consistent after re-open.
 */
static void sensor_reset_bank_state(void)
{
    if (s_active_device != RT_NULL)
    {
        s_active_device->current_bank = (rt_uint8_t)BANK_MAX;
    }
}

/**
 * @brief Switch the active register bank if needed.
 *
 * Skips the SCCB write when @p bank already matches the cached value.
 *
 * @param bank is the target bank (BANK_SENSOR or BANK_DSP).
 *
 * @return Return 0 on success; negative SCCB error on failure.
 */
static int sensor_set_bank(ov2640_bank_t bank)
{
    sensor_device_t *dev = s_active_device;
    rt_uint8_t cached = (dev != RT_NULL) ? dev->current_bank : (rt_uint8_t)BANK_MAX;
    if ((rt_uint8_t)bank == cached) return 0;
    int res = sccb_write(OV2640_ADDR, BANK_SEL, (uint8_t)bank);
    if(res) return res;
    if (dev != RT_NULL)
    {
        dev->current_bank = (rt_uint8_t)bank;
    }
    return 0;
}

/**
 * @brief  Write OV2640 single register
 * @param  bank: Bank ID
 * @param  reg: Register address
 * @param  data: Data to write
 * @return 0 on success, negative error code on failure
 */
static int sensor_write_reg(ov2640_bank_t bank, uint8_t reg, uint8_t data)
{
    int ret;
    ret = sensor_set_bank(bank);
    if(ret) return ret;
    ret = sccb_write(OV2640_ADDR, reg, data);
    if(ret) return ret;
    return 0;
}

/**
 * @brief  Write OV2640 multiple registers
 * @param  regs: Pointer to register array, ending with {0, 0}
 * @return RT_EOK on success, negative error code on failure
 */
static int sensor_write_regs(const uint8_t (*regs)[2])
{
    int ret;
    const uint8_t (*reg_ptr)[2] = regs;

    while(((*reg_ptr)[0] != 0) || ((*reg_ptr)[1] != 0))
    {
        ret = sccb_write(OV2640_ADDR, (*reg_ptr)[0], (*reg_ptr)[1]);
        if(ret) return ret;
        reg_ptr++;
    }
    return 0;
}

/**
 * @brief  Read OV2640 single register
 * @param  data: Pointer to store read data
 * @return register value, if read fails, return 0
 */
static uint8_t sensor_read_reg(uint8_t reg)
{
    return sccb_read(OV2640_ADDR, reg);
}

/**
 * @brief Get specific bits from OV2640 register, align to LSB by offset
 * @param reg: Register address
 * @param bank: Register bank
 * @param mask: Bit mask
 * @param offset: Bit offset
 * @param value: Pointer to store the extracted bits
 * @return aligned register bits, if read fails, return 0
 */
static uint8_t sensor_get_bits(uint8_t reg,ov2640_bank_t bank, uint8_t mask, uint32_t offset)
{
    int ret;
    uint8_t reg_value;
    ret = sensor_set_bank(bank);
    if(ret) return 0;
    reg_value = sensor_read_reg(reg);
    return (reg_value >> offset) & mask;

}


/**
 * @brief Reset OV2640 sensor to default settings.
 *
 * Sends the software reset command then writes the base CIF initialisation
 * register table.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; negative error code on failure.
 */
static int sensor_reset(void)
{
    sensor_write_reg(BANK_SENSOR, COM7, COM7_SRST);
    rt_thread_mdelay(10);
    sensor_write_regs(ov2640_settings_cif);
    return 0;
}

/**
 * @brief Read all sensor registers and populate @p dev->status.
 *
 * Queries AEC, AGC, AWB, gain ceiling, gamma, lens correction and geometry
 * settings over SCCB and mirrors them into the in-memory status struct so
 * callers can inspect parameters without further I2C transactions.
 *
 * @param dev is a pointer to the sensor handle.
 *
 * @return Return 0 on success; -1 if a bank switch fails.
 */
static int sensor_init_status(sensor_device_t *dev)
{
    int ret = sensor_set_bank(BANK_DSP);
    if (ret != 0) return -1;
    dev->runtime.quality = sensor_read_reg(QS);
    dev->runtime.framesize = FRAMESIZE_UXGA;
    return 0;
}

/**
 * @brief Set pixel output format (RGB565, YUV422, JPEG, RAW8).
 *
 * @param dev      is a pointer to the sensor handle.
 * @param pixformat is the desired output format.
 *
 * @return Return 0 on success; -1 for an unsupported format.
 */
static int sensor_set_pixformat(sensor_device_t *dev, pixformat_t pixformat)
{
    int ret = 0;

    switch(pixformat)
    {
        case PIXFORMAT_RGB565:
            sensor_write_regs(ov2640_settings_rgb565);
            break;
        case PIXFORMAT_YUV422:
            sensor_write_regs(ov2640_settings_yuv422);
            break;
        case PIXFORMAT_JPEG:
            sensor_write_regs(ov2640_settings_jpeg3);
            break;
        case PIXFORMAT_RAW8:
            sensor_write_regs(ov2640_settings_raw8);
            break;
        default:
            ret = -1;
    }

    return ret;
}

/**
 * @brief Configure sensor window (crop and scale) settings
 * @param dev Pointer to sensor_device_t structure
 * @param mode Sensor mode (CIF, SVGA, UXGA)
 * @param offset_x Horizontal offset
 * @param offset_y Vertical offset
 * @param max_x Maximum horizontal size
 * @param max_y Maximum vertical size
 * @param w Output width
 * @param h Output height
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_window(sensor_device_t *dev, ov2640_sensor_mode_t mode,int offset_x, int offset_y, int max_x, int max_y, int w, int h)
{
    int ret;
    const uint8_t (*regs)[2];
    ov2640_clk_t c;
    c.reserved = 0;

    max_x /= 4;
    max_y /= 4;
    w /= 4;
    h /= 4;
    uint8_t win_regs[][2] = {
        {BANK_SEL, BANK_DSP},
        {HSIZE, max_x & 0xFF},
        {VSIZE, max_y & 0xFF},
        {XOFFL, offset_x & 0xFF},
        {YOFFL, offset_y & 0xFF},
        {VHYX, ((max_y >> 1) & 0X80) | ((offset_y >> 4) & 0X70) | ((max_x >> 5) & 0X08) | ((offset_x >> 8) & 0X07)},
        {TEST, (max_x >> 2) & 0X80},
        {ZMOW, (w)&0xFF},
        {ZMOH, (h)&0xFF},
        {ZMHH, ((h>>6)&0x04)|((w>>8)&0x03)},
        {0, 0}
    };

    if (dev->runtime.pixformat == PIXFORMAT_JPEG) {
        c.clk_2x = 1;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div =6;
        if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 24;
        }
    } else {
        c.clk_2x = 1;
        c.clk_div =3;
        c.pclk_auto = 1;
        c.pclk_div = 4;
        if (mode == OV2640_MODE_CIF) {
            c.clk_div = 3;
        } else if(mode == OV2640_MODE_UXGA) {
            c.pclk_div = 12;
        }
    }

    if (mode == OV2640_MODE_CIF) {
        regs = ov2640_settings_to_cif;
    } else if (mode == OV2640_MODE_SVGA) {
        regs = ov2640_settings_to_svga;
    } else {
        regs = ov2640_settings_to_uxga;
    }

    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
    sensor_write_regs(regs);
    sensor_write_regs(win_regs);
    sensor_set_bank(BANK_SENSOR);
    sensor_write_reg(BANK_SENSOR, CLKRC, c.clk);
    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_DVP_SP, c.pclk);
    sensor_set_bank(BANK_DSP);
    sensor_write_reg(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);

    rt_thread_mdelay(10);
    //required when changing resolution
    sensor_set_pixformat(dev, dev->runtime.pixformat);

    return 0;
}

/**
 * @brief Set frame size/resolution
 * @param dev Pointer to sensor_device_t structure
 * @param framesize Desired frame size (QVGA, VGA, SVGA, UXGA, etc.)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_framesize(sensor_device_t *dev, framesize_t framesize)
{
    if(framesize >= FRAMESIZE_INVALID) {
        return -1;
    }

    int ret = 0;
    uint16_t w;
    uint16_t h;
    aspect_ratio_t ratio = s_aspect_ratios[framesize];
    uint16_t max_x = ratio_table[ratio].max_x;
    uint16_t max_y = ratio_table[ratio].max_y;
    uint16_t offset_x = ratio_table[ratio].offset_x;
    uint16_t offset_y = ratio_table[ratio].offset_y;
    ov2640_sensor_mode_t mode = OV2640_MODE_UXGA;

    if (camera_sensor_get_resolution(framesize, &w, &h) != RT_EOK) {
        return -1;
    }

    if (framesize <= FRAMESIZE_CIF) {
        mode = OV2640_MODE_CIF;
        max_x /= 4;
        max_y /= 4;
        offset_x /= 4;
        offset_y /= 4;
        if(max_y > 296){
            max_y = 296;
        }
    } else if (framesize <= FRAMESIZE_SVGA) {
        mode = OV2640_MODE_SVGA;
        max_x /= 2;
        max_y /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    ret = sensor_set_window(dev, mode, offset_x, offset_y, max_x, max_y, w, h);
    return ret;
}

/**
 * @brief Set JPEG compression quality
 * @param dev Pointer to sensor_device_t structure
 * @param quality Quality scale (0-63, lower = better quality, higher compression)
 * @return 0 on success, negative error code on failure
 */
static int sensor_set_quality(sensor_device_t *dev, int quality)
{
    int ret = 0;

    if(quality < 0) quality = 0;
    if(quality > 63) quality = 63;
    ret = sensor_write_reg(BANK_DSP, QS, (uint8_t)quality);
    if(ret != 0) {
        return ret;
    }
    dev->runtime.quality = quality;
    return 0;
}



/**
 * @brief Reset OV2640 and load the initial register state into the handle.
 *
 * Issues a software reset (CIF init table) and reads the configuration
 * registers into @p dev->status. The sensor is left in JPEG/UXGA defaults.
 *
 * @param dev is a pointer to the sensor handle to initialise.
 *
 * @return Return 0 on success; negative error code on failure.
 */
static int sensor_init(sensor_device_t *dev)
{
    int ret;

    ret = sensor_reset();
    if (ret != 0)
    {
        return ret;
    }

    ret = sensor_init_status(dev);
    if (ret != 0)
    {
        return ret;
    }

    return 0;
}


/**
 * @brief Open the single OV2640 driver instance.
 *
 * Powers up SCCB + sensor + data bus and prepares single-shot/stream state.
 * The driver is currently single-instance, so repeated opens simply reuse the
 * same static state object.
 */
static int sensor_open(void)
{
    sensor_device_t *cam_dev = &s_device;
    int ret;

    if (cam_dev->runtime.is_open)
    {
        return RT_EOK;
    }

    rt_memset(cam_dev, 0, sizeof(*cam_dev));
    cam_dev->current_bank = (rt_uint8_t)BANK_MAX;
    s_active_device = cam_dev;

    ret = sccb_init(&g_hw_config.sccb);
    if (ret != RT_EOK)
    {
        LOG_E("SCCB init failed: %d", ret);
        s_active_device = RT_NULL;
        return -RT_ERROR;
    }


    ret = sensor_init(cam_dev);
    if (ret != 0)
    {
        LOG_E("OV2640 init failed: %d", ret);
        sccb_deinit();
        s_active_device = RT_NULL;
        return -RT_ERROR;
    }
    ret = camera_sensor_runtime_open(&cam_dev->runtime, &g_hw_config.runtime);
    if (ret != RT_EOK)
    {
        LOG_E("Camera runtime init failed: %d", ret);
        sccb_deinit();
        s_active_device = RT_NULL;
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief Close the single OV2640 driver instance.
 *
 * Stops any ongoing transfer, deinitialises the active data bus backend and
 * SCCB, then destroys the frame semaphore.
 */
static int sensor_close(void)
{
    sensor_device_t *cam_dev = &s_device;

    if (!cam_dev->runtime.is_open)
    {
        return RT_EOK;
    }

    camera_sensor_runtime_close(&cam_dev->runtime);
    sccb_deinit();
    sensor_reset_bank_state();
    s_active_device = RT_NULL;

    LOG_I("Camera device closed");
    return RT_EOK;
}

/**
 * @brief Trigger a single-shot capture and block until one frame is ready.
 *
 * Starts a data bus transfer into @p buffer then waits on @c frame_sem.
 * The wait is bounded by the shared runtime timeout configuration; a timeout
 * dumps diagnostic state and returns 0.
 *
 * @param buffer is the destination frame buffer (must be DMA-accessible).
 * @param size   is the buffer size in bytes.
 *
 * @return Return the number of bytes captured; 0 on timeout or error.
 */
static rt_size_t sensor_capture(void *buffer, rt_size_t size)
{
    return camera_sensor_runtime_capture(&s_device.runtime, buffer, size);
}

static int sensor_capture_async(void *buffer,
                                rt_size_t size,
                                camera_capture_done_callback_t callback,
                                void *context)
{
    return camera_sensor_runtime_capture_async(&s_device.runtime,
                                               buffer,
                                               size,
                                               callback,
                                               context);
}

/**
 * @brief Dispatch one internal @c OV2640_CMD_* request.
 *
 * Routes @p cmd to the appropriate sensor helper or bus-adapter operation.
 * This remains an internal implementation helper: the public integration
 * surface of the driver is the strong-typed @ref camera_device_ops_t table.
 *
 * @param cam_dev  Active OV2640 runtime instance.
 * @param cmd      One of the @c OV2640_CMD_* constants declared in ov2640.h.
 * @param args     Command argument; concrete type varies per command.
 *
 * @return Return RT_EOK on success; @c -RT_EINVAL for unknown commands or
 *         invalid arguments; @c -RT_ERROR if the underlying operation fails.
 */
static rt_err_t sensor_control(sensor_device_t *cam_dev, int cmd, void *args)
{
    int ret;
    
    if (cam_dev == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    switch (cmd)
    {
        case CMD_SET_PIXFORMAT:
        {
            pixformat_t format = (pixformat_t)(rt_ubase_t)args;

            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            ret = sensor_set_pixformat(cam_dev, format);
            if (ret != 0)
            {
                return -RT_ERROR;
            }
            return camera_sensor_runtime_set_pixformat(&cam_dev->runtime, format);
        }
        
        case CMD_SET_FRAMESIZE:
        {
            framesize_t framesize = (framesize_t)(rt_ubase_t)args;

            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            if (framesize >= FRAMESIZE_INVALID)
            {
                return -RT_EINVAL;
            }
            ret = sensor_set_framesize(cam_dev, framesize);
            if (ret != 0)
            {
                return -RT_ERROR;
            }
            return camera_sensor_runtime_set_framesize(&cam_dev->runtime, framesize);
        }
        
        case CMD_SET_QUALITY:
        {
            if (camera_sensor_runtime_busy(&cam_dev->runtime))
            {
                return -RT_EBUSY;
            }
            int quality = (int)(rt_base_t)args;
            ret = sensor_set_quality(cam_dev, quality);
            return (ret == 0) ? RT_EOK : -RT_ERROR;
        }

        default:
            return -RT_EINVAL;
    }
}

/**
 * @brief Apply a new pixel format through the internal control helper.
 */
static int sensor_apply_pixformat(pixformat_t pixformat)
{
    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,
                          CMD_SET_PIXFORMAT,
                          (void *)(rt_ubase_t)pixformat);
}

/**
 * @brief Apply a new frame size through the internal control helper.
 */
static int sensor_apply_framesize(framesize_t framesize)
{
    int ret;

    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    ret = sensor_control(&s_device,
                         CMD_SET_FRAMESIZE,
                         (void *)(rt_ubase_t)framesize);
    if (ret != RT_EOK)
    {
        return ret;
    }

    /* OV2640-specific settle window after format/size path updates. */
    rt_thread_mdelay(200);
    return RT_EOK;
}

/**
 * @brief Apply a new JPEG quality level through the internal control helper.
 */
static int sensor_apply_quality(uint8_t quality)
{
    if (!s_device.runtime.is_open)
    {
        return -RT_ERROR;
    }

    return sensor_control(&s_device,CMD_SET_QUALITY,(void *)(rt_ubase_t)quality);
}

/**
 * @brief Start continuous streaming using handle-supplied buffers.
 */
static int sensor_start_stream(const camera_stream_start_args_t *args)
{
    return camera_sensor_runtime_start_stream(&s_device.runtime, args);
}

/**
 * @brief Stop continuous streaming and clear driver stream state.
 */
static int sensor_stop_stream(void)
{
    return camera_sensor_runtime_stop_stream(&s_device.runtime);
}
