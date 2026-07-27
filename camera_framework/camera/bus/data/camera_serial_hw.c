#include "camera_serial_hw.h"

#include "bf0_hal.h"
#include "dma_config.h"
#include "drv_io.h"
#include <string.h>

#define CAMERA_SERIAL_DMA_INSTANCE DMA1_Channel5
#define CAMERA_SERIAL_DMA_PRIORITY 1U

typedef struct
{
    GPT_HandleTypeDef timer;
    DMA_HandleTypeDef dma;
    camera_serial_hw_block_callback_t callback;
    void *callback_context;
    uint32_t buffer_size;
    volatile uint32_t error_count;
    uint8_t initialized;
    uint8_t running;
} camera_serial_hw_state_t;

static camera_serial_hw_state_t s_camera_serial_hw;

static void camera_serial_hw_dma_half(DMA_HandleTypeDef *dma)
{
    camera_serial_hw_state_t *state =
        (camera_serial_hw_state_t *)dma->Parent;

    if (state != NULL && state->running && state->callback != NULL)
    {
        state->callback(0U,
                        state->buffer_size / 2U,
                        state->callback_context);
    }
}

static void camera_serial_hw_dma_full(DMA_HandleTypeDef *dma)
{
    camera_serial_hw_state_t *state =
        (camera_serial_hw_state_t *)dma->Parent;

    if (state != NULL && state->running && state->callback != NULL)
    {
        state->callback(state->buffer_size / 2U,
                        state->buffer_size / 2U,
                        state->callback_context);
    }
}

static void camera_serial_hw_dma_error(DMA_HandleTypeDef *dma)
{
    camera_serial_hw_state_t *state =
        (camera_serial_hw_state_t *)dma->Parent;

    if (state != NULL)
    {
        state->error_count++;
    }
}

static int camera_serial_hw_init_timer(camera_serial_hw_state_t *state)
{
    GPT_ClockConfigTypeDef clock_config;

    state->timer.Instance = GPTIM1;
    state->timer.core = CORE_ID_HCPU;
    state->timer.Init.Prescaler = 0U;
    state->timer.Init.CounterMode = GPT_COUNTERMODE_UP;
    state->timer.Init.Period = 0U;
    state->timer.Init.RepetitionCounter = 0U;
    if (HAL_GPT_Base_Init(&state->timer) != HAL_OK)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }

    memset(&clock_config, 0, sizeof(clock_config));
    clock_config.ClockSource = GPT_CLOCKSOURCE_ETRMODE2;
    clock_config.ClockPolarity = GPT_CLOCKPOLARITY_INVERTED;
    clock_config.ClockPrescaler = GPT_CLOCKPRESCALER_DIV1;
    clock_config.ClockFilter = 0U;
    if (HAL_GPT_ConfigClockSource(&state->timer, &clock_config) != HAL_OK)
    {
        HAL_GPT_Base_DeInit(&state->timer);
        return CAMERA_SERIAL_HW_ERROR;
    }

    return CAMERA_SERIAL_HW_OK;
}

static int camera_serial_hw_init_dma(camera_serial_hw_state_t *state)
{
    state->dma.Instance = CAMERA_SERIAL_DMA_INSTANCE;
    state->dma.Init.Request = GPTIM1_UPDATE_DMA_REQUEST;
    state->dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    state->dma.Init.PeriphInc = DMA_PINC_DISABLE;
    state->dma.Init.MemInc = DMA_MINC_ENABLE;
    state->dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    state->dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    state->dma.Init.Mode = DMA_CIRCULAR;
    state->dma.Init.Priority = DMA_PRIORITY_HIGH;
    state->dma.Init.BurstSize = 0U;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    state->dma.Init.IrqPrio = CAMERA_SERIAL_DMA_PRIORITY;
#endif
    state->dma.Parent = state;
    if (HAL_DMA_Init(&state->dma) != HAL_OK)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }

    state->dma.XferHalfCpltCallback = camera_serial_hw_dma_half;
    state->dma.XferCpltCallback = camera_serial_hw_dma_full;
    state->dma.XferErrorCallback = camera_serial_hw_dma_error;
    state->dma.XferAbortCallback = NULL;
    return CAMERA_SERIAL_HW_OK;
}

int camera_serial_hw_init(camera_serial_hw_block_callback_t callback,
                          void *context)
{
    camera_serial_hw_state_t *state = &s_camera_serial_hw;

    if (callback == NULL)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }
    if (state->initialized)
    {
        return CAMERA_SERIAL_HW_OK;
    }

    memset(state, 0, sizeof(*state));
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA39, GPTIM1_ETR, PIN_PULLDOWN, 1);
    HAL_RCC_EnableModule(RCC_MOD_GPTIM1);

    state->callback = callback;
    state->callback_context = context;
    if (camera_serial_hw_init_timer(state) != CAMERA_SERIAL_HW_OK)
    {
        HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
        return CAMERA_SERIAL_HW_ERROR;
    }
    if (camera_serial_hw_init_dma(state) != CAMERA_SERIAL_HW_OK)
    {
        HAL_GPT_Base_DeInit(&state->timer);
        HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
        return CAMERA_SERIAL_HW_ERROR;
    }

    state->initialized = 1U;
    return CAMERA_SERIAL_HW_OK;
}

int camera_serial_hw_start(uint8_t *buffer, uint32_t size)
{
    camera_serial_hw_state_t *state = &s_camera_serial_hw;

    if (!state->initialized || buffer == NULL || size <= 128U ||
        size > 0xffffU || (size & 1U) != 0U)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }
    if (state->running)
    {
        return CAMERA_SERIAL_HW_OK;
    }

    state->buffer_size = size;
    state->dma.XferHalfCpltCallback = camera_serial_hw_dma_half;
    state->dma.XferCpltCallback = camera_serial_hw_dma_full;
    state->dma.XferErrorCallback = camera_serial_hw_dma_error;
    if (HAL_DMA_Start_IT(&state->dma,
                         (uint32_t)((uint8_t *)hwp_gpio1 + 0x80),
                         (uint32_t)buffer,
                         size) != HAL_OK)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }

    state->running = 1U;
    __HAL_GPT_SET_COUNTER(&state->timer, 0U);
    __HAL_GPT_ENABLE_DMA(&state->timer, GPT_DMA_UPDATE);
    __HAL_GPT_ENABLE(&state->timer);
    return CAMERA_SERIAL_HW_OK;
}

int camera_serial_hw_stop(void)
{
    camera_serial_hw_state_t *state = &s_camera_serial_hw;

    if (!state->initialized || !state->running)
    {
        return CAMERA_SERIAL_HW_OK;
    }

    __HAL_GPT_DISABLE_DMA(&state->timer, GPT_DMA_UPDATE);
    __HAL_GPT_DISABLE(&state->timer);
    state->running = 0U;
    if (HAL_DMA_Abort(&state->dma) != HAL_OK)
    {
        return CAMERA_SERIAL_HW_ERROR;
    }
    return CAMERA_SERIAL_HW_OK;
}

void camera_serial_hw_deinit(void)
{
    camera_serial_hw_state_t *state = &s_camera_serial_hw;

    if (!state->initialized)
    {
        return;
    }

    camera_serial_hw_stop();
    HAL_DMA_DeInit(&state->dma);
    HAL_GPT_Base_DeInit(&state->timer);
    HAL_RCC_DisableModule(RCC_MOD_GPTIM1);
    memset(state, 0, sizeof(*state));
}

uint32_t camera_serial_hw_error_count(void)
{
    return s_camera_serial_hw.error_count;
}
