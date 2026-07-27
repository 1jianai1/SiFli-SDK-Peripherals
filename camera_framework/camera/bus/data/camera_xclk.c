#include "camera_xclk.h"

#include "bf0_hal.h"
#include "drv_io.h"
#include "rtthread.h"

#define DBG_TAG "camera.xclk"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static GPT_HandleTypeDef s_xclk_gptim;
static rt_bool_t s_xclk_initialized = RT_FALSE;
static int s_xclk_pin = -1;

int camera_xclk_start(int pin, uint32_t frequency_hz)
{
    HAL_StatusTypeDef status;
    GPT_OC_InitTypeDef output_config = {0};
    uint32_t timer_hz;
    uint32_t period;
    int ret;

    if (pin < 0 || pin > 44)
    {
        return CAMERA_XCLK_INVALID;
    }

    if (s_xclk_initialized)
    {
        ret = camera_xclk_stop(s_xclk_pin);
        if (ret != CAMERA_XCLK_OK)
        {
            return ret;
        }
    }

#if defined(SOC_SF32LB52X) && SOC_SF32LB52X == 1
    timer_hz = 24000000U;
#else
    timer_hz = HAL_RCC_GetPCLKFreq(s_xclk_gptim.core, 1);
#endif

    ret = camera_xclk_calculate_period(timer_hz, frequency_hz, &period);
    if (ret != CAMERA_XCLK_OK)
    {
        LOG_E("invalid frequency %u Hz (timer=%u Hz)",
              (unsigned int)frequency_hz,
              (unsigned int)timer_hz);
        return ret;
    }

    HAL_PIN_Set(PAD_PA00 + pin, GPTIM2_CH1, PIN_NOPULL, 1);
    HAL_RCC_EnableModule(RCC_MOD_GPTIM2);

    s_xclk_gptim.Instance = hwp_gptim2;
    s_xclk_gptim.Init.Prescaler = 0;
    s_xclk_gptim.Init.CounterMode = GPT_COUNTERMODE_UP;
    s_xclk_gptim.Init.Period = period;

    status = HAL_GPT_Base_Init(&s_xclk_gptim);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 base init failed: %d", status);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    output_config.OCMode = GPT_OCMODE_PWM1;
    output_config.Pulse = period / 2U + 1U;
    output_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    output_config.OCFastMode = GPT_OCFAST_DISABLE;
    status = HAL_GPT_PWM_ConfigChannel(&s_xclk_gptim,
                                       &output_config,
                                       GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 PWM config failed: %d", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    status = HAL_GPT_PWM_Start(&s_xclk_gptim, GPT_CHANNEL_1);
    if (status != HAL_OK)
    {
        LOG_E("GPTIM2 PWM start failed: %d", status);
        HAL_GPT_Base_DeInit(&s_xclk_gptim);
        HAL_PIN_Set(PAD_PA00 + pin, GPIO_A0 + pin, PIN_NOPULL, 1);
        return CAMERA_XCLK_HW;
    }

    s_xclk_initialized = RT_TRUE;
    s_xclk_pin = pin;
    rt_thread_mdelay(10);
    LOG_I("%u Hz on PA%d (period=%u)",
          (unsigned int)frequency_hz,
          pin,
          (unsigned int)period);
    return CAMERA_XCLK_OK;
}

int camera_xclk_stop(int pin)
{
    HAL_StatusTypeDef pwm_status;
    HAL_StatusTypeDef base_status;
    int active_pin;

    if (!s_xclk_initialized)
    {
        return CAMERA_XCLK_OK;
    }

    active_pin = s_xclk_pin;
    pwm_status = HAL_GPT_PWM_Stop(&s_xclk_gptim, GPT_CHANNEL_1);
    base_status = HAL_GPT_Base_DeInit(&s_xclk_gptim);
    s_xclk_initialized = RT_FALSE;
    s_xclk_pin = -1;

    if (pin >= 0 && pin <= 44)
    {
        active_pin = pin;
    }
    HAL_PIN_Set(PAD_PA00 + active_pin,
                GPIO_A0 + active_pin,
                PIN_NOPULL,
                1);
    if (pwm_status != HAL_OK || base_status != HAL_OK)
    {
        LOG_E("stop failed: pwm=%d base=%d", pwm_status, base_status);
        return CAMERA_XCLK_HW;
    }

    LOG_I("stopped (PA%d)", active_pin);
    return CAMERA_XCLK_OK;
}
