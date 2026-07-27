#include "camera_xclk.h"

#include <stddef.h>

int camera_xclk_calculate_period(uint32_t timer_hz,
                                 uint32_t output_hz,
                                 uint32_t *period)
{
    uint32_t calculated_period;

    if (timer_hz == 0U || output_hz == 0U || period == NULL ||
        output_hz > timer_hz || timer_hz % output_hz != 0U)
    {
        return CAMERA_XCLK_INVALID;
    }

    calculated_period = timer_hz / output_hz - 1U;
    if (calculated_period < 1U || calculated_period > 0xffffU)
    {
        return CAMERA_XCLK_RANGE;
    }

    *period = calculated_period;
    return CAMERA_XCLK_OK;
}
