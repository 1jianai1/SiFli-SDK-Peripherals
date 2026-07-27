#include "camera_xclk.h"

#include <stdint.h>
#include <stdio.h>

static int expect_period(uint32_t timer_hz,
                         uint32_t output_hz,
                         int expected_status,
                         uint32_t expected_period)
{
    uint32_t period = 0xa5a5a5a5U;
    int ret = camera_xclk_calculate_period(timer_hz, output_hz, &period);

    if (ret != expected_status)
    {
        printf("status mismatch: timer=%u output=%u got=%d expected=%d\n",
               (unsigned int)timer_hz,
               (unsigned int)output_hz,
               ret,
               expected_status);
        return 1;
    }
    if (ret == CAMERA_XCLK_OK && period != expected_period)
    {
        printf("period mismatch: timer=%u output=%u got=%u expected=%u\n",
               (unsigned int)timer_hz,
               (unsigned int)output_hz,
               (unsigned int)period,
               (unsigned int)expected_period);
        return 1;
    }
    return 0;
}

int main(void)
{
    if (expect_period(24000000U, 12000000U, CAMERA_XCLK_OK, 1U) != 0)
        return 1;
    if (expect_period(24000000U, 6000000U, CAMERA_XCLK_OK, 3U) != 0)
        return 1;
    if (expect_period(24000000U, 0U, CAMERA_XCLK_INVALID, 0U) != 0)
        return 1;
    if (expect_period(12000000U, 24000000U, CAMERA_XCLK_INVALID, 0U) != 0)
        return 1;
    if (expect_period(24000000U, 7000000U, CAMERA_XCLK_INVALID, 0U) != 0)
        return 1;
    if (expect_period(24000000U, 100U, CAMERA_XCLK_RANGE, 0U) != 0)
        return 1;

    puts("camera_xclk_test: PASS");
    return 0;
}
