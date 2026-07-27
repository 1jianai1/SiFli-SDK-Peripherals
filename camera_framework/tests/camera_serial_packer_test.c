#include "camera_serial_packer.h"

#include <stdint.h>
#include <stdio.h>

static int test_syncs_from_half_byte_offset_and_packs_lsb_first(void)
{
    static const uint8_t samples[] =
    {
        0x00, 0x00,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x40, 0x40, 0x00, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[4] = {0};
    size_t output_size = 0;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  samples,
                                  sizeof(samples),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0)
    {
        puts("pack failed");
        return 1;
    }
    if (output_size != sizeof(output) ||
        output[0] != 0xffU ||
        output[1] != 0xffU ||
        output[2] != 0xffU ||
        output[3] != 0xcaU)
    {
        printf("sync/pack mismatch: size=%u data=%02x %02x %02x %02x\n",
               (unsigned int)output_size, output[0], output[1],
               output[2], output[3]);
        return 1;
    }
    return 0;
}

static int test_preserves_sync_run_across_blocks(void)
{
    static const uint8_t sync_and_code[] =
    {
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x40, 0x40, 0x00, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[4] = {0};
    size_t output_size = 0;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  sync_and_code,
                                  7U,
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != 0U)
    {
        puts("partial feed failed");
        return 1;
    }
    if (camera_serial_packer_feed(&packer,
                                  sync_and_code + 7U,
                                  sizeof(sync_and_code) - 7U,
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != sizeof(output) ||
        output[3] != 0xcaU)
    {
        puts("cross-block synchronization failed");
        return 1;
    }
    return 0;
}

static int test_rejects_false_sync_candidate(void)
{
    static const uint8_t samples[] =
    {
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x00, 0x20, 0x40, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x40, 0x40, 0x00, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[4] = {0};
    size_t output_size = 0;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  samples,
                                  sizeof(samples),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != sizeof(output) ||
        output[3] != 0xcaU)
    {
        puts("false sync candidate was accepted");
        return 1;
    }
    return 0;
}

static int test_all_symbol_values_lsb_first(void)
{
    static const uint8_t samples[] =
    {
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x40, 0x40, 0x00, 0x60,
        0x00, 0x20, 0x40, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[5] = {0};
    size_t output_size = 0;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  samples,
                                  sizeof(samples),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != sizeof(output) ||
        output[3] != 0xcaU ||
        output[4] != 0xe4U)
    {
        puts("symbol mapping failed");
        return 1;
    }
    return 0;
}

static int test_rejects_overflow_without_consuming_input(void)
{
    static const uint8_t samples[] =
    {
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60,
        0x40, 0x40, 0x00, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[4] = {0};
    size_t output_size = 99U;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  samples,
                                  sizeof(samples),
                                  output,
                                  3U,
                                  &output_size) == 0 ||
        output_size != 0U ||
        packer.synchronized != 0U ||
        packer.ff_symbol_run != 0U)
    {
        puts("overflow was not rejected atomically");
        return 1;
    }
    if (camera_serial_packer_feed(&packer,
                                  samples,
                                  sizeof(samples),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != sizeof(output) ||
        output[3] != 0xcaU)
    {
        puts("retry after overflow failed");
        return 1;
    }
    return 0;
}

static int test_empty_and_reset(void)
{
    static const uint8_t partial_sync[] =
    {
        0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
        0x60, 0x60, 0x60, 0x60, 0x60,
    };
    static const uint8_t tail[] =
    {
        0x60, 0x40, 0x40, 0x00, 0x60,
    };
    camera_serial_packer_t packer;
    uint8_t output[4] = {0};
    size_t output_size = 99U;

    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  NULL,
                                  0U,
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != 0U)
    {
        puts("empty feed failed");
        return 1;
    }
    if (camera_serial_packer_feed(&packer,
                                  partial_sync,
                                  sizeof(partial_sync),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0)
    {
        puts("reset setup failed");
        return 1;
    }
    camera_serial_packer_reset(&packer);
    if (camera_serial_packer_feed(&packer,
                                  tail,
                                  sizeof(tail),
                                  output,
                                  sizeof(output),
                                  &output_size) != 0 ||
        output_size != 0U)
    {
        puts("reset retained synchronization state");
        return 1;
    }
    return 0;
}

int main(void)
{
    if (test_syncs_from_half_byte_offset_and_packs_lsb_first() != 0 ||
        test_preserves_sync_run_across_blocks() != 0 ||
        test_rejects_false_sync_candidate() != 0 ||
        test_all_symbol_values_lsb_first() != 0 ||
        test_rejects_overflow_without_consuming_input() != 0 ||
        test_empty_and_reset() != 0)
    {
        return 1;
    }

    puts("camera_serial_packer_test: PASS");
    return 0;
}
