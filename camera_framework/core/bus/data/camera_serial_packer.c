#include "camera_serial_packer.h"

static int camera_serial_is_sync_code(uint8_t value)
{
    return value == 0xcaU || value == 0x80U ||
           value == 0x9dU || value == 0xb6U;
}

static uint8_t camera_serial_trailing_ff_symbols(uint8_t value)
{
    uint8_t count = 0U;
    int shift;

    for (shift = 6; shift >= 0; shift -= 2)
    {
        if (((value >> shift) & 0x03U) != 0x03U)
        {
            break;
        }
        count++;
    }
    return count;
}

void camera_serial_packer_reset(camera_serial_packer_t *packer)
{
    if (packer == NULL)
    {
        return;
    }

    packer->partial_byte = 0;
    packer->symbol_count = 0;
    packer->synchronized = 0;
    packer->ff_symbol_run = 0;
    packer->candidate_byte = 0;
    packer->candidate_symbols = 0;
}

int camera_serial_packer_feed(camera_serial_packer_t *packer,
                              const uint8_t *samples,
                              size_t sample_count,
                              uint8_t *output,
                              size_t output_capacity,
                              size_t *output_size)
{
    camera_serial_packer_t next;
    size_t written = 0;
    size_t index;

    if (output_size != NULL)
    {
        *output_size = 0;
    }
    if (packer == NULL || output_size == NULL ||
        (sample_count > 0U && samples == NULL))
    {
        return CAMERA_SERIAL_PACK_INVALID;
    }

    next = *packer;
    for (index = 0; index < sample_count; index++)
    {
        uint8_t symbol = (uint8_t)((samples[index] >> 5) & 0x03U);

        if (!next.synchronized)
        {
            if (next.candidate_symbols > 0U)
            {
                next.candidate_byte |=
                    (uint8_t)(symbol <<
                              (next.candidate_symbols * 2U));
                next.candidate_symbols++;
                if (next.candidate_symbols < 4U)
                {
                    continue;
                }
                if (camera_serial_is_sync_code(
                        next.candidate_byte))
                {
                    if (output == NULL ||
                        output_capacity - written < 4U)
                    {
                        return CAMERA_SERIAL_PACK_INVALID;
                    }
                    output[written++] = 0xffU;
                    output[written++] = 0xffU;
                    output[written++] = 0xffU;
                    output[written++] = next.candidate_byte;
                    next.synchronized = 1U;
                    next.ff_symbol_run = 0U;
                    next.candidate_byte = 0U;
                    next.candidate_symbols = 0U;
                    continue;
                }
                next.ff_symbol_run =
                    camera_serial_trailing_ff_symbols(
                        next.candidate_byte);
                next.candidate_byte = 0U;
                next.candidate_symbols = 0U;
                continue;
            }
            if (symbol == 0x03U)
            {
                if (next.ff_symbol_run < 12U)
                {
                    next.ff_symbol_run++;
                }
                continue;
            }
            if (next.ff_symbol_run < 12U)
            {
                next.ff_symbol_run = 0U;
                continue;
            }
            next.candidate_byte = symbol;
            next.candidate_symbols = 1U;
            next.ff_symbol_run = 0U;
            continue;
        }

        next.partial_byte |=
            (uint8_t)(symbol << (next.symbol_count * 2U));
        next.symbol_count++;
        if (next.symbol_count == 4U)
        {
            if (output == NULL || written >= output_capacity)
            {
                return CAMERA_SERIAL_PACK_INVALID;
            }
            output[written++] = next.partial_byte;
            next.partial_byte = 0U;
            next.symbol_count = 0U;
        }
    }

    *packer = next;
    *output_size = written;
    return CAMERA_SERIAL_PACK_OK;
}
