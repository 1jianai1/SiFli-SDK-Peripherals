/**
 * @file arducam_fifo_controller.c
 * @brief Arducam FIFO protocol and frame extraction helpers.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#include "arducam_fifo_controller.h"

typedef struct
{
    uint32_t size;
    uint8_t started;
    uint8_t finished;
    uint8_t saw_ff;
} arducam_fifo_jpeg_extractor_t;

static int arducam_fifo_ops_valid(const arducam_fifo_controller_ops_t *ops)
{
    return ops != NULL && ops->chip_write != NULL && ops->chip_read != NULL &&
           ops->fifo_begin != NULL && ops->fifo_read != NULL &&
           ops->fifo_end != NULL && ops->delay_ms != NULL;
}

static int arducam_fifo_cancelled(const arducam_fifo_controller_ops_t *ops,
                                  void *context)
{
    return ops->is_cancelled != NULL && ops->is_cancelled(context) != 0;
}

static uint32_t arducam_fifo_length(uint8_t low, uint8_t middle,
                                    uint8_t high)
{
    return ((uint32_t)(high & 0x7fU) << 16) | ((uint32_t)middle << 8) |
           (uint32_t)low;
}

static int arducam_fifo_wait_capture(const arducam_fifo_controller_ops_t *ops,
                                     void *context, uint32_t timeout_ms)
{
    uint32_t elapsed;
    uint8_t trigger;

    for (elapsed = 0U; elapsed <= timeout_ms; elapsed++)
    {
        if (arducam_fifo_cancelled(ops, context))
            return ARDUCAM_FIFO_ERROR_CANCELLED;
        if (ops->chip_read(context, ARDUCAM_FIFO_REG_TRIGGER, &trigger) != 0)
            return ARDUCAM_FIFO_ERROR_IO;
        if ((trigger & ARDUCAM_FIFO_CAPTURE_DONE_MASK) != 0U)
            return ARDUCAM_FIFO_OK;
        if (elapsed != timeout_ms)
            ops->delay_ms(context, 1U);
    }
    return ARDUCAM_FIFO_ERROR_TIMEOUT;
}

static int arducam_fifo_read_length(const arducam_fifo_controller_ops_t *ops,
                                    void *context, uint32_t *size)
{
    uint8_t low;
    uint8_t middle;
    uint8_t high;

    if (ops->chip_read(context, ARDUCAM_FIFO_REG_SIZE1, &low) != 0 ||
        ops->chip_read(context, ARDUCAM_FIFO_REG_SIZE2, &middle) != 0 ||
        ops->chip_read(context, ARDUCAM_FIFO_REG_SIZE3, &high) != 0)
        return ARDUCAM_FIFO_ERROR_IO;

    *size = arducam_fifo_length(low, middle, high);
    return *size > 0U && *size <= ARDUCAM_FIFO_MAX_SIZE ?
           ARDUCAM_FIFO_OK : ARDUCAM_FIFO_ERROR_LENGTH;
}

static int arducam_fifo_jpeg_feed(arducam_fifo_jpeg_extractor_t *extractor,
                                  const uint8_t *input, size_t input_size,
                                  uint8_t *output, size_t output_capacity,
                                  size_t *output_size)
{
    size_t input_index;

    *output_size = 0U;
    for (input_index = 0U; input_index < input_size; input_index++)
    {
        uint8_t value = input[input_index];

        if (!extractor->started)
        {
            if (extractor->saw_ff && value == 0xd8U)
            {
                if (output_capacity < 2U)
                    return ARDUCAM_FIFO_ERROR_LENGTH;
                output[(*output_size)++] = 0xffU;
                output[(*output_size)++] = 0xd8U;
                extractor->size += 2U;
                extractor->started = 1U;
                extractor->saw_ff = 0U;
            }
            else
            {
                extractor->saw_ff = value == 0xffU;
            }
            continue;
        }

        if (*output_size >= output_capacity)
            return ARDUCAM_FIFO_ERROR_LENGTH;
        output[(*output_size)++] = value;
        extractor->size++;
        if (extractor->saw_ff && value == 0xd9U)
        {
            extractor->finished = 1U;
            break;
        }
        extractor->saw_ff = value == 0xffU;
    }
    return ARDUCAM_FIFO_OK;
}

static int arducam_fifo_copy_raw(const arducam_fifo_controller_ops_t *ops,
                                 void *context, uint8_t *scratch,
                                 size_t scratch_size,
                                 arducam_fifo_write_t write,
                                 void *write_context, uint32_t size)
{
    uint32_t offset = 0U;

    while (offset < size)
    {
        size_t chunk = size - offset;

        if (arducam_fifo_cancelled(ops, context))
            return ARDUCAM_FIFO_ERROR_CANCELLED;
        if (chunk > scratch_size)
            chunk = scratch_size;
        if (ops->fifo_read(context, scratch, chunk) != 0)
            return ARDUCAM_FIFO_ERROR_IO;
        if (write(write_context, scratch, chunk) != 0)
            return ARDUCAM_FIFO_ERROR_WRITE;
        offset += (uint32_t)chunk;
    }
    return ARDUCAM_FIFO_OK;
}

static int arducam_fifo_copy_jpeg(const arducam_fifo_controller_ops_t *ops,
                                  void *context, uint8_t *scratch,
                                  size_t scratch_size,
                                  arducam_fifo_write_t write,
                                  void *write_context, uint32_t fifo_size,
                                  uint32_t *captured_size)
{
    arducam_fifo_jpeg_extractor_t extractor = {0};
    uint32_t offset = 0U;
    size_t input_capacity = (scratch_size - 1U) / 2U;
    uint8_t *output = scratch + input_capacity;
    size_t output_capacity = scratch_size - input_capacity;
    int result = ARDUCAM_FIFO_OK;

    while (offset < fifo_size && !extractor.finished)
    {
        size_t chunk = fifo_size - offset;
        size_t output_size;

        if (arducam_fifo_cancelled(ops, context))
            return ARDUCAM_FIFO_ERROR_CANCELLED;
        if (chunk > input_capacity)
            chunk = input_capacity;
        if (ops->fifo_read(context, scratch, chunk) != 0)
            return ARDUCAM_FIFO_ERROR_IO;
        offset += (uint32_t)chunk;
        result = arducam_fifo_jpeg_feed(&extractor, scratch, chunk, output,
                                        output_capacity, &output_size);
        if (result != ARDUCAM_FIFO_OK)
            return result;
        if (output_size > 0U && write(write_context, output, output_size) != 0)
            return ARDUCAM_FIFO_ERROR_WRITE;
    }

    if (!extractor.started || !extractor.finished || extractor.size < 4U)
        return ARDUCAM_FIFO_ERROR_JPEG;
    *captured_size = extractor.size;
    return ARDUCAM_FIFO_OK;
}

int arducam_fifo_controller_probe(const arducam_fifo_controller_ops_t *ops,
                                  void *context)
{
    uint8_t value;

    if (!arducam_fifo_ops_valid(ops))
        return ARDUCAM_FIFO_ERROR_ARGUMENT;
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_CPLD_RESET, 0x80U) != 0)
        return ARDUCAM_FIFO_ERROR_IO;
    ops->delay_ms(context, 100U);
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_CPLD_RESET, 0x00U) != 0)
        return ARDUCAM_FIFO_ERROR_IO;
    ops->delay_ms(context, 100U);
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_TEST,
                        ARDUCAM_FIFO_TEST_VALUE) != 0 ||
        ops->chip_read(context, ARDUCAM_FIFO_REG_TEST, &value) != 0 ||
        value != ARDUCAM_FIFO_TEST_VALUE)
        return ARDUCAM_FIFO_ERROR_IO;
    return ARDUCAM_FIFO_OK;
}

int arducam_fifo_controller_capture(
    const arducam_fifo_controller_ops_t *ops, void *context,
    bus_capture_mode_t mode, uint32_t expected_raw_size, uint8_t *scratch,
    size_t scratch_size, arducam_fifo_write_t write, void *write_context,
    uint32_t *captured_size, uint32_t timeout_ms)
{
    uint32_t fifo_size;
    int result;
    int fifo_open = 0;

    if (!arducam_fifo_ops_valid(ops) || scratch == NULL || scratch_size < 4U ||
        write == NULL || captured_size == NULL ||
        (mode != BUS_CAPTURE_MODE_JPEG && mode != BUS_CAPTURE_MODE_RGB565) ||
        (mode == BUS_CAPTURE_MODE_RGB565 &&
         (expected_raw_size == 0U ||
          expected_raw_size > ARDUCAM_FIFO_MAX_SIZE)))
        return ARDUCAM_FIFO_ERROR_ARGUMENT;

    *captured_size = 0U;
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_CONTROL,
                        ARDUCAM_FIFO_CLEAR_MASK) != 0)
        return ARDUCAM_FIFO_ERROR_IO;
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_CONTROL,
                        ARDUCAM_FIFO_START_MASK) != 0)
    {
        result = ARDUCAM_FIFO_ERROR_IO;
        goto clear_fifo;
    }
    result = arducam_fifo_wait_capture(ops, context, timeout_ms);
    if (result != ARDUCAM_FIFO_OK)
        goto clear_fifo;
    result = arducam_fifo_read_length(ops, context, &fifo_size);
    if (result != ARDUCAM_FIFO_OK)
        goto clear_fifo;
    if (mode == BUS_CAPTURE_MODE_RGB565 &&
        fifo_size != expected_raw_size &&
        (expected_raw_size > ARDUCAM_FIFO_MAX_SIZE -
         ARDUCAM_FIFO_RGB565_TRAILER ||
         fifo_size != expected_raw_size + ARDUCAM_FIFO_RGB565_TRAILER))
    {
        result = ARDUCAM_FIFO_ERROR_LENGTH;
        goto clear_fifo;
    }
    if (ops->fifo_begin(context) != 0)
    {
        result = ARDUCAM_FIFO_ERROR_IO;
        goto clear_fifo;
    }
    fifo_open = 1;
    if (mode == BUS_CAPTURE_MODE_RGB565)
    {
        result = arducam_fifo_copy_raw(ops, context, scratch, scratch_size,
                                       write, write_context,
                                       expected_raw_size);
        if (result == ARDUCAM_FIFO_OK)
            *captured_size = expected_raw_size;
    }
    else
    {
        result = arducam_fifo_copy_jpeg(ops, context, scratch, scratch_size,
                                        write, write_context, fifo_size,
                                        captured_size);
    }

clear_fifo:
    if (fifo_open && ops->fifo_end(context) != 0 && result == ARDUCAM_FIFO_OK)
        result = ARDUCAM_FIFO_ERROR_IO;
    if (ops->chip_write(context, ARDUCAM_FIFO_REG_CONTROL,
                        ARDUCAM_FIFO_CLEAR_MASK) != 0 &&
        result == ARDUCAM_FIFO_OK)
        result = ARDUCAM_FIFO_ERROR_IO;
    return result;
}
