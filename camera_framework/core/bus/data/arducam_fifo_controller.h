/**
 * @file arducam_fifo_controller.h
 * @brief Arducam FIFO protocol and frame extraction interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef __ARDUCAM_FIFO_CONTROLLER_H__
#define __ARDUCAM_FIFO_CONTROLLER_H__

#include <stddef.h>
#include <stdint.h>

#include "data_bus_adapter.h"

#define ARDUCAM_FIFO_REG_TEST       0x00U
#define ARDUCAM_FIFO_REG_CONTROL    0x04U
#define ARDUCAM_FIFO_REG_CPLD_RESET 0x07U
#define ARDUCAM_FIFO_REG_TRIGGER    0x41U
#define ARDUCAM_FIFO_REG_SIZE1      0x42U
#define ARDUCAM_FIFO_REG_SIZE2      0x43U
#define ARDUCAM_FIFO_REG_SIZE3      0x44U

#define ARDUCAM_FIFO_CLEAR_MASK         0x01U
#define ARDUCAM_FIFO_START_MASK         0x02U
#define ARDUCAM_FIFO_CAPTURE_DONE_MASK  0x08U
#define ARDUCAM_FIFO_TEST_VALUE         0x55U
#define ARDUCAM_FIFO_MAX_SIZE           0x00800000U
#define ARDUCAM_FIFO_RGB565_TRAILER     8U

enum
{
    ARDUCAM_FIFO_OK = 0,
    ARDUCAM_FIFO_ERROR_ARGUMENT = -1,
    ARDUCAM_FIFO_ERROR_IO = -2,
    ARDUCAM_FIFO_ERROR_TIMEOUT = -3,
    ARDUCAM_FIFO_ERROR_LENGTH = -4,
    ARDUCAM_FIFO_ERROR_JPEG = -5,
    ARDUCAM_FIFO_ERROR_WRITE = -6,
    ARDUCAM_FIFO_ERROR_CANCELLED = -7,
};

typedef int (*arducam_fifo_write_t)(void *context, const uint8_t *data,
                                    size_t size);

typedef struct
{
    int (*chip_write)(void *context, uint8_t reg, uint8_t value);
    int (*chip_read)(void *context, uint8_t reg, uint8_t *value);
    int (*fifo_begin)(void *context);
    int (*fifo_read)(void *context, uint8_t *data, size_t size);
    int (*fifo_end)(void *context);
    void (*delay_ms)(void *context, uint32_t milliseconds);
    int (*is_cancelled)(void *context);
} arducam_fifo_controller_ops_t;

int arducam_fifo_controller_probe(const arducam_fifo_controller_ops_t *ops,
                                  void *context);
int arducam_fifo_controller_capture(
    const arducam_fifo_controller_ops_t *ops, void *context,
    bus_capture_mode_t mode, uint32_t expected_raw_size, uint8_t *scratch,
    size_t scratch_size, arducam_fifo_write_t write, void *write_context,
    uint32_t *captured_size, uint32_t timeout_ms);

#endif /* __ARDUCAM_FIFO_CONTROLLER_H__ */
