/**
 * @file    ov2640.h
 * @brief   OV2640 camera sensor driver types and internal control IDs
 *
 * This header provides the OV2640 sensor data structures, runtime state used
 * by the driver implementation, and the internal command IDs consumed by
 * ov2640.c helper dispatch.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef OV2640_H_
#define OV2640_H_

#include <stdint.h>
#include <stdbool.h>
#include "sccb.h"
#include "rtthread.h"
#include "rtconfig.h"
#include "../../handle/camera_handle_internal.h"
#include "camera_sensor_runtime.h"

#define OV2640_ADDR 0x30  /* 7-bit I2C/SCCB slave address */

/*
 *******************************************************************************
 * Data types
 *******************************************************************************
 */

/** @brief OV2640 driver runtime instance. */
typedef struct {
    camera_sensor_runtime_t runtime;
    rt_uint8_t current_bank;
} sensor_device_t;

/* Internal command IDs used by ov2640.c helper dispatch. Not part of the
 * application-facing API. */

/* --- Image format & resolution ------------------------------------------ */
#define CMD_SET_PIXFORMAT       0x01  /* pixformat_t  */
#define CMD_SET_FRAMESIZE       0x02  /* framesize_t  */

/* --- Image quality ------------------------------------------------------- */
#define CMD_SET_QUALITY         0x06  /* int  0…63   */

/*
 *******************************************************************************
 * Public API
 *******************************************************************************
 */

/** @brief OV2640 camera_device_ops table. */
extern const camera_device_ops_t ov2640_ops;

#endif /* OV2640_H_ */
