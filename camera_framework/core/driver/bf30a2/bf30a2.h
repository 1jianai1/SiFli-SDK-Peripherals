/**
 * @file bf30a2.h
 * @brief BF30A2 camera sensor driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef BF30A2_H_
#define BF30A2_H_

#include "rtconfig.h"
#include "../../handle/camera_handle_internal.h"

#define BF30A2_ADDR        0x6e
#define BF30A2_CHIP_ID     0x3b02
#define BF30A2_WIDTH       240U
#define BF30A2_HEIGHT      320U
#define BF30A2_FRAME_SIZE  (BF30A2_WIDTH * BF30A2_HEIGHT * 2U)

extern const camera_device_ops_t bf30a2_ops;

#endif /* BF30A2_H_ */
