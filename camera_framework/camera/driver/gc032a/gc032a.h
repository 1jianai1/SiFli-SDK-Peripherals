/**
 * @file gc032a.h
 * @brief GC032A camera sensor driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef GC032A_H_
#define GC032A_H_

#include "rtconfig.h"
#include "../../handle/camera_handle_internal.h"

#define GC032A_ADDR 0x21
#define GC032A_CHIP_ID 0x232A

extern const camera_device_ops_t gc032a_ops;

#endif /* GC032A_H_ */
