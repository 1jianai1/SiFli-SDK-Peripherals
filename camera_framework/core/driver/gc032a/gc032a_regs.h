/**
 * @file gc032a_regs.h
 * @brief GC032A registers used by the framework driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026
 */

#ifndef GC032A_REGS_H_
#define GC032A_REGS_H_

#define GC032A_REG_CHIP_ID_HIGH          0xF0
#define GC032A_REG_CHIP_ID_LOW           0xF1
#define GC032A_REG_PAGE_SELECT           0xFE

#define GC032A_REG_ROW_START_HIGH        0x09
#define GC032A_REG_ROW_START_LOW         0x0A
#define GC032A_REG_COLUMN_START_HIGH     0x0B
#define GC032A_REG_COLUMN_START_LOW      0x0C
#define GC032A_REG_WINDOW_HEIGHT_HIGH    0x0D
#define GC032A_REG_WINDOW_HEIGHT_LOW     0x0E
#define GC032A_REG_WINDOW_WIDTH_HIGH     0x0F
#define GC032A_REG_WINDOW_WIDTH_LOW      0x10
#define GC032A_REG_OUTPUT_FORMAT         0x44
#define GC032A_REG_WINDOW_MODE           0x50
#define GC032A_REG_OUTPUT_HEIGHT_HIGH    0x55
#define GC032A_REG_OUTPUT_HEIGHT_LOW     0x56
#define GC032A_REG_OUTPUT_WIDTH_HIGH     0x57
#define GC032A_REG_OUTPUT_WIDTH_LOW      0x58

#define GC032A_PAGE_0                    0x00
#define GC032A_SOFTWARE_RESET            0xF0
#define GC032A_OUTPUT_FORMAT_MASK        0x1F
#define GC032A_OUTPUT_FORMAT_RGB565      0x06
#define GC032A_OUTPUT_FORMAT_YUV422      0x03

#endif /* GC032A_REGS_H_ */
