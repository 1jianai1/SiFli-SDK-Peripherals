#ifndef OV2640_JPEG_ASSEMBLER_H
#define OV2640_JPEG_ASSEMBLER_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    OV2640_JPEG_ERROR = -1,
    OV2640_JPEG_INCOMPLETE = 0,
    OV2640_JPEG_COMPLETE = 1,
} ov2640_jpeg_result_t;

typedef struct
{
    uint8_t *buffer;
    size_t capacity;
    size_t frame_size;
    uint8_t previous_byte;
    uint8_t previous_valid;
    uint8_t soi_found;
    uint8_t complete;
} ov2640_jpeg_assembler_t;

void ov2640_jpeg_assembler_reset(ov2640_jpeg_assembler_t *assembler,
                                 uint8_t *buffer,
                                 size_t capacity);

ov2640_jpeg_result_t ov2640_jpeg_assembler_feed(
    ov2640_jpeg_assembler_t *assembler,
    const uint8_t *segment,
    size_t segment_size);

#endif /* OV2640_JPEG_ASSEMBLER_H */
