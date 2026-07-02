#include "ov2640_jpeg_assembler.h"

void ov2640_jpeg_assembler_reset(ov2640_jpeg_assembler_t *assembler,
                                 uint8_t *buffer,
                                 size_t capacity)
{
    if (assembler == NULL)
    {
        return;
    }

    assembler->buffer = buffer;
    assembler->capacity = capacity;
    assembler->frame_size = 0;
    assembler->previous_byte = 0;
    assembler->previous_valid = 0;
    assembler->soi_found = 0;
    assembler->complete = 0;
}

ov2640_jpeg_result_t ov2640_jpeg_assembler_feed(
    ov2640_jpeg_assembler_t *assembler,
    const uint8_t *segment,
    size_t segment_size)
{
    size_t i;

    if (assembler == NULL || assembler->buffer == NULL ||
        assembler->capacity < 2 || segment == NULL || segment_size == 0)
    {
        return OV2640_JPEG_ERROR;
    }
    if (assembler->complete)
    {
        return OV2640_JPEG_COMPLETE;
    }

    if (assembler->soi_found)
    {
        uintptr_t buffer_addr = (uintptr_t)assembler->buffer;
        uintptr_t segment_addr = (uintptr_t)segment;
        if (segment_addr >= buffer_addr &&
            segment_addr < buffer_addr + assembler->capacity &&
            (size_t)(segment_addr - buffer_addr) < assembler->frame_size)
        {
            return OV2640_JPEG_ERROR;
        }
    }

    for (i = 0; i < segment_size; i++)
    {
        uint8_t byte = segment[i];

        if (!assembler->soi_found)
        {
            if (assembler->previous_valid &&
                assembler->previous_byte == 0xFFU && byte == 0xD8U)
            {
                assembler->buffer[0] = 0xFFU;
                assembler->buffer[1] = 0xD8U;
                assembler->frame_size = 2;
                assembler->soi_found = 1;
            }
            assembler->previous_byte = byte;
            assembler->previous_valid = 1;
            continue;
        }

        if (assembler->frame_size >= assembler->capacity)
        {
            return OV2640_JPEG_ERROR;
        }
        assembler->buffer[assembler->frame_size++] = byte;
        if (assembler->previous_byte == 0xFFU && byte == 0xD9U)
        {
            assembler->complete = 1;
            return OV2640_JPEG_COMPLETE;
        }
        assembler->previous_byte = byte;
        assembler->previous_valid = 1;
    }

    return OV2640_JPEG_INCOMPLETE;
}
