#include "camera_serial_decoder.h"

#include <string.h>

#define GC032A_FRAME_START_CODE 0xcaU
#define GC032A_LINE_START_CODE  0x80U
#define GC032A_LINE_END_CODE    0x9dU
#define GC032A_FRAME_END_CODE   0xb6U
#define GC032A_LINE_NUMBER_BYTES 2U

static void camera_serial_decoder_drop_frame(
    camera_serial_decoder_t *decoder)
{
    decoder->pending_head = 0;
    decoder->pending_count = 0;
    decoder->in_frame = 0;
    decoder->in_line = 0;
    decoder->frame_length = 0;
    decoder->completed_lines = 0;
    decoder->line_prefix_remaining = 0;
}

static uint8_t camera_serial_decoder_trailing_ff(
    const camera_serial_decoder_t *decoder)
{
    uint8_t count = 0U;

    while (count < 3U &&
           decoder->pending[
               (decoder->pending_head + 3U - count) & 3U] == 0xffU)
    {
        count++;
    }
    return count;
}

static int camera_serial_decoder_finish_marker(
    camera_serial_decoder_t *decoder,
    uint8_t code,
    size_t *frame_length)
{
    size_t expected_frame_length;

    switch (code)
    {
        case GC032A_FRAME_START_CODE:
            decoder->frame_length = 0;
            decoder->line_start_length = 0;
            decoder->completed_lines = 0;
            decoder->in_frame = 1;
            decoder->in_line = 0;
            decoder->line_prefix_remaining = 0;
            return CAMERA_SERIAL_DECODE_MORE;

        case GC032A_LINE_START_CODE:
            if (!decoder->in_frame)
            {
                return CAMERA_SERIAL_DECODE_BAD_FRAME;
            }
            if (decoder->in_line)
            {
                /* Previous line never received a LINE_END marker.
                 * Close it implicitly so the new line can start. */
                decoder->completed_lines++;
                decoder->in_line = 0;
                decoder->line_prefix_remaining = 0;
            }
            decoder->line_start_length = decoder->frame_length;
            decoder->in_line = 1;
            decoder->line_prefix_remaining = GC032A_LINE_NUMBER_BYTES;
            return CAMERA_SERIAL_DECODE_MORE;

        case GC032A_LINE_END_CODE:
            if (!decoder->in_frame || !decoder->in_line ||
                decoder->line_prefix_remaining != 0U ||
                decoder->frame_length - decoder->line_start_length !=
                    decoder->line_bytes)
            {
                return CAMERA_SERIAL_DECODE_BAD_FRAME;
            }
            decoder->completed_lines++;
            decoder->in_line = 0;
            decoder->line_prefix_remaining = 0;
            return CAMERA_SERIAL_DECODE_MORE;

        case GC032A_FRAME_END_CODE:
            if (decoder->in_line)
            {
                /* Last line never received a LINE_END.  Close it
                 * implicitly so the frame can complete. */
                decoder->completed_lines++;
                decoder->in_line = 0;
                decoder->line_prefix_remaining = 0;
            }
            expected_frame_length =
                (size_t)decoder->line_bytes * decoder->height;
            if (!decoder->in_frame ||
                decoder->completed_lines != decoder->height ||
                decoder->frame_length != expected_frame_length)
            {
                return CAMERA_SERIAL_DECODE_BAD_FRAME;
            }
            decoder->in_frame = 0;
            *frame_length = decoder->frame_length;
            return CAMERA_SERIAL_DECODE_FRAME_READY;

        default:
            return CAMERA_SERIAL_DECODE_MORE;
    }
}

static int camera_serial_decoder_emit_payload(
    camera_serial_decoder_t *decoder,
    uint8_t value)
{
    if (!decoder->in_line)
    {
        return CAMERA_SERIAL_DECODE_MORE;
    }
    if (decoder->line_prefix_remaining > 0U)
    {
        decoder->line_prefix_remaining--;
        return CAMERA_SERIAL_DECODE_MORE;
    }
    /* After the expected line_bytes have been written to the frame
     * buffer, ignore any further payload bytes until a line-end or
     * frame-end marker resets the state.  The GC032A may emit
     * blanking / padding data between active pixel data and the
     * end-of-line marker, and those bytes must not overwrite valid
     * pixel data or inflate frame_length. */
    if (decoder->frame_length - decoder->line_start_length >=
        decoder->line_bytes)
    {
        return CAMERA_SERIAL_DECODE_MORE;
    }
    if (decoder->frame_length >= decoder->frame_capacity)
    {
        return CAMERA_SERIAL_DECODE_OVERFLOW;
    }

    decoder->frame_buffer[decoder->frame_length++] = value;
    return CAMERA_SERIAL_DECODE_MORE;
}

static int camera_serial_decoder_process_byte(
    camera_serial_decoder_t *decoder,
    uint8_t value,
    size_t *frame_length)
{
    int ret;
    uint8_t tail;
    uint8_t code;

    if (decoder->in_line &&
        (decoder->line_prefix_remaining > 0U ||
         decoder->frame_length - decoder->line_start_length <
             decoder->line_bytes))
    {
        return camera_serial_decoder_emit_payload(decoder, value);
    }

    tail = (uint8_t)((decoder->pending_head + decoder->pending_count) & 3U);
    decoder->pending[tail] = value;
    decoder->pending_count++;
    if (decoder->pending_count < sizeof(decoder->pending))
    {
        return CAMERA_SERIAL_DECODE_MORE;
    }

    if (decoder->pending[decoder->pending_head] == 0xffU &&
        decoder->pending[(decoder->pending_head + 1U) & 3U] == 0xffU &&
        decoder->pending[(decoder->pending_head + 2U) & 3U] == 0xffU)
    {
        code = decoder->pending[(decoder->pending_head + 3U) & 3U];

        if (code == GC032A_FRAME_START_CODE ||
            code == GC032A_LINE_START_CODE ||
            code == GC032A_LINE_END_CODE ||
            code == GC032A_FRAME_END_CODE)
        {
            decoder->pending_head = 0;
            decoder->pending_count = 0;
            ret = camera_serial_decoder_finish_marker(decoder,
                                                       code,
                                                       frame_length);
            if (ret == CAMERA_SERIAL_DECODE_BAD_FRAME)
            {
                camera_serial_decoder_drop_frame(decoder);
            }
            return ret;
        }
    }

    if (decoder->in_line)
    {
        /* Consume blanking / padding bytes that appear between the
         * end of active pixel data and the next marker.  The GC032A
         * may emit a variable-length horizontal blanking period
         * (often much longer than 16 bytes).  Slide the pending
         * window one byte at a time and keep looking for a valid
         * marker without an artificial padding limit. */
        uint8_t trailing_ff =
            camera_serial_decoder_trailing_ff(decoder);

        decoder->pending_head =
            (uint8_t)((decoder->pending_head + 1U) & 3U);
        decoder->pending_count =
            sizeof(decoder->pending) - 1U;
        while (decoder->pending_count < trailing_ff)
        {
            decoder->pending[decoder->pending_count++] = 0xffU;
        }
        return CAMERA_SERIAL_DECODE_MORE;
    }

    ret = camera_serial_decoder_emit_payload(
        decoder,
        decoder->pending[decoder->pending_head]);
    decoder->pending_head = (uint8_t)((decoder->pending_head + 1U) & 3U);
    decoder->pending_count = sizeof(decoder->pending) - 1U;
    return ret;
}

void camera_serial_decoder_init(camera_serial_decoder_t *decoder,
                                uint16_t width,
                                uint16_t height)
{
    if (decoder == NULL)
    {
        return;
    }

    memset(decoder, 0, sizeof(*decoder));
    decoder->width = width;
    decoder->height = height;
    decoder->line_bytes = (uint16_t)(width * 2U);
}

void camera_serial_decoder_reset(camera_serial_decoder_t *decoder)
{
    if (decoder == NULL)
    {
        return;
    }

    decoder->pending_head = 0;
    decoder->pending_count = 0;
    decoder->line_prefix_remaining = 0;
    decoder->frame_buffer = NULL;
    decoder->frame_capacity = 0;
    decoder->frame_length = 0;
    decoder->line_start_length = 0;
    decoder->completed_lines = 0;
    decoder->in_frame = 0;
    decoder->in_line = 0;
}

int camera_serial_decoder_start(camera_serial_decoder_t *decoder,
                                void *buffer,
                                size_t capacity)
{
    size_t expected_frame_length;

    if (decoder == NULL || buffer == NULL ||
        decoder->width == 0U || decoder->height == 0U)
    {
        return CAMERA_SERIAL_DECODE_INVALID;
    }

    expected_frame_length = (size_t)decoder->line_bytes * decoder->height;
    if (capacity < expected_frame_length)
    {
        return CAMERA_SERIAL_DECODE_OVERFLOW;
    }

    camera_serial_decoder_reset(decoder);
    decoder->frame_buffer = (uint8_t *)buffer;
    decoder->frame_capacity = capacity;
    return CAMERA_SERIAL_DECODE_MORE;
}

int camera_serial_decoder_feed(camera_serial_decoder_t *decoder,
                               const uint8_t *spi_bytes,
                               size_t byte_count,
                               size_t *frame_length)
{
    size_t i;
    int last_result = CAMERA_SERIAL_DECODE_MORE;

    if (decoder == NULL || spi_bytes == NULL || frame_length == NULL ||
        decoder->frame_buffer == NULL)
    {
        return CAMERA_SERIAL_DECODE_INVALID;
    }

    for (i = 0; i < byte_count; i++)
    {
        int ret = camera_serial_decoder_process_byte(decoder,
                                                     spi_bytes[i],
                                                     frame_length);

        if (ret == CAMERA_SERIAL_DECODE_FRAME_READY ||
            ret == CAMERA_SERIAL_DECODE_INVALID ||
            ret == CAMERA_SERIAL_DECODE_OVERFLOW)
        {
            return ret;
        }
        if (ret == CAMERA_SERIAL_DECODE_BAD_FRAME)
        {
            last_result = ret;
        }
    }

    return last_result;
}
