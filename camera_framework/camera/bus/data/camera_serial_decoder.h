#ifndef CAMERA_SERIAL_DECODER_H_
#define CAMERA_SERIAL_DECODER_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_SERIAL_DECODE_MORE         0
#define CAMERA_SERIAL_DECODE_FRAME_READY  1
#define CAMERA_SERIAL_DECODE_INVALID     -1
#define CAMERA_SERIAL_DECODE_OVERFLOW    -2
#define CAMERA_SERIAL_DECODE_BAD_FRAME   -3

typedef struct
{
    uint8_t pending[4];
    uint8_t pending_head;
    uint8_t pending_count;
    uint8_t line_prefix_remaining;
    uint8_t *frame_buffer;
    size_t frame_capacity;
    size_t frame_length;
    size_t line_start_length;
    uint16_t width;
    uint16_t height;
    uint16_t line_bytes;
    uint16_t completed_lines;
    uint8_t in_frame;
    uint8_t in_line;
} camera_serial_decoder_t;

void camera_serial_decoder_init(camera_serial_decoder_t *decoder,
                                uint16_t width,
                                uint16_t height);
int camera_serial_decoder_start(camera_serial_decoder_t *decoder,
                                void *buffer,
                                size_t capacity);
int camera_serial_decoder_feed(camera_serial_decoder_t *decoder,
                               const uint8_t *spi_bytes,
                               size_t byte_count,
                               size_t *frame_length);
void camera_serial_decoder_reset(camera_serial_decoder_t *decoder);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERIAL_DECODER_H_ */
