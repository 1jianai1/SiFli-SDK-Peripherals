#include "camera_serial_decoder.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static int test_decodes_spi_packed_frame(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x9c, 0x63,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0x9c, 0x63 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("feed failed: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame))
    {
        printf("length mismatch: got=%u expected=%u\n",
               (unsigned int)frame_length,
               (unsigned int)sizeof(expected_frame));
        return 1;
    }
    if (memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("payload mismatch: got=%02x %02x\n", frame[0], frame[1]);
        return 1;
    }

    return 0;
}

static int test_preserves_partial_sync_across_feeds(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0xff, 0x63,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0xff, 0x63 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    size_t split = 7U;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("split start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     split,
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("first split returned: %d\n", ret);
        return 1;
    }
    ret = camera_serial_decoder_feed(&decoder,
                                     &wire_bytes[split],
                                     ARRAY_SIZE(wire_bytes) - split,
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("second split returned: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("split payload mismatch\n");
        return 1;
    }

    return 0;
}

static int test_keeps_marker_like_bytes_in_line_payload(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12, 0xff, 0xff, 0xff, 0x9d, 0x34,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] =
    {
        0x12, 0xff, 0xff, 0xff, 0x9d, 0x34,
    };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 3U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("marker payload start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("marker payload feed failed: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("marker payload mismatch\n");
        return 1;
    }

    return 0;
}

static int test_skips_line_number_prefix(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0x01, 0x80, 0x02, 0xe0, 0x01,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x9c, 0x63,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0x9c, 0x63 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("line number start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("line number feed failed: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("line number payload mismatch\n");
        return 1;
    }

    return 0;
}

static int test_skips_post_line_padding(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x9c, 0x63,
        0x20, 0x8b, 0x21, 0x7e, 0x21, 0x8b, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0x9c, 0x63 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("line padding start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("line padding feed failed: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("line padding payload mismatch\n");
        return 1;
    }

    return 0;
}

static int test_decodes_frame_without_line_end_markers(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x10, 0x11, 0x12, 0x13,
        0x00, 0x00, 0x00,
        0xff, 0xff, 0xff, 0x80,
        0x02, 0x00,
        0x20, 0x21, 0x22, 0x23,
        0x00, 0x00,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] =
    {
        0x10, 0x11, 0x12, 0x13,
        0x20, 0x21, 0x22, 0x23,
    };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 2U, 2U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("implicit line end start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("implicit line end feed failed: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("implicit line end payload mismatch\n");
        return 1;
    }
    return 0;
}

static int test_rejects_small_destination_buffer(void)
{
    uint8_t frame[1];
    camera_serial_decoder_t decoder;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_OVERFLOW)
    {
        printf("small buffer returned: %d\n", ret);
        return 1;
    }

    return 0;
}

static int test_rejects_mismatched_completed_line_count(void)
{
    static const uint8_t line_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12, 0x34,
        0xff, 0xff, 0xff, 0x9d,
    };
    static const uint8_t frame_end[] =
    {
        0xff, 0xff, 0xff, 0xb6,
    };
    uint8_t frame[2] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("line count start failed: %d\n", ret);
        return 1;
    }
    ret = camera_serial_decoder_feed(&decoder,
                                     line_bytes,
                                     ARRAY_SIZE(line_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_MORE ||
        decoder.completed_lines != 1U)
    {
        printf("line count setup failed: %d lines=%u\n",
               ret,
               (unsigned int)decoder.completed_lines);
        return 1;
    }

    decoder.completed_lines++;
    ret = camera_serial_decoder_feed(&decoder,
                                     frame_end,
                                     ARRAY_SIZE(frame_end),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_BAD_FRAME)
    {
        printf("mismatched line count returned: %d\n", ret);
        return 1;
    }

    return 0;
}

static int test_recovers_after_truncated_frame(void)
{
    static const uint8_t bad_wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12,
        0xff, 0xff, 0xff, 0x9d,
    };
    static const uint8_t good_wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12, 0x34,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0x12, 0x34 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("recovery start failed: %d\n", ret);
        return 1;
    }
    ret = camera_serial_decoder_feed(&decoder,
                                     bad_wire_bytes,
                                     ARRAY_SIZE(bad_wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("truncated frame returned: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     good_wire_bytes,
                                     ARRAY_SIZE(good_wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("recovery feed returned: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("recovery payload mismatch\n");
        return 1;
    }

    return 0;
}

static int test_accepts_long_post_line_padding(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12, 0x34,
        0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
        0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
        0x56,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0x12, 0x34 };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("overlong start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("long padding returned: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("long padding payload mismatch\n");
        return 1;
    }
    return 0;
}

static int test_recovers_within_same_dma_chunk(void)
{
    static const uint8_t wire_bytes[] =
    {
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0x12,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xca,
        0xff, 0xff, 0xff, 0x80,
        0x01, 0x00,
        0xab, 0xcd,
        0xff, 0xff, 0xff, 0x9d,
        0xff, 0xff, 0xff, 0xb6,
    };
    static const uint8_t expected_frame[] = { 0xab, 0xcd };
    uint8_t frame[sizeof(expected_frame)] = { 0 };
    camera_serial_decoder_t decoder;
    size_t frame_length = 0;
    int ret;

    camera_serial_decoder_init(&decoder, 1U, 1U);
    ret = camera_serial_decoder_start(&decoder, frame, sizeof(frame));
    if (ret != CAMERA_SERIAL_DECODE_MORE)
    {
        printf("same chunk start failed: %d\n", ret);
        return 1;
    }

    ret = camera_serial_decoder_feed(&decoder,
                                     wire_bytes,
                                     ARRAY_SIZE(wire_bytes),
                                     &frame_length);
    if (ret != CAMERA_SERIAL_DECODE_FRAME_READY)
    {
        printf("same chunk recovery returned: %d\n", ret);
        return 1;
    }
    if (frame_length != sizeof(expected_frame) ||
        memcmp(frame, expected_frame, sizeof(frame)) != 0)
    {
        printf("same chunk payload mismatch\n");
        return 1;
    }

    return 0;
}

int main(void)
{
    if (test_decodes_spi_packed_frame() != 0)
    {
        return 1;
    }
    if (test_preserves_partial_sync_across_feeds() != 0)
    {
        return 1;
    }
    if (test_keeps_marker_like_bytes_in_line_payload() != 0)
    {
        return 1;
    }
    if (test_skips_line_number_prefix() != 0)
    {
        return 1;
    }
    if (test_skips_post_line_padding() != 0)
    {
        return 1;
    }
    if (test_decodes_frame_without_line_end_markers() != 0)
    {
        return 1;
    }
    if (test_rejects_small_destination_buffer() != 0)
    {
        return 1;
    }
    if (test_rejects_mismatched_completed_line_count() != 0)
    {
        return 1;
    }
    if (test_recovers_after_truncated_frame() != 0)
    {
        return 1;
    }
    if (test_accepts_long_post_line_padding() != 0)
    {
        return 1;
    }
    if (test_recovers_within_same_dma_chunk() != 0)
    {
        return 1;
    }

    puts("camera_serial_decoder_test: PASS");
    return 0;
}
