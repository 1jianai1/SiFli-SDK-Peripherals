#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "camera_jpeg_assembler.h"

int main(void)
{
    uint8_t buffer[32] = {0x46, 0xE8, 0x66, 0x82,
                          0x35, 0xFF,
                          0xD8, 0x11, 0x22, 0xFF,
                          0xD9, 0x77};
    const uint8_t expected[] = {0xFF, 0xD8, 0x11, 0x22, 0xFF, 0xD9};
    camera_jpeg_assembler_t assembler;

    camera_jpeg_assembler_reset(&assembler, buffer, sizeof(buffer));
    assert(camera_jpeg_assembler_feed(&assembler, buffer, 4) ==
           CAMERA_JPEG_INCOMPLETE);
    assert(camera_jpeg_assembler_feed(&assembler, buffer + 4, 2) ==
           CAMERA_JPEG_INCOMPLETE);
    assert(camera_jpeg_assembler_feed(&assembler, buffer + 6, 4) ==
           CAMERA_JPEG_INCOMPLETE);
    assert(camera_jpeg_assembler_feed(&assembler, buffer + 10, 2) ==
           CAMERA_JPEG_COMPLETE);
    assert(assembler.frame_size == sizeof(expected));
    assert(memcmp(buffer, expected, sizeof(expected)) == 0);

    puts("camera jpeg assembler: PASS");
    return 0;
}
