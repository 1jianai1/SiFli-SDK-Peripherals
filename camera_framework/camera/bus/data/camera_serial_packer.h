#ifndef CAMERA_SERIAL_PACKER_H_
#define CAMERA_SERIAL_PACKER_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_SERIAL_PACK_OK       0
#define CAMERA_SERIAL_PACK_INVALID -1

typedef struct
{
    uint8_t partial_byte;
    uint8_t symbol_count;
    uint8_t synchronized;
    uint8_t ff_symbol_run;
    uint8_t candidate_byte;
    uint8_t candidate_symbols;
} camera_serial_packer_t;

void camera_serial_packer_reset(camera_serial_packer_t *packer);
int camera_serial_packer_feed(camera_serial_packer_t *packer,
                              const uint8_t *samples,
                              size_t sample_count,
                              uint8_t *output,
                              size_t output_capacity,
                              size_t *output_size);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERIAL_PACKER_H_ */
