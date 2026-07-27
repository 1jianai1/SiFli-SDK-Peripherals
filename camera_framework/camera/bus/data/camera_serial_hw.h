#ifndef CAMERA_SERIAL_HW_H_
#define CAMERA_SERIAL_HW_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_SERIAL_HW_OK      0
#define CAMERA_SERIAL_HW_ERROR  -1

typedef void (*camera_serial_hw_block_callback_t)(uint32_t offset,
                                                   uint32_t size,
                                                   void *context);

int camera_serial_hw_init(camera_serial_hw_block_callback_t callback,
                          void *context);
int camera_serial_hw_start(uint8_t *buffer, uint32_t size);
int camera_serial_hw_stop(void);
void camera_serial_hw_deinit(void);
uint32_t camera_serial_hw_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERIAL_HW_H_ */
