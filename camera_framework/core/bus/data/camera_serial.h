#ifndef CAMERA_SERIAL_H_
#define CAMERA_SERIAL_H_

#include "camera_serial_decoder.h"
#include "camera_serial_packer.h"
#include "data_bus_adapter.h"
#include "rtthread.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_SERIAL_BUS_ADAPTER_NAME "camera_serial"

typedef struct
{
    bus_capture_mode_t mode;
    uint8_t *frame_buffer;
    uint32_t frame_buffer_size;
    uint8_t *sample_buffer;
    uint32_t sample_buffer_size;
    uint32_t sample_buffer_pool_size;
    uint8_t *packed_buffer;
    uint32_t packed_buffer_size;
} camera_serial_config_t;

typedef struct
{
    volatile uint8_t initialized;
    volatile uint8_t running;
    volatile uint8_t capture_enabled;
    volatile uint32_t completed_frames;
    volatile uint32_t bad_frames;
    volatile uint32_t dma_errors;
    volatile uint32_t dma_callbacks;
    volatile uint32_t dma_bytes;
    volatile uint32_t producer_sequence;
    volatile uint32_t consumer_sequence;
    volatile uint32_t overruns;
    volatile uint8_t worker_exit;
} camera_serial_state_t;

typedef struct
{
    camera_serial_config_t config;
    camera_serial_state_t state;
    camera_serial_packer_t packer;
    camera_serial_decoder_t decoder;
    rt_sem_t block_sem;
    rt_mutex_t lock;
    rt_thread_t worker;
    bus_frame_notify_callback_t callback;
    void *callback_data;
} camera_serial_handle_t;

int camera_serial_configure(bus_adapter_t *self,
                            const bus_adapter_config_t *config);
int camera_serial_init(bus_adapter_t *self);
int camera_serial_deinit(bus_adapter_t *self);
int camera_serial_start(bus_adapter_t *self);
int camera_serial_stop(bus_adapter_t *self);
int camera_serial_set_frame_notify_callback(
    bus_adapter_t *self,
    bus_frame_notify_callback_t callback,
    void *user_data);
int camera_serial_start_capture(bus_adapter_t *self,
                                void *buffer,
                                uint32_t size);
int camera_serial_rearm_capture(bus_adapter_t *self,
                                void *buffer,
                                uint32_t size);
int camera_serial_abort_capture(bus_adapter_t *self);
int camera_serial_set_pingpong_size(bus_adapter_t *self, uint32_t size);
int camera_serial_set_mode(bus_adapter_t *self, bus_capture_mode_t mode);
void camera_serial_dump_state(bus_adapter_t *self);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_SERIAL_H_ */
