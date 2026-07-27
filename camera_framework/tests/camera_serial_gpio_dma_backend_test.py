#!/usr/bin/env python3

import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
HW_SOURCE = ROOT / "camera/bus/data/camera_serial_hw.c"
ADAPTER_SOURCE = ROOT / "camera/bus/data/camera_serial.c"
RUNTIME_SOURCE = ROOT / "camera/driver/camera_sensor_runtime.c"


def require(source, tokens, subject):
    for token in tokens:
        if token not in source:
            raise AssertionError(f"missing {subject}: {token}")


def forbid(source, tokens, subject):
    for token in tokens:
        if token in source:
            raise AssertionError(f"{subject} remains: {token}")


def main():
    hardware = HW_SOURCE.read_text()
    adapter = ADAPTER_SOURCE.read_text()
    runtime = RUNTIME_SOURCE.read_text()

    require(
        hardware,
        (
            "GPT_CLOCKSOURCE_ETRMODE2",
            "GPTIM1_UPDATE_DMA_REQUEST",
            "DMA1_Channel5",
            "DMA_CIRCULAR",
            "HAL_DMA_Start_IT",
        ),
        "GPIO-DMA behavior",
    )
    forbid(
        hardware + adapter,
        (
            "rt_hw_spi_device_attach",
            "rt_spi_transfer",
            "RT_SPI_CTRL_CONFIG_DMA_CIRCULAR",
            "struct rt_spi_device",
        ),
        "ordinary SPI code",
    )
    require(
        adapter,
        (
            "camera_serial_hw_start",
            "camera_serial_packer_feed",
            "camera_serial_worker",
            "CAMERA_SERIAL_MAX_BLOCKS_PER_WAKE",
            "producer_sequence",
            "consumer_sequence",
            "overruns",
        ),
        "serial adapter behavior",
    )
    forbid(
        adapter,
        (
            "sample_bytes",
            "packed_sample_bytes",
            "first_nonzero_packed",
            "sync_candidates",
            "first_sync_code",
            "first_bad_",
            "frame_start_count",
        ),
        "bring-up diagnostics",
    )

    callback_registration = runtime.index(
        "bus_adapter_set_frame_notify_callback"
    )
    adapter_init = runtime.index("bus_adapter_init", callback_registration)
    if callback_registration >= adapter_init:
        raise AssertionError("runtime must register callback before init")

    init_body = adapter[
        adapter.index("int camera_serial_init"):
        adapter.index("int camera_serial_deinit")
    ]
    reset = init_body.index("rt_memset(handle, 0, sizeof(*handle))")
    for saved, restored in (
        ("callback = handle->callback", "handle->callback = callback"),
        ("callback_data = handle->callback_data",
         "handle->callback_data = callback_data"),
    ):
        if not (init_body.index(saved) < reset < init_body.index(restored)):
            raise AssertionError("camera_serial_init clears frame callback")

    print("camera_serial_gpio_dma_backend_test: PASS")


if __name__ == "__main__":
    main()
