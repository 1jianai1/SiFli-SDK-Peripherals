#!/usr/bin/env python3

import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples/take_photo_to_sdcard"


def main():
    config = (EXAMPLE / "project/proj.conf").read_text()
    source = (EXAMPLE / "src/main.c").read_text()

    for setting in (
        "CONFIG_SENSOR_USING_GC032A=y",
        "CONFIG_CAMERA_GC032A_INTERFACE_SERIAL_2BIT=y",
    ):
        if setting not in config:
            raise AssertionError(f"missing example config: {setting}")

    for setting in (
        "CONFIG_CAMERA_SERIAL_SPI2",
        "CONFIG_BSP_USING_SPI2",
        "CONFIG_BSP_SPI2_RX_USING_DMA",
    ):
        if setting in config:
            raise AssertionError(f"obsolete serial-SPI config: {setting}")

    if source.count("MSH_CMD_EXPORT(take_photo,") != 1:
        raise AssertionError("take_photo must be the single synchronous command")
    if "take_rgb565" in source:
        raise AssertionError("sensor-specific take_rgb565 command remains")

    for text in (
        "caps_has_pixformat(caps, PIXFORMAT_JPEG)",
        "caps_has_pixformat(caps, PIXFORMAT_RGB565)",
        'cfg.pixformat = pixformat;',
        '"%s/photo_%03d.jpg"',
        '"%s/photo_%03d.ppm"',
        "save_rgb565_ppm",
        "MSH_CMD_EXPORT(take_photo_async",
    ):
        if text not in source:
            raise AssertionError(f"missing unified capture behavior: {text}")

    print("camera_sdcard_example_test: PASS")


if __name__ == "__main__":
    main()
