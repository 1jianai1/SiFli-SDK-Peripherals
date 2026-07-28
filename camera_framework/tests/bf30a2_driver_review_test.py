#!/usr/bin/env python3
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
DRIVER = ROOT / "camera" / "driver" / "bf30a2" / "bf30a2.c"
HEADER = ROOT / "camera" / "driver" / "bf30a2" / "bf30a2.h"
KCONFIG = ROOT / "Kconfig"
SCONSCRIPT = ROOT / "SConscript"
HANDLE = ROOT / "camera" / "handle" / "camera_handle.h"


def read(path):
    return path.read_text(encoding="utf-8")


def test_bf30a2_uses_camera_handle_ops_only():
    source = read(DRIVER)
    assert "const camera_device_ops_t bf30a2_ops" in source
    assert "CAMERA_DRIVER_EXPORT(bf30a2, &bf30a2_ops)" in source
    assert ".capture_async = bf30a2_capture_async" in source
    assert ".start_stream = bf30a2_start_stream" in source
    assert "rt_device_register" not in source
    assert "MSH_CMD_EXPORT" not in source


def test_bf30a2_reports_real_capability():
    assert "FRAMESIZE_240X320" in read(HANDLE)
    source = read(DRIVER)
    assert "FRAMESIZE_240X320" in source
    assert ".max_buffer_size = BF30A2_FRAME_SIZE" in source
    assert "#define BF30A2_FRAME_SIZE" in read(HEADER)


def test_bf30a2_build_wiring_is_sensor_scoped():
    kconfig = read(KCONFIG)
    sconscript = read(SCONSCRIPT)
    assert "config SENSOR_USING_BF30A2" in kconfig
    assert "depends on SENSOR_USING_BF30A2" in kconfig
    assert "SENSOR_USING_BF30A2" in sconscript
    assert "camera/driver/bf30a2" in sconscript


def test_bf30a2_uses_original_control_pins():
    kconfig = read(KCONFIG)
    source = read(DRIVER)
    assert "default 30 if SENSOR_USING_BF30A2" in kconfig
    assert "default 33 if SENSOR_USING_BF30A2" in kconfig
    assert "default 9 if SENSOR_USING_BF30A2" in kconfig
    assert "PAD_PA00 + CAMERA_SCCB_SCL_PIN" in source
    assert "PAD_PA00 + CAMERA_SCCB_SDA_PIN" in source
    assert "CAMERA_SCCB_I2C_BUS_NAME" in source
    assert "camera_xclk_start(CAMERA_XCLK_PIN, CAMERA_XCLK_FREQ)" in source


def test_bf30a2_holds_spi_slave_selected():
    source = read(DRIVER)
    assert "HAL_PIN_Set(PAD_PA40, SPI2_CS, PIN_PULLDOWN, 1);" in source


def test_bf30a2_outputs_rgb565_msb_first():
    source = read(DRIVER)
    expected = """\
        *rgb++ = (uint8_t)(p0 >> 8);
        *rgb++ = (uint8_t)p0;
        *rgb++ = (uint8_t)(p1 >> 8);
        *rgb++ = (uint8_t)p1;"""
    assert expected in source


if __name__ == "__main__":
    test_bf30a2_uses_camera_handle_ops_only()
    test_bf30a2_reports_real_capability()
    test_bf30a2_build_wiring_is_sensor_scoped()
    test_bf30a2_uses_original_control_pins()
    test_bf30a2_holds_spi_slave_selected()
    test_bf30a2_outputs_rgb565_msb_first()
