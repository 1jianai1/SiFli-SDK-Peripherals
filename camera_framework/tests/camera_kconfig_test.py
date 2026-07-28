#!/usr/bin/env python3

import pathlib

import kconfiglib


ROOT = pathlib.Path(__file__).resolve().parents[1]
KCONFIG = ROOT / "Kconfig"


def load_config(sensor, interface=None):
    config = kconfiglib.Kconfig(str(KCONFIG), warn=False)
    config.syms["CAMERA_FRAMEWORK_ENABLE"].set_value(2)
    config.syms[sensor].set_value(2)
    if interface is not None:
        config.syms[interface].set_value(2)
    return config


def expect_value(config, symbol, expected):
    actual = config.syms[symbol].str_value
    if actual != expected:
        raise AssertionError(
            f"{symbol}: expected {expected}, got {actual}"
        )


def test_ov2640_selects_dvp():
    config = load_config("SENSOR_USING_OV2640")
    expect_value(config, "CAMERA_USING_DVP", "y")
    expect_value(config, "CAMERA_USING_SERIAL", "n")


def test_gc032a_dvp_selects_dvp():
    config = load_config(
        "SENSOR_USING_GC032A",
        "CAMERA_GC032A_INTERFACE_DVP_8BIT",
    )
    expect_value(config, "CAMERA_USING_DVP", "y")
    expect_value(config, "CAMERA_USING_SERIAL", "n")
    expect_value(config, "CAMERA_XCLK_FREQ", "12000000")


def test_gc032a_serial_selects_only_serial():
    config = load_config(
        "SENSOR_USING_GC032A",
        "CAMERA_GC032A_INTERFACE_SERIAL_2BIT",
    )
    expect_value(config, "CAMERA_USING_DVP", "n")
    expect_value(config, "CAMERA_USING_SERIAL", "y")
    expect_value(config, "CAMERA_SERIAL_SAMPLE_BUFFER_SIZE", "32768")
    expect_value(config, "CAMERA_SERIAL_WORKER_STACK_SIZE", "2048")
    expect_value(config, "CAMERA_SCCB_SCL_PIN", "30")
    expect_value(config, "CAMERA_SCCB_SDA_PIN", "33")
    expect_value(config, "CAMERA_XCLK_PIN", "9")
    expect_value(config, "CAMERA_XCLK_FREQ", "6000000")


def test_bf30a2_uses_24mhz_xclk():
    config = load_config("SENSOR_USING_BF30A2")
    expect_value(config, "CAMERA_XCLK_FREQ", "24000000")


def test_serial_kconfig_does_not_select_spi():
    source = KCONFIG.read_text()
    serial_block = source.split("if CAMERA_USING_SERIAL", 1)[1].split(
        "endif", 1
    )[0]
    for symbol in (
        "BSP_USING_SPI2",
        "BSP_SPI2_RX_USING_DMA",
        "BSP_USING_SPI_DMA_CIRCULAR",
        "CAMERA_SERIAL_SPI_BUS_NAME",
    ):
        if symbol in serial_block:
            raise AssertionError(
                f"serial GPIO-DMA still selects SPI: {symbol}"
            )


if __name__ == "__main__":
    test_ov2640_selects_dvp()
    test_gc032a_dvp_selects_dvp()
    test_gc032a_serial_selects_only_serial()
    test_bf30a2_uses_24mhz_xclk()
    test_serial_kconfig_does_not_select_spi()
    print("camera_kconfig_test: PASS")
