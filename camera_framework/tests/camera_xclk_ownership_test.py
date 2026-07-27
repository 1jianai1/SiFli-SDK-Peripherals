#!/usr/bin/env python3

import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]


def function_body(source, signature, next_signature):
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def check_sensor(path, open_signature, close_signature, next_signature):
    source = path.read_text()
    open_body = function_body(source, open_signature, close_signature)
    close_body = function_body(source, close_signature, next_signature)

    if open_body.index("camera_xclk_start") > open_body.index("sccb_init"):
        raise AssertionError(f"{path.name}: XCLK must start before SCCB")
    if open_body.count("camera_xclk_stop") < 3:
        raise AssertionError(f"{path.name}: every open failure must stop XCLK")
    if "camera_xclk_stop" not in close_body:
        raise AssertionError(f"{path.name}: close must stop XCLK")


def main():
    check_sensor(
        ROOT / "camera/driver/ov2640/ov2640.c",
        "static int sensor_open(void)\n{",
        "static int sensor_close(void)\n{",
        "static rt_size_t sensor_capture(",
    )
    check_sensor(
        ROOT / "camera/driver/gc032a/gc032a.c",
        "static int gc032a_open(void)\n{",
        "static int gc032a_close(void)\n{",
        "static int gc032a_set_pixformat(",
    )

    dvp_source = (ROOT / "camera/bus/data/dvp.c").read_text()
    if "camera_xclk_" in dvp_source:
        raise AssertionError("DVP adapter must not own the sensor clock")

    print("camera_xclk_ownership_test: PASS")


if __name__ == "__main__":
    main()
