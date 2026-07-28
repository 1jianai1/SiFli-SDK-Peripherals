#!/usr/bin/env python3
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
SYNC = ROOT / "examples" / "take_photo" / "src" / "main.c"
STREAM = (
    ROOT / "examples" / "take_photo_to_sdcard_streaming" / "src" / "main.c"
)


def read(path):
    return path.read_text(encoding="utf-8")


def test_sync_example_supports_fixed_rgb565_sensor():
    source = read(SYNC)
    for text in (
        "Usage: take_photo <framesize|RGB565> <quality> <count>",
        'strcmp(argv[1], "RGB565") == 0',
        "caps->num_framesizes != 1U",
        "framesize = caps->framesizes[0];",
        "FRAMESIZE_240X320",
        "camera_capture_single(camera_instance, &req)",
    ):
        assert text in source


def test_stream_example_supports_jpeg_and_rgb565():
    source = read(STREAM)
    for text in (
        "Usage: take_photo <framesize|RGB565> <quality> <count>",
        'strcmp(argv[1], "RGB565") == 0',
        "caps->num_framesizes != 1U",
        "framesize = caps->framesizes[0];",
        "FRAMESIZE_240X320",
        "PIXFORMAT_JPEG",
        "PIXFORMAT_RGB565",
        "camera_start_stream(camera_instance, &stream_cfg)",
        "camera_get_stream_frame(camera_instance, &frame",
        '"%s/photo_%03d.jpg"',
        '"%s/photo_%03d.ppm"',
        "save_rgb565_ppm",
        "rt_memcpy(rgb565_save_buffer, frame.buffer, buffer_size);",
    ):
        assert text in source
    assert "camera_capture_single(camera_instance" not in source


if __name__ == "__main__":
    test_sync_example_supports_fixed_rgb565_sensor()
    test_stream_example_supports_jpeg_and_rgb565()
    print("camera_examples_capability_test: PASS")
