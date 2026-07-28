# take_photo_to_sdcard（流式采集）示例

[English](README_EN.md)

## 概述

该示例使用 `camera_handle.h` 的流式 API 连续采集 JPEG 或 RGB565，并在
每次取到完整帧后立即写入 SD 卡。

当前 JPEG 流式路径为：

- DVP DMA 将数据写入内部环形缓冲
- handle 层 JPEG parser thread 识别完整帧
- `camera_get_stream_frame()` 向应用返回完整帧描述
- 应用立即写 `/photo/photo_NNN.jpg`

RGB565 流式路径使用 sensor 驱动提供的双缓冲帧，应用先复制完成帧，再转换并
保存为 `/photo/photo_NNN.ppm`，避免 SD 卡写入期间缓冲区被下一帧覆盖。

## 命令

```text
msh> take_photo <framesize|RGB565> <quality> <count>
```

## 选择摄像头

在 menuconfig 的 `Camera drivers -> Sensor settings -> Active camera sensor`
中选择摄像头，构建系统只会编译选中的驱动。OV2640 使用 JPEG；GC032A
serial 和 BF30A2 使用 RGB565。

参数：

- `framesize`：`QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA / 240X320`
- `RGB565`：自动选择当前 RGB565 摄像头唯一支持的分辨率
- `quality`：JPEG 质量，`0` 最好、`63` 压缩最强；RGB565 模式忽略该参数
- `count`：保存帧数，必须大于等于 `1`

## 调用流程

1. `camera_handler_instance_init()`
2. `camera_get_capabilities()`
3. 根据摄像头能力设置 `PIXFORMAT_JPEG` 或 `PIXFORMAT_RGB565`
4. 分配 PSRAM 缓冲并调用 `camera_start_stream()`
5. 循环 `camera_get_stream_frame()`，每帧保存到 SD 卡
6. `camera_stop_stream()`
7. `camera_deinit()`

## 双缓冲说明

JPEG 模式分配两块缓冲；RGB565 模式额外分配一块保存缓冲：

- `buffer[0]`：流式采集主缓冲
- `buffer[1]`：内部归一化工作缓冲
- `rgb565_save_buffer`：保存 PPM 时使用的稳定副本

这两块缓冲都是示例自己的 PSRAM 分配逻辑，不是框架提供的统一分配器。

## 备注

- ready-frame queue 深度当前为 `4`
- 如果 SD 写入速度跟不上，可能看到 `dseq` 跳变或丢帧统计增加
- JPEG 保存为 `.jpg`，RGB565 转换后保存为 `.ppm`
- 引脚复用由 camera framework 内部完成
