/******************************************************************************
 * @file    main.c
 * @brief   Capture RGB565 frames through the camera handle API.
 *
 * The example uses the camera_handle.h API to grab one or more raw RGB565
 * frames into a PSRAM buffer. It then prints the buffer address and size so
 * the captured pixels can be exported from RAM with the SDK helper scripts
 * (e.g. format_sram.py) and viewed as an image on the host.
 *
 * Call sequence:
 *   camera_handler_instance_init()  - prepare handle state, register and open RT-Thread device
 *   camera_change_settings()        - configure RGB565 + framesize
 *   camera_capture_single()         - blocking single-frame grab (looped)
 *   camera_deinit()                 - close the device
 *
 * Low-level SCCB, image-data and XCLK pin muxing is handled by the framework.
 *****************************************************************************/

#include "rtthread.h"
#include "bf0_hal.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <rtdevice.h>
#include "mem_section.h"
#include "camera_handle.h"

/* ------------------------------------------------------------------ *
 * PSRAM heap - holds the RGB565 frame buffer (internal SRAM is too
 * small for anything beyond QVGA).
 * ------------------------------------------------------------------ */
static uint8_t psram_heap_pool[4096 * 1024] L2_RET_BSS_SECT(psram_heap_pool);
static struct rt_memheap psram_memheap;

/**
 * @brief Initialize the PSRAM heap pool.
 *
 * @return Return 0 on success (fixed value).
 */
int psram_heap_init(void)
{
    rt_memheap_init(&psram_memheap, "psram_heap", (void *)psram_heap_pool,
                    sizeof(psram_heap_pool));
    return 0;
}

/**
 * @brief Allocate memory from the PSRAM heap.
 *
 * @param size is the number of bytes to allocate.
 *
 * @return Return a pointer on success, RT_NULL on failure.
 */
void *psram_heap_malloc(uint32_t size)
{
    return rt_memheap_alloc(&psram_memheap, size);
}

/**
 * @brief Release memory previously returned by psram_heap_malloc().
 *
 * @param p is the pointer to free.
 */
void psram_heap_free(void *p)
{
    rt_memheap_free(p);
}

/* ------------------------------------------------------------------ *
 * Frame-size parsing and buffer-size computation
 * ------------------------------------------------------------------ */

/**
 * @brief Convert a frame-size name (e.g. "VGA") into the framesize_t enum.
 *
 * @param str is the frame-size string to look up.
 *
 * @return Return the matching framesize_t, or FRAMESIZE_INVALID if unknown.
 */
static framesize_t format_string_to_framesize(const char *str)
{
    if (strcmp(str, "QQVGA") == 0)      return FRAMESIZE_QQVGA;
    else if (strcmp(str, "QCIF") == 0)  return FRAMESIZE_QCIF;
    else if (strcmp(str, "QVGA") == 0)  return FRAMESIZE_QVGA;
    else if (strcmp(str, "CIF") == 0)   return FRAMESIZE_CIF;
    else if (strcmp(str, "VGA") == 0)   return FRAMESIZE_VGA;
    else if (strcmp(str, "SVGA") == 0)  return FRAMESIZE_SVGA;
    else if (strcmp(str, "XGA") == 0)   return FRAMESIZE_XGA;
    else if (strcmp(str, "HD") == 0)    return FRAMESIZE_HD;
    else if (strcmp(str, "SXGA") == 0)  return FRAMESIZE_SXGA;
    else if (strcmp(str, "UXGA") == 0)  return FRAMESIZE_UXGA;
    else if (strcmp(str, "240X320") == 0) return FRAMESIZE_240X320;
    else                                return FRAMESIZE_INVALID;
}

/**
 * @brief Resolve a framesize_t into pixel width and height.
 *
 * @param size   is the input framesize_t.
 * @param width  is the output pointer that receives the width in pixels.
 * @param height is the output pointer that receives the height in pixels.
 *
 * @return Return RT_EOK on success, or -RT_EINVAL when @p size is unknown
 *         or one of the output pointers is RT_NULL.
 */
static int framesize_to_resolution(framesize_t size, uint16_t *width, uint16_t *height)
{
    if (width == RT_NULL || height == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (size)
    {
        case FRAMESIZE_QQVGA: *width = 160;  *height = 120;  break;
        case FRAMESIZE_QCIF:  *width = 176;  *height = 144;  break;
        case FRAMESIZE_QVGA:  *width = 320;  *height = 240;  break;
        case FRAMESIZE_CIF:   *width = 400;  *height = 296;  break;
        case FRAMESIZE_VGA:   *width = 640;  *height = 480;  break;
        case FRAMESIZE_SVGA:  *width = 800;  *height = 600;  break;
        case FRAMESIZE_XGA:   *width = 1024; *height = 768;  break;
        case FRAMESIZE_HD:    *width = 1280; *height = 720;  break;
        case FRAMESIZE_SXGA:  *width = 1280; *height = 1024; break;
        case FRAMESIZE_UXGA:  *width = 1600; *height = 1200; break;
        case FRAMESIZE_240X320: *width = 240; *height = 320; break;
        default:
            return -RT_EINVAL;
    }

    return RT_EOK;
}

/**
 * @brief Compute the exact RGB565 frame size in bytes for a given resolution.
 *
 * RGB565 is two bytes per pixel, so the buffer size is simply
 * width * height * 2.
 *
 * @param size   is the target framesize_t.
 * @param width  is the output pointer that receives the resolved width.
 * @param height is the output pointer that receives the resolved height.
 *
 * @return Return the buffer size in bytes, or 0 if @p size is unsupported.
 */
static rt_size_t calc_rgb565_buffer_size(framesize_t size,
                                         uint16_t *width, uint16_t *height)
{
    if (framesize_to_resolution(size, width, height) != RT_EOK)
    {
        return 0;
    }
    return (rt_size_t)(*width) * (rt_size_t)(*height) * 2u;
}

static rt_bool_t caps_has_pixformat(const camera_capabilities_t *caps, pixformat_t fmt)
{
    rt_uint8_t i;
    if (caps == RT_NULL || caps->pixformats == RT_NULL)
    {
        return RT_FALSE;
    }
    for (i = 0; i < caps->num_pixformats; i++)
    {
        if (caps->pixformats[i] == fmt)
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

static rt_bool_t caps_has_framesize(const camera_capabilities_t *caps, framesize_t size)
{
    rt_uint8_t i;
    if (caps == RT_NULL || caps->framesizes == RT_NULL)
    {
        return RT_FALSE;
    }
    for (i = 0; i < caps->num_framesizes; i++)
    {
        if (caps->framesizes[i] == size)
        {
            return RT_TRUE;
        }
    }
    return RT_FALSE;
}

/* ------------------------------------------------------------------ *
 * MSH command: take_photo
 * ------------------------------------------------------------------ */

/**
 * @brief MSH command: capture one or more RGB565 frames and print their
 *        PSRAM addresses so they can be exported with the SDK helper scripts.
 *
 * Usage:
 *   take_photo <framesize|RGB565> <quality> <count>
 *
 *   framesize : QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
 *   quality   : ignored for RGB565; use 0
 *   count     : number of frames to capture (>= 1)
 *
 * Example:
 *   take_photo RGB565 0 1
 *
 * After the capture finishes, the command prints the buffer base address,
 * the resolved width / height, and the byte count. The host can then dump
 * that memory region (e.g. via sftool / J-Link savebin) and feed it into
 * the SDK conversion script to render the raw RGB565 buffer as an image.
 *
 * @param argc is the argument count (4 including the command name).
 * @param argv is the argument vector.
 */
void take_photo(int argc, char **argv)
{
    camera_handler_instance_t     *camera_instance = RT_NULL;
    const camera_capabilities_t   *caps = RT_NULL;
    camera_capture_config_t        cfg;
    camera_capture_request_t       req;
    camera_handle_status_t         status;
    uint8_t                       *buffer = RT_NULL;
    uint16_t                       width  = 0;
    uint16_t                       height = 0;
    rt_size_t                      buffer_size;
    framesize_t                    framesize;
    rt_bool_t                      auto_rgb565;
    int                            quality;
    int                            count;

    if (argc != 4)
    {
        rt_kprintf("Usage: take_photo <framesize|RGB565> <quality> <count>\n");
        rt_kprintf("Framesize options: QQVGA, QCIF, QVGA, CIF, VGA, SVGA, XGA, HD, SXGA, UXGA, 240X320\n");
        rt_kprintf("RGB565 selects the camera's only supported framesize\n");
        rt_kprintf("quality: ignored for RGB565; use 0\n");
        rt_kprintf("count: number of RGB565 frames to capture (>=1)\n");
        rt_kprintf("Example: take_photo RGB565 0 1\n");
        return;
    }

    auto_rgb565 = strcmp(argv[1], "RGB565") == 0;
    framesize = FRAMESIZE_INVALID;
    if (!auto_rgb565)
    {
        framesize = format_string_to_framesize(argv[1]);
        if (framesize == FRAMESIZE_INVALID)
        {
            rt_kprintf("Unsupported framesize or format: %s\n", argv[1]);
            return;
        }
    }

    quality = atoi(argv[2]);
    if (quality < 0 || quality > 63)
    {
        rt_kprintf("Quality must be between 0 and 63\n");
        return;
    }

    count = atoi(argv[3]);
    if (count <= 0)
    {
        rt_kprintf("Count must be >= 1\n");
        return;
    }

    /* 1) Initialise the camera handler instance.  The driver is selected at
     *    compile time via Kconfig and the RT-Thread
     *    device is registered internally. */
    status = camera_handler_instance_init(&camera_instance);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to initialize camera handler (%d)\n", status);
        return;
    }

    status = camera_get_capabilities(camera_instance, &caps);
    if (status != CAMERA_OK || caps == RT_NULL)
    {
        rt_kprintf("Failed to query camera capabilities (%d)\n", status);
        goto close_camera;
    }
    if (!caps_has_pixformat(caps, PIXFORMAT_RGB565))
    {
        rt_kprintf("Camera does not support PIXFORMAT_RGB565\n");
        goto close_camera;
    }
    if (auto_rgb565)
    {
        if (caps->framesizes == RT_NULL || caps->num_framesizes != 1U)
        {
            rt_kprintf("RGB565 auto mode requires exactly one framesize\n");
            goto close_camera;
        }
        framesize = caps->framesizes[0];
    }
    else if (!caps_has_framesize(caps, framesize))
    {
        rt_kprintf("Camera does not support requested framesize: %s\n", argv[1]);
        goto close_camera;
    }

    buffer_size = calc_rgb565_buffer_size(framesize, &width, &height);
    if (buffer_size == 0)
    {
        rt_kprintf("Failed to resolve framesize\n");
        goto close_camera;
    }
    if (caps->max_buffer_size != 0 && buffer_size > caps->max_buffer_size)
    {
        rt_kprintf("Requested RGB565 frame needs %u bytes, capability max=%u\n",
                   (unsigned int)buffer_size, (unsigned int)caps->max_buffer_size);
        goto close_camera;
    }

    /* 2) Push RGB565 + framesize configuration. The quality field is
     *    validated for command compatibility but ignored by RGB565 drivers. */
    cfg.pixformat = PIXFORMAT_RGB565;
    cfg.framesize = framesize;
    cfg.quality   = (uint8_t)quality;
    status = camera_change_settings(camera_instance, &cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to configure camera (%d)\n", status);
        goto close_camera;
    }

    /* 4) Allocate one RGB565 frame buffer in PSRAM. */
    buffer = psram_heap_malloc(buffer_size);
    if (buffer == RT_NULL)
    {
        rt_kprintf("Failed to allocate %u bytes for RGB565 capture!\n",
                   (unsigned int)buffer_size);
        goto close_camera;
    }

    rt_kprintf("RGB565 capture: %ux%u, %u bytes/frame, buffer @ %p\n",
               (unsigned int)width, (unsigned int)height,
               (unsigned int)buffer_size, buffer);

    /* 5) Grab `count` frames sequentially; the same buffer is reused each
     *    iteration so only the last frame survives in PSRAM. */
    for (int photo_idx = 0; photo_idx < count; photo_idx++)
    {
        req.buffer      = buffer;
        req.buffer_size = buffer_size;
        req.frame_size  = 0;

        status = camera_capture_single(camera_instance, &req);
        if (status != CAMERA_OK)
        {
            rt_kprintf("Capture failed or timed out (index=%d, status=%d)\n",
                       photo_idx, status);
            continue;
        }

        rt_kprintf("Frame %d captured: %u bytes @ %p (RGB565 %ux%u)\n",
                   photo_idx + 1, (unsigned int)req.frame_size,
                   buffer, (unsigned int)width, (unsigned int)height);
    }

    rt_kprintf("Export the buffer with the SDK script, e.g.:\n"
               "  sftool ... read_mem %p %u rgb565.bin\n",
               buffer, (unsigned int)buffer_size);
        
    psram_heap_free(buffer);
    buffer = RT_NULL;

close_camera:
    camera_deinit(&camera_instance);
    if (buffer != RT_NULL)
    {
        psram_heap_free(buffer);
    }
}
MSH_CMD_EXPORT(take_photo, Capture RGB565 frame(s));

/**
 * @brief Program entry point: initialize the PSRAM heap and idle, waiting
 *        for MSH commands.
 *
 * Capture is triggered through the `take_photo` command on the serial
 * console.
 *
 * @return Return 0 (never actually returns; the main loop spins forever).
 */
int main(void)
{
    rt_kprintf("Camera Take Photo Example (RGB565)\n");
    psram_heap_init();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
