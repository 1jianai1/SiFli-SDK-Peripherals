/******************************************************************************
 * @file    main.c
 * @brief   Capture JPEG or RGB565 frames through the camera handle API and
 *          save them to the SD card.
 *
 * OV2640 frames are saved as JPEG. GC032A RGB565/VGA frames are converted to
 * PPM so the captured image can be inspected without a host-side converter.
 *
 * Call sequence (per `take_photo` invocation):
 *   camera_handler_instance_init()  - prepare handle state, register and open RT-Thread device
 *   camera_get_capabilities()       - select JPEG or RGB565 output
 *   camera_change_settings()        - configure format and frame size
 *   camera_capture_single() (loop)  - blocking single-frame grab
 *   save numbered JPEG or PPM files - write each captured frame
 *   camera_deinit()                 - close the device
 *
 * Low-level SCCB, image-data and XCLK pin muxing is handled by the framework.
 *****************************************************************************/

#include "rtthread.h"
#include "string.h"
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdevice.h>
#include "mem_section.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "camera_handle.h"

/* ------------------------------------------------------------------ *
 * PSRAM heap - holds camera frame buffers that are too large for SRAM.
 * ------------------------------------------------------------------ */
static uint8_t psram_heap_pool[4096 * 1024] L2_RET_BSS_SECT(psram_heap_pool);
static struct rt_memheap psram_memheap;

#define JPEG_MIN_BUFFER_SIZE    (64 * 1024)
#define JPEG_MAX_BUFFER_SIZE    (2 * 1024 * 1024)
#define PHOTO_DIR               "/photo"

/**
 * @brief Initialize the PSRAM heap pool.
 *
 * @return Return 0 on success (fixed value).
 */
static int psram_heap_init(void)
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
static void *psram_heap_malloc(uint32_t size)
{
    return rt_memheap_alloc(&psram_memheap, size);
}

/**
 * @brief Release memory previously returned by psram_heap_malloc().
 *
 * @param p is the pointer to free.
 */
static void psram_heap_free(void *p)
{
    rt_memheap_free(p);
}

/* ------------------------------------------------------------------ *
 * SD card mount
 * ------------------------------------------------------------------ */

/**
 * @brief Locate the SD card device and mount it as the root FAT volume.
 */
static void sdcard_init(void)
{
    rt_device_t msd = rt_device_find("sd0");
    if (msd == RT_NULL)
    {
        rt_kprintf("sd card not found\n");
        return;
    }

    if (dfs_mount("sd0", "/", "elm", 0, 0) != 0)
    {
        rt_kprintf("mount fs on tf card to / fail\n");
        rt_kprintf("sd card might not be formatted or is corrupted.\n");
        return;
    }

    rt_kprintf("mount fs on tf card to / success\n");
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
    else                                return FRAMESIZE_INVALID;
}

/**
 * @brief Resolve a framesize_t into pixel width and height.
 *
 * @param size   is the input framesize_t.
 * @param width  is the output pointer that receives the width in pixels.
 * @param height is the output pointer that receives the height in pixels.
 *
 * @return Return RT_EOK on success, or -RT_EINVAL when @p size is unknown.
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
        default:
            return -RT_EINVAL;
    }

    return RT_EOK;
}

/**
 * @brief Estimate a reasonable JPEG output buffer size for a given resolution.
 *
 * JPEG output length is variable; we use width * height as a rough upper
 * bound (about 1 byte per pixel for typical photographic content) and clamp
 * it to the [JPEG_MIN_BUFFER_SIZE, JPEG_MAX_BUFFER_SIZE] range.
 *
 * @param size is the target framesize_t.
 *
 * @return Return the buffer size in bytes (never zero).
 */
static rt_size_t calc_jpeg_buffer_size(framesize_t size)
{
    uint16_t width  = 0;
    uint16_t height = 0;
    if (framesize_to_resolution(size, &width, &height) != RT_EOK)
    {
        return JPEG_MAX_BUFFER_SIZE;
    }

    rt_size_t buf_size = (rt_size_t)width * (rt_size_t)height;
    if (buf_size < JPEG_MIN_BUFFER_SIZE)
    {
        buf_size = JPEG_MIN_BUFFER_SIZE;
    }
    if (buf_size > JPEG_MAX_BUFFER_SIZE)
    {
        buf_size = JPEG_MAX_BUFFER_SIZE;
    }
    return buf_size;
}

static rt_size_t calc_rgb565_buffer_size(framesize_t size,
                                         uint16_t *width,
                                         uint16_t *height)
{
    if (framesize_to_resolution(size, width, height) != RT_EOK)
    {
        return 0;
    }
    return (rt_size_t)(*width) * (rt_size_t)(*height) * 2U;
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
 * SD-card output path helpers
 * ------------------------------------------------------------------ */

/**
 * @brief Create the photo directory if it does not exist.
 *
 * @return Return RT_EOK on success or when the directory already exists.
 */
static int ensure_photo_dir(void)
{
    int ret = mkdir(PHOTO_DIR, 0);
    if (ret == 0)
    {
        return RT_EOK;
    }

    int err = rt_get_errno();
    if (ret < 0 && (err == EEXIST || err == -EEXIST))
    {
        return RT_EOK;
    }

    return -RT_ERROR;
}

static int save_photo(const char *path, const void *buffer, rt_size_t size)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
    {
        rt_kprintf("Failed to open %s for writing\n", path);
        return -RT_ERROR;
    }

    int written = write(fd, buffer, size);
    close(fd);
    if (written != (int)size)
    {
        rt_kprintf("Write failed for %s (%d/%u bytes)\n",
                   path, written, (unsigned int)size);
        return -RT_ERROR;
    }

    rt_kprintf("Saved %s (%u bytes)\n", path, (unsigned int)size);
    return RT_EOK;
}

static int save_rgb565_ppm(const char *path,
                           const uint8_t *buffer,
                           uint16_t width,
                           uint16_t height);

/* ------------------------------------------------------------------ *
 * MSH command: take_photo
 * ------------------------------------------------------------------ */

/**
 * @brief Capture one or more frames and save them to the SD card.
 *
 * Usage:
 *   take_photo <framesize> <quality> <count>
 *
 *   framesize : QQVGA / QCIF / QVGA / CIF / VGA / SVGA / XGA / HD / SXGA / UXGA
 *   quality   : JPEG quality (ignored for RGB565 sensors)
 *   count     : number of frames to capture (>= 1)
 *
 * Example:
 *   take_photo VGA 10 3
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
    rt_size_t                      buffer_size;
    pixformat_t                    pixformat;
    uint16_t                       width = 0;
    uint16_t                       height = 0;
    int                            quality;
    int                            count;

    if (argc != 4)
    {
        rt_kprintf("Usage: take_photo <framesize> <quality> <count>\n");
        rt_kprintf("Framesize options: QQVGA, QCIF, QVGA, CIF, VGA, SVGA, XGA, HD, SXGA, UXGA\n");
        rt_kprintf("quality: 0 (highest) to 63 (lowest)\n");
        rt_kprintf("count: number of photos to capture (>=1)\n");
        rt_kprintf("Example: take_photo VGA 10 3\n");
        return;
    }

    framesize_t framesize = format_string_to_framesize(argv[1]);
    if (framesize == FRAMESIZE_INVALID)
    {
        rt_kprintf("Unsupported framesize: %s\n", argv[1]);
        return;
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

    if (ensure_photo_dir() != RT_EOK)
    {
        rt_kprintf("Failed to create or access %s\n", PHOTO_DIR);
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
    if (!caps_has_framesize(caps, framesize))
    {
        rt_kprintf("Camera does not support requested framesize: %s\n", argv[1]);
        goto close_camera;
    }

    if (caps_has_pixformat(caps, PIXFORMAT_JPEG))
    {
        pixformat = PIXFORMAT_JPEG;
        buffer_size = caps->max_buffer_size != 0
                          ? caps->max_buffer_size
                          : calc_jpeg_buffer_size(framesize);
    }
    else if (caps_has_pixformat(caps, PIXFORMAT_RGB565))
    {
        pixformat = PIXFORMAT_RGB565;
        buffer_size = calc_rgb565_buffer_size(framesize, &width, &height);
        if (buffer_size == 0 || width > 640U)
        {
            rt_kprintf("Unsupported RGB565 framesize: %s\n", argv[1]);
            goto close_camera;
        }
        rt_kprintf("RGB565 capture ignores the quality argument\n");
    }
    else
    {
        rt_kprintf("Camera supports neither JPEG nor RGB565 capture\n");
        goto close_camera;
    }

    cfg.pixformat = pixformat;
    cfg.framesize = framesize;
    cfg.quality = (uint8_t)quality;
    status = camera_change_settings(camera_instance, &cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to configure camera (%d)\n", status);
        goto close_camera;
    }

    buffer = psram_heap_malloc(buffer_size);
    if (buffer == RT_NULL)
    {
        rt_kprintf("Failed to allocate %u bytes for capture\n",
                   (unsigned int)buffer_size);
        goto close_camera;
    }

    rt_kprintf("Capture: format=%s, framesize=%s, buffer=%u bytes @ %p\n",
               pixformat == PIXFORMAT_JPEG ? "JPEG" : "RGB565",
               argv[1], (unsigned int)buffer_size, buffer);

    for (int photo_idx = 0; photo_idx < count; photo_idx++)
    {
        req.buffer      = buffer;
        req.buffer_size = buffer_size;
        req.frame_size  = 0;

        status = camera_capture_single(camera_instance, &req);
        if (status != CAMERA_OK || req.frame_size == 0)
        {
            rt_kprintf("Capture failed or timed out (index=%d, status=%d)\n",
                       photo_idx, status);
            continue;
        }

        char file_path[64];
        if (pixformat == PIXFORMAT_JPEG)
        {
            rt_snprintf(file_path, sizeof(file_path),
                        "%s/photo_%03d.jpg", PHOTO_DIR, photo_idx + 1);
            save_photo(file_path, buffer, req.frame_size);
        }
        else if (req.frame_size == buffer_size)
        {
            rt_snprintf(file_path, sizeof(file_path),
                        "%s/photo_%03d.ppm", PHOTO_DIR, photo_idx + 1);
            save_rgb565_ppm(file_path, buffer, width, height);
        }
        else
        {
            rt_kprintf("Incomplete RGB565 frame: %u/%u bytes\n",
                       (unsigned int)req.frame_size,
                       (unsigned int)buffer_size);
        }
    }

    psram_heap_free(buffer);
    buffer = RT_NULL;

close_camera:
    camera_deinit(&camera_instance);
    if (buffer != RT_NULL)
    {
        psram_heap_free(buffer);
    }
}
MSH_CMD_EXPORT(take_photo, Capture photo(s) and save to SD card);

static int save_rgb565_ppm(const char *path,
                           const uint8_t *buffer,
                           uint16_t width,
                           uint16_t height)
{
    static uint8_t row[640U * 3U];
    char header[32];
    int header_size;
    int fd;
    uint16_t y;

    if (buffer == RT_NULL || width > 640U)
    {
        return -RT_EINVAL;
    }

    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
    {
        rt_kprintf("Failed to open %s for writing\n", path);
        return -RT_ERROR;
    }

    header_size = rt_snprintf(header,
                              sizeof(header),
                              "P6\n%u %u\n255\n",
                              width,
                              height);
    if (write(fd, header, header_size) != header_size)
    {
        close(fd);
        return -RT_ERROR;
    }

    for (y = 0; y < height; y++)
    {
        uint16_t x;
        const uint8_t *source = buffer + (uint32_t)y * width * 2U;

        for (x = 0; x < width; x++)
        {
            uint16_t pixel =
                (uint16_t)(((uint16_t)source[x * 2U] << 8) |
                           source[x * 2U + 1U]);
            uint8_t red = (uint8_t)((pixel >> 11) & 0x1fU);
            uint8_t green = (uint8_t)((pixel >> 5) & 0x3fU);
            uint8_t blue = (uint8_t)(pixel & 0x1fU);

            row[x * 3U] = (uint8_t)((red << 3) | (red >> 2));
            row[x * 3U + 1U] =
                (uint8_t)((green << 2) | (green >> 4));
            row[x * 3U + 2U] =
                (uint8_t)((blue << 3) | (blue >> 2));
        }

        if (write(fd, row, width * 3U) != (int)(width * 3U))
        {
            close(fd);
            return -RT_ERROR;
        }
    }

    close(fd);
    rt_kprintf("Saved %s (%u x %u RGB565 -> PPM)\n",
               path,
               width,
               height);
    return RT_EOK;
}

typedef struct
{
    camera_handler_instance_t *camera;
    uint8_t *buffer;
    volatile rt_bool_t busy;
} async_photo_context_t;

static async_photo_context_t s_async_photo;

static void take_photo_async_done(void *context,
                                  camera_handle_status_t status,
                                  rt_size_t frame_size)
{
    async_photo_context_t *photo = (async_photo_context_t *)context;

    if (status == CAMERA_OK && frame_size != 0)
    {
        if (ensure_photo_dir() == RT_EOK)
        {
            save_photo(PHOTO_DIR "/async_photo.jpg", photo->buffer, frame_size);
        }
        else
        {
            rt_kprintf("Async capture succeeded (%u bytes), but %s is unavailable\n",
                       (unsigned int)frame_size, PHOTO_DIR);
        }
    }
    else
    {
        rt_kprintf("Async capture failed (status=%d, size=%u)\n",
                   status, (unsigned int)frame_size);
    }

    camera_deinit(&photo->camera);
    psram_heap_free(photo->buffer);
    photo->buffer = RT_NULL;
    photo->busy = RT_FALSE;
    rt_kprintf("Async capture complete\n");
}

void take_photo_async(int argc, char **argv)
{
    const camera_capabilities_t *caps = RT_NULL;
    camera_capture_config_t cfg;
    camera_capture_request_t req;
    camera_handle_status_t status;
    framesize_t framesize;
    rt_size_t buffer_size;
    int quality;

    if (argc != 3)
    {
        rt_kprintf("Usage: take_photo_async <framesize> <quality>\n");
        return;
    }
    if (s_async_photo.busy)
    {
        rt_kprintf("Async capture already in progress\n");
        return;
    }

    framesize = format_string_to_framesize(argv[1]);
    quality = atoi(argv[2]);
    if (framesize == FRAMESIZE_INVALID || quality < 0 || quality > 63)
    {
        rt_kprintf("Invalid framesize or quality (0..63)\n");
        return;
    }
    s_async_photo.busy = RT_TRUE;
    status = camera_handler_instance_init(&s_async_photo.camera);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to initialize camera handler (%d)\n", status);
        goto fail;
    }

    status = camera_get_capabilities(s_async_photo.camera, &caps);
    if (status != CAMERA_OK || caps == RT_NULL ||
        !caps_has_pixformat(caps, PIXFORMAT_JPEG) ||
        !caps_has_framesize(caps, framesize))
    {
        rt_kprintf("Camera does not support requested JPEG mode\n");
        goto fail;
    }

    cfg.pixformat = PIXFORMAT_JPEG;
    cfg.framesize = framesize;
    cfg.quality = (uint8_t)quality;
    status = camera_change_settings(s_async_photo.camera, &cfg);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to configure camera (%d)\n", status);
        goto fail;
    }

    buffer_size = calc_jpeg_buffer_size(framesize);
    s_async_photo.buffer = psram_heap_malloc(buffer_size);
    if (s_async_photo.buffer == RT_NULL)
    {
        rt_kprintf("Failed to allocate %u bytes\n", (unsigned int)buffer_size);
        goto fail;
    }

    req.buffer = s_async_photo.buffer;
    req.buffer_size = buffer_size;
    req.frame_size = 0;
    status = camera_capture_single_async(s_async_photo.camera, &req,
                                         take_photo_async_done, &s_async_photo);
    if (status != CAMERA_OK)
    {
        rt_kprintf("Failed to start async capture (%d)\n", status);
        goto fail;
    }

    rt_kprintf("Async capture started; result will be saved to %s/async_photo.jpg\n",
               PHOTO_DIR);
    return;

fail:
    if (s_async_photo.camera != RT_NULL)
    {
        camera_deinit(&s_async_photo.camera);
    }
    if (s_async_photo.buffer != RT_NULL)
    {
        psram_heap_free(s_async_photo.buffer);
        s_async_photo.buffer = RT_NULL;
    }
    s_async_photo.busy = RT_FALSE;
}
MSH_CMD_EXPORT(take_photo_async, Capture one JPEG asynchronously and save to SD card);

/**
 * @brief Program entry point: initialize the PSRAM heap, mount the SD card
 *        and idle, waiting for MSH commands.
 *
 * @return Return 0 (never actually returns; the main loop spins forever).
 */
int main(void)
{
    rt_kprintf("Camera Take Photo to SD Card Example\n");
    psram_heap_init();
    sdcard_init();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}
