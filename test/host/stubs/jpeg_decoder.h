/* Host stub for espressif/esp_jpeg jpeg_decoder.h (test/host only).
 * Type layout mirrors esp_jpeg 1.3.1 (the version resolved by the IDF Component Manager for this repo).
 * The decoder functions are implemented in test/host/stubs/jpeg_decoder_stub.c and always return
 * ESP_ERR_NOT_SUPPORTED: the JPEG *decode* path (jpg2rgb565, jpg2bmp, fmt2rgb888 for JPEG input)
 * is NOT exercised by this harness. */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JPEG_IMAGE_SCALE_0 = 0,
    JPEG_IMAGE_SCALE_1_2,
    JPEG_IMAGE_SCALE_1_4,
    JPEG_IMAGE_SCALE_1_8,
} esp_jpeg_image_scale_t;

typedef enum {
    JPEG_IMAGE_FORMAT_RGB888 = 0,
    JPEG_IMAGE_FORMAT_RGB565,
} esp_jpeg_image_format_t;

typedef struct esp_jpeg_image_cfg_s {
    uint8_t *indata;
    uint32_t indata_size;
    uint8_t *outbuf;
    uint32_t outbuf_size;
    esp_jpeg_image_format_t out_format;
    esp_jpeg_image_scale_t  out_scale;
    struct {
        uint8_t swap_color_bytes: 1;
    } flags;
    struct {
        void *working_buffer;
        size_t working_buffer_size;
    } advanced;
    struct {
        uint32_t read;
    } priv;
} esp_jpeg_image_cfg_t;

typedef struct esp_jpeg_image_output_s {
    uint16_t width;
    uint16_t height;
    size_t output_len;
} esp_jpeg_image_output_t;

esp_err_t esp_jpeg_decode(esp_jpeg_image_cfg_t *cfg, esp_jpeg_image_output_t *img);
esp_err_t esp_jpeg_get_image_info(esp_jpeg_image_cfg_t *cfg, esp_jpeg_image_output_t *img);

#ifdef __cplusplus
}
#endif
