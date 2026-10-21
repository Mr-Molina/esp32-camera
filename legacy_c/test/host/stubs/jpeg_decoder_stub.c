/* Host stub implementation of esp_jpeg entry points (test/host only). */
#include "jpeg_decoder.h"

esp_err_t esp_jpeg_decode(esp_jpeg_image_cfg_t *cfg, esp_jpeg_image_output_t *img)
{
    (void)cfg;
    (void)img;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_jpeg_get_image_info(esp_jpeg_image_cfg_t *cfg, esp_jpeg_image_output_t *img)
{
    (void)cfg;
    (void)img;
    return ESP_ERR_NOT_SUPPORTED;
}
