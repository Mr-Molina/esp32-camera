idf_component_register(SRCS "klipper_cam.c"
                        PRIV_INCLUDE_DIRS "."
                        PRIV_REQUIRES nvs_flash esp_psram esp_wifi esp_event esp_netif esp_http_server)
