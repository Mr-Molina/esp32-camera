#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "esp_camera.h"
#define BOARD_WROVER_KIT 1
#include "camera_pinout.h"
#include "klipper_cam_config.h"

static const char *TAG = "klipper_cam";

static httpd_handle_t s_stream_httpd = NULL;

/* -------------------------------------------------------------------------- */
/*                               Camera Subsystem                             */
/* -------------------------------------------------------------------------- */

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_UXGA,   /* 1600x1200 High-Detail for 3D Printer Inspection */
    .jpeg_quality = CONFIG_KLIPPER_CAM_QUALITY,
    .fb_count = 2,                  /* Double buffering in PSRAM */
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing OV2640 camera on WROVER-DEV bus...");
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, CONFIG_KLIPPER_CAM_VFLIP);
        s->set_hmirror(s, CONFIG_KLIPPER_CAM_HMIRROR);
        ESP_LOGI(TAG, "Camera sensor configured (vflip=%d, hmirror=%d, quality=%d)",
                 CONFIG_KLIPPER_CAM_VFLIP, CONFIG_KLIPPER_CAM_HMIRROR, CONFIG_KLIPPER_CAM_QUALITY);
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                            HTTP Server Handlers                            */
/* -------------------------------------------------------------------------- */

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[128];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "10");

    ESP_LOGI(TAG, "MJPEG stream client connected");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned int)fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        if (res != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(80)); /* ~10-12 FPS throttle for Wi-Fi stability */
    }

    ESP_LOGI(TAG, "MJPEG stream client disconnected");
    return res;
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera snapshot capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (strstr(query, "action=stream")) {
            return stream_handler(req);
        }
        if (strstr(query, "action=snapshot")) {
            return snapshot_handler(req);
        }
    }

    const char *status_page = 
        "Klipper 3D Printer Camera (ESP32-WROVER-DEV)\n"
        "Resolution: 1600x1200 UXGA (OV2640 Hardware JPEG)\n"
        "\n"
        "Endpoints:\n"
        "  - Stream:   /stream   (or /?action=stream)\n"
        "  - Snapshot: /snapshot (or /?action=snapshot)\n"
        "\n"
        "Moonraker Configuration:\n"
        "  [webcam printer_cam]\n"
        "  location: printer\n"
        "  service: mjpegstreamer-adaptive\n"
        "  stream_url: http://<this-ip>/stream\n"
        "  snapshot_url: http://<this-ip>/snapshot\n";

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, status_page, strlen(status_page));
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    httpd_uri_t uri_stream = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t uri_snapshot = {
        .uri       = "/snapshot",
        .method    = HTTP_GET,
        .handler   = snapshot_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t uri_root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting HTTP server on port %d...", config.server_port);
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_stream);
        httpd_register_uri_handler(server, &uri_snapshot);
        httpd_register_uri_handler(server, &uri_root);
        ESP_LOGI(TAG, "HTTP Server started successfully. Endpoints: /stream, /snapshot");
        return server;
    }

    ESP_LOGE(TAG, "Error starting HTTP server!");
    return NULL;
}

/* -------------------------------------------------------------------------- */
/*                               Wi-Fi Station                                */
/* -------------------------------------------------------------------------- */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi station started, connecting to SSID '%s'...", CONFIG_KLIPPER_CAM_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected. Reconnecting in 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "==========================================================");
        ESP_LOGI(TAG, "Wi-Fi Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Stream URL:   http://" IPSTR "/stream", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Snapshot URL: http://" IPSTR "/snapshot", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "==========================================================");

        if (s_stream_httpd == NULL) {
            s_stream_httpd = start_webserver();
        }
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

#if defined(CONFIG_KLIPPER_CAM_USE_DHCP) && (CONFIG_KLIPPER_CAM_USE_DHCP == 1)
    ESP_LOGI(TAG, "Dynamic DHCP client enabled on Wi-Fi station interface");
#elif defined(CONFIG_KLIPPER_CAM_STATIC_IP)
    ESP_LOGI(TAG, "Configuring Static IP: %s (GW: %s, MASK: %s)",
             CONFIG_KLIPPER_CAM_STATIC_IP, CONFIG_KLIPPER_CAM_STATIC_GW, CONFIG_KLIPPER_CAM_STATIC_NETMASK);
    esp_netif_dhcpc_stop(sta_netif);

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_IP);
    ip_info.gw.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_GW);
    ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_NETMASK);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_DNS);
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info);
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_KLIPPER_CAM_WIFI_SSID,
            .password = CONFIG_KLIPPER_CAM_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* -------------------------------------------------------------------------- */
/*                                Main Entry                                  */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize camera hardware first */
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Fatal: Camera hardware initialization failed");
        return;
    }

    /* Initialize Wi-Fi station stack */
    wifi_init_sta();

    /* Monitor loop */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGD(TAG, "Free heap: %u, Free PSRAM: %u",
                 (unsigned int)esp_get_free_heap_size(),
                 (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}
