#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#if !defined(BOARD_WROVER_KIT) && !defined(BOARD_ESP32CAM_AITHINKER)
#define BOARD_WROVER_KIT 1
#endif

#include "esp_camera.h"
#include "camera_pinout.h"
#include "klipper_cam_config.h"

static const char *TAG = "klipper_cam";

static httpd_handle_t s_camera_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static SemaphoreHandle_t s_fb_mutex = NULL;
static volatile bool s_stream_active = false;
static esp_timer_handle_t s_wifi_reconnect_timer = NULL;

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
    .frame_size = FRAMESIZE_HD,     /* 1280x720 Native 16:9 Widescreen for Klipper */
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

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[128];

    /* Port 81 Busy Guard (NET-002): atomic single-client lock */
    if (!__sync_bool_compare_and_swap(&s_stream_active, false, true)) {
        ESP_LOGW(TAG, "Stream rejected: another client is already streaming");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream already active");
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        s_stream_active = false;
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "5");

    ESP_LOGI(TAG, "MJPEG stream client connected");

    int fail_count = 0;
    while (true) {
        if (xSemaphoreTake(s_fb_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGW(TAG, "Stream fb mutex timeout, retrying...");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        fb = esp_camera_fb_get();
        if (!fb) {
            xSemaphoreGive(s_fb_mutex);
            fail_count++;
            ESP_LOGW(TAG, "Camera capture retry %d/5", fail_count);
            if (fail_count >= 5) {
                ESP_LOGE(TAG, "Camera capture failed 5 times, aborting stream");
                res = ESP_FAIL;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        fail_count = 0;

        size_t hlen = snprintf(part_buf, sizeof(part_buf),
                               "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                               (unsigned int)fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);
        fb = NULL;
        xSemaphoreGive(s_fb_mutex);

        if (res != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); /* 50ms socket pacing for LwIP TCP buffer drain */
    }

    s_stream_active = false;
    ESP_LOGI(TAG, "MJPEG stream client disconnected");
    return res;
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_fb_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Camera snapshot mutex timeout");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera busy");
        return ESP_FAIL;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        xSemaphoreGive(s_fb_mutex);
        ESP_LOGE(TAG, "Camera snapshot capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    xSemaphoreGive(s_fb_mutex);
    return res;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char json_buf[384];
    wifi_ap_record_t ap_info;
    int rssi = 0;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = ap_info.rssi;
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    const char *part_label = running ? running->label : "unknown";
    uint32_t free_heap = (uint32_t)esp_get_free_heap_size();
    uint32_t free_psram = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    int64_t uptime_ms = esp_timer_get_time() / 1000;

    int len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"uptime_ms\":%lld,"
        "\"free_heap\":%u,"
        "\"free_psram\":%u,"
        "\"wifi_rssi\":%d,"
        "\"active_partition\":\"%s\","
        "\"framesize\":\"1280x720\","
        "\"stream_active\":%s"
        "}\n",
        (long long)uptime_ms,
        (unsigned int)free_heap,
        (unsigned int)free_psram,
        rssi,
        part_label,
        s_stream_active ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_buf, len);
}

static esp_err_t root_handler(httpd_req_t *req)
{
    /* Dynamic query buffer (APP-013) */
    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen > 0 && qlen <= 256) {
        char *query = malloc(qlen + 1);
        if (query) {
            if (httpd_req_get_url_query_str(req, query, qlen + 1) == ESP_OK) {
                if (strstr(query, "action=snapshot")) {
                    free(query);
                    return snapshot_handler(req);
                }
                if (strstr(query, "action=stream")) {
                    free(query);
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Stream available only on port 81 (/stream)");
                }
            }
            free(query);
        }
    }

    const char *status_page = 
        "Klipper 3D Printer Camera (ESP32-WROVER-DEV)\n"
        "Resolution: 1280x720 HD 16:9 (OV2640 Hardware JPEG)\n"
        "Framerate:  4-5 FPS (OV2640 Hardware HD Limit with LwIP Pacing)\n"
        "\n"
        "Endpoints:\n"
        "  - Dedicated Stream: http://<this-ip>:81/stream (Isolated Worker Task)\n"
        "  - Snapshot:         http://<this-ip>/snapshot  (Instantaneous, Zero-Block)\n"
        "  - Status JSON:      http://<this-ip>/status\n"
        "  - OTA Update:       http://<this-ip>/update\n"
        "\n"
        "Moonraker Configuration:\n"
        "  [webcam printer_cam]\n"
        "  location: printer\n"
        "  service: mjpegstreamer-adaptive\n"
        "  stream_url: http://<this-ip>:81/stream\n"
        "  snapshot_url: http://<this-ip>/snapshot\n";

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, status_page, strlen(status_page));
}

/* -------------------------------------------------------------------------- */
/*                               OTA Update Handlers                          */
/* -------------------------------------------------------------------------- */

static esp_err_t ota_update_get_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    httpd_resp_set_type(req, "text/html");

    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><title>Klipper Cam OTA Update</title>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<style>body{font-family:sans-serif;margin:40px;background:#1e1e1e;color:#eee}"
        ".card{background:#2a2a2a;padding:24px;border-radius:8px;max-width:500px;margin:auto}"
        "input,button{margin:12px 0;padding:10px;border-radius:4px;border:none;font-size:16px;box-sizing:border-box;width:100%}"
        "button{background:#007acc;color:#fff;cursor:pointer;width:100%}"
        "#prg{width:100%;background:#444;border-radius:4px;height:20px;display:none;margin-top:12px}"
        "#bar{width:0%;height:100%;background:#28a745;border-radius:4px;text-align:center;font-size:12px;line-height:20px}"
        "</style></head><body><div class=\"card\">"
        "<h2>Klipper Cam OTA Update</h2>");

    char info_buf[256];
    snprintf(info_buf, sizeof(info_buf),
        "<p>Running Partition: <b>%s</b></p>"
        "<p>Next Target: <b>%s</b></p>",
        running ? running->label : "unknown",
        update_partition ? update_partition->label : "none");
    httpd_resp_sendstr_chunk(req, info_buf);

    httpd_resp_sendstr_chunk(req,
        "<input type=\"password\" id=\"tok\" placeholder=\"OTA Security Token\"><br>"
        "<input type=\"file\" id=\"f\" accept=\".bin\"><br>"
        "<button id=\"btn\" onclick=\"upload()\">Flash Firmware</button>"
        "<div id=\"prg\"><div id=\"bar\">0%</div></div>"
        "<p id=\"st\"></p>"
        "</div><script>"
        "function upload(){"
        "  var tok=document.getElementById('tok').value;"
        "  var file=document.getElementById('f').files[0];"
        "  if(!tok){alert('Enter OTA Token');return;}"
        "  if(!file){alert('Select .bin file first');return;}"
        "  document.getElementById('btn').disabled=true;"
        "  document.getElementById('prg').style.display='block';"
        "  document.getElementById('st').innerText='Uploading and writing flash...';"
        "  var xhr=new XMLHttpRequest();"
        "  xhr.open('POST','/update',true);"
        "  xhr.setRequestHeader('X-OTA-Token',tok);"
        "  xhr.upload.onprogress=function(e){if(e.lengthComputable){"
        "    var p=Math.round((e.loaded/e.total)*100);"
        "    document.getElementById('bar').style.width=p+'%';"
        "    document.getElementById('bar').innerText=p+'%';"
        "  }};"
        "  xhr.onload=function(){"
        "    if(xhr.status==200){"
        "      document.getElementById('st').innerText='Flash successful! Rebooting in 5s...';"
        "      setTimeout(function(){location.href='/';},5000);"
        "    }else{"
        "      document.getElementById('st').innerText='Error: '+xhr.responseText;"
        "      document.getElementById('btn').disabled=false;"
        "    }"
        "  };"
        "  xhr.onerror=function(){"
        "    document.getElementById('st').innerText='Network error during upload';"
        "    document.getElementById('btn').disabled=false;"
        "  };"
        "  xhr.send(file);"
        "}"
        "</script></body></html>");

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t ota_update_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        ESP_LOGE(TAG, "OTA failed: Next update partition not found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No passive OTA partition found");
        return ESP_FAIL;
    }

    /* OTA Authentication Enforcement (APP-002, FW-010, CWE-306) */
    char token_hdr[64] = {0};
    esp_err_t tok_res = httpd_req_get_hdr_value_str(req, "X-OTA-Token", token_hdr, sizeof(token_hdr));
    if (tok_res != ESP_OK || strcmp(token_hdr, WIFI_OTA_TOKEN) != 0) {
        ESP_LOGW(TAG, "OTA authorization failed: invalid or missing X-OTA-Token header");
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "Unauthorized: Invalid or missing X-OTA-Token\n", HTTPD_RESP_USE_STRLEN);
    }

    int total_len = req->content_len;
    if (total_len <= 0) {
        ESP_LOGE(TAG, "OTA failed: Missing or invalid Content-Length: %d", total_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length header required");
        return ESP_FAIL;
    }

    /* Target Partition Size Upper Bound Check (APP-008) */
    if ((size_t)total_len > update_partition->size) {
        ESP_LOGE(TAG, "OTA failed: Image size %d exceeds partition size %u",
                 total_len, (unsigned int)update_partition->size);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "OTA image size exceeds target partition size");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update: size=%d bytes, writing to partition '%s' (offset 0x%08lx)",
             total_len, update_partition->label, (unsigned long)update_partition->address);

    /* Pass known image size to esp_ota_begin to bound erase cycles (APP-007) */
    esp_err_t err = esp_ota_begin(update_partition, (size_t)total_len, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char *ota_buf = malloc(1024);
    if (!ota_buf) {
        esp_ota_abort(update_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    int remaining = total_len;
    int received = 0;
    int timeout_retries = 0;

    while (remaining > 0) {
        int to_read = remaining > 1024 ? 1024 : remaining;
        int ret = httpd_req_recv(req, ota_buf, to_read);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                timeout_retries++;
                if (timeout_retries <= 5) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue; /* APP-007: bound retry loop */
                }
                ESP_LOGE(TAG, "OTA socket read timed out (%d retries)", timeout_retries);
            }
            ESP_LOGE(TAG, "OTA socket read error: %d", ret);
            free(ota_buf);
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA socket read error or timeout");
            return ESP_FAIL;
        }
        timeout_retries = 0;

        err = esp_ota_write(update_handle, (const void *)ota_buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %d bytes: %s", received, esp_err_to_name(err));
            free(ota_buf);
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
            return ESP_FAIL;
        }

        remaining -= ret;
        received += ret;
    }

    free(ota_buf);

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (signature/image validation error): %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Firmware image validation failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to update boot partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA firmware update successful! Written %d bytes to '%s'. Scheduling restart...",
             received, update_partition->label);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Firmware update successful! Rebooting...\n");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static void start_webservers(void)
{
    /* 1. Main HTTP Server on Port 80: Handles snapshots, status, OTA updates, and control */
    httpd_config_t config_p80 = HTTPD_DEFAULT_CONFIG();
    config_p80.server_port = 80;
    config_p80.ctrl_port = 32768;
    config_p80.max_open_sockets = 4;
    config_p80.lru_purge_enable = true;

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

    httpd_uri_t uri_status = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t uri_ota_get = {
        .uri       = "/update",
        .method    = HTTP_GET,
        .handler   = ota_update_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t uri_ota_post = {
        .uri       = "/update",
        .method    = HTTP_POST,
        .handler   = ota_update_post_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting main HTTP server on port %d...", config_p80.server_port);
    if (httpd_start(&s_camera_httpd, &config_p80) == ESP_OK) {
        httpd_register_uri_handler(s_camera_httpd, &uri_snapshot);
        httpd_register_uri_handler(s_camera_httpd, &uri_root);
        httpd_register_uri_handler(s_camera_httpd, &uri_status);
        httpd_register_uri_handler(s_camera_httpd, &uri_ota_get);
        httpd_register_uri_handler(s_camera_httpd, &uri_ota_post);
        ESP_LOGI(TAG, "Main HTTP server active on port 80 (/snapshot, /, /status, /update)");
    } else {
        ESP_LOGE(TAG, "Error starting main HTTP server on port 80!");
    }

    /* 2. Dedicated Video Stream Server on Port 81: Isolated FreeRTOS task */
    httpd_config_t config_p81 = HTTPD_DEFAULT_CONFIG();
    config_p81.server_port = 81;
    config_p81.ctrl_port = 32769;
    config_p81.max_open_sockets = 2;
    config_p81.send_wait_timeout = 2;
    config_p81.recv_wait_timeout = 2;
    config_p81.lru_purge_enable = true;

    httpd_uri_t uri_stream_p81 = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t uri_root_p81 = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting dedicated MJPEG stream server on port %d...", config_p81.server_port);
    if (httpd_start(&s_stream_httpd, &config_p81) == ESP_OK) {
        httpd_register_uri_handler(s_stream_httpd, &uri_stream_p81);
        httpd_register_uri_handler(s_stream_httpd, &uri_root_p81);
        ESP_LOGI(TAG, "Dedicated MJPEG stream server active on port 81 (/stream, /)");
    } else {
        ESP_LOGE(TAG, "Error starting dedicated stream server on port 81!");
    }
}

/* -------------------------------------------------------------------------- */
/*                               Wi-Fi Station                                */
/* -------------------------------------------------------------------------- */

static void wifi_reconnect_timer_cb(void* arg)
{
    ESP_LOGI(TAG, "Reconnecting Wi-Fi...");
    esp_wifi_connect();
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi station started, connecting to SSID '%s'...", CONFIG_KLIPPER_CAM_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected. Scheduling reconnect in 2 seconds...");
        if (s_wifi_reconnect_timer) {
            esp_timer_stop(s_wifi_reconnect_timer);
            esp_timer_start_once(s_wifi_reconnect_timer, 2000000); /* 2s non-blocking (APP-009) */
        } else {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "==========================================================");
        ESP_LOGI(TAG, "Wi-Fi Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Dedicated Stream URL: http://" IPSTR ":81/stream", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Control/Snapshot URL: http://" IPSTR "/snapshot", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Status Telemetry URL: http://" IPSTR "/status", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "==========================================================");

        /* Cancel rollback on successful network acquisition (APP-003, FW-002) */
        esp_ota_mark_app_valid_cancel_rollback();

        if (s_camera_httpd == NULL && s_stream_httpd == NULL) {
            start_webservers();
        }
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_timer_create_args_t timer_args = {
        .callback = &wifi_reconnect_timer_cb,
        .name = "wifi_reconnect"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_wifi_reconnect_timer));

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

#if defined(CONFIG_KLIPPER_CAM_USE_DHCP) && (CONFIG_KLIPPER_CAM_USE_DHCP == 1)
    ESP_LOGI(TAG, "Dynamic DHCP client enabled on Wi-Fi station interface");
#elif defined(CONFIG_KLIPPER_CAM_STATIC_IP)
    ESP_LOGI(TAG, "Configuring Static IP: %s (GW: %s, MASK: %s)",
             CONFIG_KLIPPER_CAM_STATIC_IP, CONFIG_KLIPPER_CAM_STATIC_GW, CONFIG_KLIPPER_CAM_STATIC_NETMASK);
    esp_err_t err_dhcp = esp_netif_dhcpc_stop(sta_netif);
    if (err_dhcp != ESP_OK && err_dhcp != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "Failed to stop DHCP client: %s", esp_err_to_name(err_dhcp));
    }

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_IP);
    ip_info.gw.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_GW);
    ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_NETMASK);
    if (ip_info.ip.addr == 0) {
        ESP_LOGE(TAG, "Invalid static IP configuration: %s", CONFIG_KLIPPER_CAM_STATIC_IP);
    }
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = esp_ip4addr_aton(CONFIG_KLIPPER_CAM_STATIC_DNS);
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    ESP_ERROR_CHECK(esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi modem sleep disabled (WIFI_PS_NONE active for line-rate video throughput)");
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

    /* Initialize frame buffer synchronization mutex (APP-006) */
    s_fb_mutex = xSemaphoreCreateMutex();
    assert(s_fb_mutex != NULL);

#if defined(BOARD_ESP32CAM_AITHINKER)
    /* Ensure AI-Thinker flash LED (GPIO 4) is held OFF only on AiThinker board (APP-004, FW-001) */
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);
#endif

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
