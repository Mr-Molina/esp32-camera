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
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "klipper_cam_config.h"

static const char *TAG = "ota_minimal";
static httpd_handle_t s_httpd = NULL;

static esp_err_t root_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    char resp[512];
    snprintf(resp, sizeof(resp),
        "Klipper Cam OTA Minimal Intermediary\n"
        "State: Ready for main application upload\n"
        "Running Partition: %s (0x%08lx)\n"
        "Next Target:      %s (0x%08lx)\n"
        "Free Heap:        %u bytes\n"
        "\n"
        "Upload Main Firmware:\n"
        "  CLI: curl -X POST --data-binary @firmware.bin http://<this-ip>/update\n"
        "  Web: http://<this-ip>/update\n",
        running ? running->label : "unknown",
        running ? (unsigned long)running->address : 0,
        update_partition ? update_partition->label : "none",
        update_partition ? (unsigned long)update_partition->address : 0,
        (unsigned int)esp_get_free_heap_size()
    );

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t ota_update_get_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><title>Klipper Minimal OTA Intermediary</title>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<style>body{font-family:sans-serif;margin:40px;background:#1e1e1e;color:#eee}"
        ".card{background:#2a2a2a;padding:24px;border-radius:8px;max-width:500px;margin:auto}"
        "input,button{margin:12px 0;padding:10px;border-radius:4px;border:none;font-size:16px}"
        "button{background:#007acc;color:#fff;cursor:pointer;width:100%}"
        "#prg{width:100%;background:#444;border-radius:4px;height:20px;display:none;margin-top:12px}"
        "#bar{width:0%;height:100%;background:#28a745;border-radius:4px;text-align:center;font-size:12px;line-height:20px}"
        "</style></head><body><div class=\"card\">"
        "<h2>Klipper Cam OTA Minimal Intermediary</h2>"
        "<p>Camera is powered down. System is in high-stability OTA mode.</p>");

    char info_buf[256];
    snprintf(info_buf, sizeof(info_buf),
        "<p>Running Partition: <b>%s</b></p>"
        "<p>Next Target: <b>%s</b></p>",
        running ? running->label : "unknown",
        update_partition ? update_partition->label : "none");
    httpd_resp_sendstr_chunk(req, info_buf);

    httpd_resp_sendstr_chunk(req,
        "<input type=\"file\" id=\"f\" accept=\".bin\"><br>"
        "<button id=\"btn\" onclick=\"upload()\">Flash Main Klipper Firmware</button>"
        "<div id=\"prg\"><div id=\"bar\">0%</div></div>"
        "<p id=\"st\"></p>"
        "</div><script>"
        "function upload(){"
        "  var file=document.getElementById('f').files[0];"
        "  if(!file){alert('Select .bin file first');return;}"
        "  document.getElementById('btn').disabled=true;"
        "  document.getElementById('prg').style.display='block';"
        "  document.getElementById('st').innerText='Uploading and writing flash...';"
        "  var xhr=new XMLHttpRequest();"
        "  xhr.open('POST','/update',true);"
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

    int total_len = req->content_len;
    if (total_len <= 0) {
        ESP_LOGE(TAG, "OTA failed: Missing Content-Length header");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Length header required");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update: size=%d bytes, writing to '%s' (offset 0x%08lx)",
             total_len, update_partition->label, (unsigned long)update_partition->address);

    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char *ota_buf = malloc(2048);
    if (!ota_buf) {
        esp_ota_abort(update_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    int remaining = total_len;
    int received = 0;

    while (remaining > 0) {
        int to_read = remaining > 2048 ? 2048 : remaining;
        int ret = httpd_req_recv(req, ota_buf, to_read);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Socket receive error: %d", ret);
            free(ota_buf);
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }

        err = esp_ota_write(update_handle, (const void *)ota_buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %d: %s", received, esp_err_to_name(err));
            free(ota_buf);
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write error");
            return ESP_FAIL;
        }

        remaining -= ret;
        received += ret;
    }

    free(ota_buf);

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end validation failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image validation failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA successful! Written %d bytes. Rebooting...", received);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "Firmware update successful! Rebooting...\n");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    httpd_uri_t uri_root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_handler,
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

    if (httpd_start(&s_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(s_httpd, &uri_root);
        httpd_register_uri_handler(s_httpd, &uri_ota_get);
        httpd_register_uri_handler(s_httpd, &uri_ota_post);
        ESP_LOGI(TAG, "OTA Minimal HTTP server active on port 80");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi station started, connecting to '%s'...", CONFIG_KLIPPER_CAM_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected. Reconnecting...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        if (s_httpd == NULL) {
            start_webserver();
        }
    }
}

void app_main(void)
{
    esp_ota_mark_app_valid_cancel_rollback();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

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

    ESP_LOGI(TAG, "OTA Minimal Intermediary running. Camera powered down. Awaiting firmware upload.");
}
