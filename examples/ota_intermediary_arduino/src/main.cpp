#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

static const char* WIFI_SSID = "iot";
static const char* WIFI_PASS = "@LiRHkrnFT*iWk#j#*^7mKW_gfjA.C";

WebServer server(80);

static const char HTML_UPDATE[] PROGMEM = 
"<!DOCTYPE html><html><head><title>Klipper Cam Arduino Intermediary</title>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<style>body{font-family:sans-serif;margin:40px;background:#1e1e1e;color:#eee}"
".card{background:#2a2a2a;padding:24px;border-radius:8px;max-width:500px;margin:auto}"
"input,button{margin:12px 0;padding:10px;border-radius:4px;border:none;font-size:16px}"
"button{background:#007acc;color:#fff;cursor:pointer;width:100%}"
"#prg{width:100%;background:#444;border-radius:4px;height:20px;display:none;margin-top:12px}"
"#bar{width:0%;height:100%;background:#28a745;border-radius:4px;text-align:center;font-size:12px;line-height:20px}"
"</style></head><body><div class=\"card\">"
"<h2>Klipper Cam Arduino Intermediary</h2>"
"<p>Intermediary running under Arduino-ESP32 (IDF v4.4 ABI).</p>"
"<p>Hardware Flash LED is disabled. System ready for final firmware upload.</p>"
"<input type=\"file\" id=\"f\" accept=\".bin\"><br>"
"<button id=\"btn\" onclick=\"upload()\">Flash Firmware</button>"
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
"    document.getElementById('st').innerText=xhr.responseText;"
"    if(xhr.status==200){"
"      setTimeout(function(){location.reload();},15000);"
"    }"
"  };"
"  var fd=new FormData();"
"  fd.append('firmware',file);"
"  xhr.send(fd);"
"}"
"</script></body></html>";

void handleRoot() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next_ota = esp_ota_get_next_update_partition(NULL);

    String msg = "Klipper Cam Arduino OTA Intermediary\n";
    msg += "State: Active and Ready\n";
    msg += "IP Address: " + WiFi.localIP().toString() + "\n";
    msg += "MAC Address: " + WiFi.macAddress() + "\n";
    msg += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    msg += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    msg += "Running Partition: " + String(running ? running->label : "unknown") + "\n";
    msg += "Next Partition: " + String(next_ota ? next_ota->label : "none") + "\n\n";
    msg += "Update endpoints:\n";
    msg += "  Web UI: http://" + WiFi.localIP().toString() + "/update\n";
    msg += "  CLI:    curl -F \"firmware=@image.bin\" http://" + WiFi.localIP().toString() + "/update\n";
    msg += "  ArduinoOTA: Port 3232 (espota)\n";
    server.send(200, "text/plain", msg);
}

void handleUpdateGet() {
    server.send_P(200, "text/html", HTML_UPDATE);
}

void handleUpdatePost() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        server.send(500, "text/plain", "Update FAILED: " + String(Update.getError()));
    } else {
        server.send(200, "text/plain", "Update SUCCESSFUL. Rebooting MCU...");
    }
    delay(1000);
    ESP.restart();
}

void handleFileUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update Start: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("Update Success: %u bytes\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

void setup() {
    // Force flash LED OFF and camera PWDN HIGH immediately
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);

    // Cancel automatic bootloader rollback
    esp_ota_mark_app_valid_cancel_rollback();

    Serial.begin(115200);
    Serial.println("\n--- Klipper Cam Arduino OTA Intermediary Starting ---");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting to Wi-Fi SSID: ");
    Serial.println(WIFI_SSID);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Wi-Fi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to Wi-Fi, entering fallback...");
    }

    // Configure WebServer
    server.on("/", HTTP_GET, handleRoot);
    server.on("/update", HTTP_GET, handleUpdateGet);
    server.on("/update", HTTP_POST, handleUpdatePost, handleFileUpload);
    server.begin();
    Serial.println("HTTP Server started on port 80");

    // Configure ArduinoOTA
    ArduinoOTA.setHostname("Ender3V2-CAM-OTA");
    ArduinoOTA.onStart([]() {
        Serial.println("ArduinoOTA Start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nArduinoOTA End");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("ArduinoOTA Error[%u]\n", error);
    });
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA service active on port 3232");
}

void loop() {
    server.handleClient();
    ArduinoOTA.handle();
    delay(2);
}
