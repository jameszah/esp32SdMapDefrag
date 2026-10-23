// esp32SdMapDefrag.ino - SD card map, file pieces, defrag, delete and format, on a web page.
// https://github.com/jameszah/esp32SdMapDefrag
// The work is in SdTools.cpp; this connects to WiFi and starts it.
//
// Board: AI Thinker ESP32-CAM, or any ESP32 with an SD card on the SD_MMC pins (1-bit mode).
// Fill in WIFI_SSID and WIFI_PASS, open the serial monitor at 115200 and browse to the address
// it prints.

#include <WiFi.h>
#include <esp_heap_caps.h>
#include "SdTools.h"

const char *WIFI_SSID = "";  // your network
const char *WIFI_PASS = "";

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  logf("esp32SdMapDefrag %s", SDMAP_VERSION);
  logf("PSRAM %u KB free", (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

  sdBegin();  // no card yet is fine: the page mounts it when one is put in

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to \"%s\"", WIFI_SSID);
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi did not connect - check WIFI_SSID and WIFI_PASS. Restarting in 10 s");
    delay(10000);
    ESP.restart();
  }
  Serial.println();
  logf("Open http://%s/sdmap - Click Read Card to see map and files", WiFi.localIP().toString().c_str());
  sdToolsStartServer();
}

void loop() {
  static unsigned long last = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - last > 10000) {  // dropped: try again every 10 s
    last = millis();
    logf("WiFi lost, reconnecting");
    WiFi.reconnect();
  }
  sdToolsLoop();
  delay(100);
}
