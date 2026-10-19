/*
 * WifiTest - WiFi connectivity test for MDD32_Bot library
 *
 * Connects to a WiFi network and serves a live status page at the
 * bot's IP address. The page shows battery voltage, charging state,
 * uptime, and signal strength, refreshing automatically via a
 * /status JSON endpoint (no full-page reload).
 *
 * Connection status and the URL to visit are printed over Serial.
 *
 * Setup: edit WIFI_SSID and WIFI_PASS below, then upload.
 */

#include <MDD32_Bot.h>
#include <WiFi.h>
#include <WebServer.h>
#include "wifi_page.h"

MDD32_Bot bot;
WebServer server(80);

const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleStatus() {
  char json[128];
  snprintf(json, sizeof(json),
    "{\"uptime_s\":%lu,\"voltage\":%.2f,\"usb\":%s,\"charging\":%s,\"rssi\":%d}",
    (unsigned long)(millis() / 1000),
    bot.battery.voltage(),
    bot.battery.chargerConnected() ? "true" : "false",
    bot.battery.charging() ? "true" : "false",
    WiFi.RSSI()
  );
  server.send(200, "application/json", json);
}

void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected, open http://");
    Serial.print(WiFi.localIP());
    Serial.println("/ in a browser");
    Serial.print("[WiFi] RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("[WiFi] Connection failed");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
  Serial.println("MDD32 Bot WiFi Test");

  if (!bot.begin()) {
    Serial.println("Failed to initialize MDD32 Bot!");
    while (1);
  }
  Serial.println("MDD32 Bot initialized successfully");

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("[WiFi] HTTP server started on port 80");
}

void loop() {
  server.handleClient();

  // Reconnect if WiFi drops.
  static uint32_t lastCheckMs = 0;
  if (millis() - lastCheckMs >= 5000) {
    lastCheckMs = millis();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost, reconnecting...");
      connectWiFi();
    }
  }
}
