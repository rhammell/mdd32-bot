/*
 * BatteryTest - Battery status test for MDD32_Bot library
 *
 * Prints charger connection, charging state, and battery voltage
 * over Serial using the MDD32_Bot battery API.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

// Simple timer for periodic status output.
static uint32_t lastUpdateMs = 0;

void setup() {
  // Bring up serial and core board peripherals.
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
  Serial.println("MDD32 Bot Battery Test");

  // Initialize the bot
  if (!bot.begin()) {
    Serial.println("Failed to initialize MDD32 Bot!");
    while (1);
  }
  Serial.println("MDD32 Bot initialized successfully");
}

void loop() {
  // Print status at a steady interval.
  const uint32_t now = millis();
  if (now - lastUpdateMs >= 500) {
    lastUpdateMs = now;

    Serial.print("USB: ");
    Serial.print(bot.battery.chargerConnected() ? "Connected" : "Not connected");
    Serial.print(" | Charging: ");
    Serial.print(bot.battery.charging() ? "Yes" : "No");
    Serial.print(" | Voltage: ");
    Serial.print(bot.battery.voltage(), 2);
    Serial.println(" V");
  }
}
