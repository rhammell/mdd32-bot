/*
 * ForwardTest - Basic drive/encoder/odometry sanity check for MDD32_Bot
 *
 * Drives the bot straight forward at a modest speed for a few seconds,
 * then stops. Throughout, it prints the left/right encoder counts and the
 * accumulated odometry distance over Serial.
 *
 * Use this to confirm the fundamentals after a build or wiring change:
 *   - The bot drives forward (not backward, not spinning).
 *   - Both encoder counts climb POSITIVELY and stay close to each other.
 *   - getDistance() grows and roughly matches the physical distance driven.
 *
 * SAFETY: the bot drives itself forward - give it a clear runway, or prop
 * the wheels up so it runs in place.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

const float    DRIVE_SPEED_CM_S = 6.0f;   // modest forward speed
const uint32_t DRIVE_MS         = 3000;   // how long to drive before stopping

bool stopped = false;
uint32_t startMs = 0;

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Forward Test");

    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");

    // Start driving straight forward
    bot.drive.straight(DRIVE_SPEED_CM_S);
    startMs = millis();
    Serial.print("Driving forward at ");
    Serial.print(DRIVE_SPEED_CM_S, 1);
    Serial.println(" cm/s");
}

void loop() {
    // Run the drive control loop (no-op once stopped)
    bot.drive.update();

    uint32_t now = millis();

    // Stop after the drive window elapses
    if (!stopped && now - startMs >= DRIVE_MS) {
        bot.drive.stop();
        stopped = true;
        Serial.println("--- Stop commanded ---");
    }

    // Print encoder counts and distance at 10 Hz
    static uint32_t lastPrintMs = 0;
    if (now - lastPrintMs >= 100) {
        lastPrintMs = now;

        DriveTelemetry s = bot.drive.getTelemetry();

        Serial.print("T: ");
        Serial.print((now - startMs) / 1000.0, 1);
        Serial.print("s  L Enc: ");
        Serial.print((long)s.leftCount);
        Serial.print("  R Enc: ");
        Serial.print((long)s.rightCount);
        Serial.print("  Dist: ");
        Serial.print(bot.drive.getDistance(), 2);
        Serial.println(" cm");
    }
}
