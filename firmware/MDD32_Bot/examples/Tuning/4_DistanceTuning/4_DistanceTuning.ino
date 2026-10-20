/*
 * 4_DistanceTuning - Distance and deceleration tuning for MDD32_Bot library
 *
 * Prerequisite: PID and wheel-sync tuning must be completed
 * first (see 2_PIDTuning, 3_WheelSyncTuning).
 *
 * Use this sketch to tune the deceleration profile for distance-
 * bounded moves. The relevant parameters are defined in
 * robot_definitions.h. Each change requires recompiling and
 * uploading.
 *
 * The sketch commands a drive to a target distance, then monitors
 * how accurately the bot stops at the mark. After stopping, the
 * final distance is printed so overshoot can be measured.
 *
 * Key parameters to tune:
 *   DECEL_DISTANCE_CM  - How far before the target to begin
 *                        decelerating. Too short causes overshoot;
 *                        too long causes an unnecessarily slow
 *                        approach.
 *   MIN_SPEED_CM_S     - Minimum speed during deceleration. Too
 *                        low and the bot stalls before reaching
 *                        the target; too high and it arrives with
 *                        too much momentum.
 *   MAX_ACCEL_RPM_S    - Ramp rate for acceleration and
 *                        deceleration. Affects how quickly the bot
 *                        reaches cruising speed and how aggressively
 *                        it slows down.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

const float DRIVE_SPEED_CM_S = 5.0f;
const uint32_t DRIVE_DURATION_MS = 10000;

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Distance Tuning");

    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }

    Serial.println("MDD32 Bot initialized successfully");

    // Start driving forward
    bot.drive.straight(DRIVE_SPEED_CM_S);
}

void loop() {
    // Stop after set duration
    static bool stopped = false;
    if (!stopped && millis() >= DRIVE_DURATION_MS) {
        bot.drive.stop();
        stopped = true;
        Serial.println("\n--- Stop commanded ---");
    }

    // Update telemetry at 10 Hz
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();

        DriveTelemetry s = bot.drive.getTelemetry();

        int64_t avgTicks = (abs(s.leftCount) + abs(s.rightCount)) / 2;
        float distanceTraveled = (float)avgTicks / TICKS_PER_CM;
        float avgRPM = (s.leftRPM + s.rightRPM) / 2.0f;
        float speed_cm_s = (avgRPM * WHEEL_CIRCUMFERENCE_CM) / 60.0f;

        // Serial telemetry
        Serial.print("Dist: ");
        Serial.print(distanceTraveled, 2);
        Serial.print(" cm | Spd: ");
        Serial.print(speed_cm_s, 2);
        Serial.print(" cm/s | L Enc: ");
        Serial.print((long)s.leftCount);
        Serial.print(" R Enc: ");
        Serial.print((long)s.rightCount);
        Serial.print(" Diff: ");
        Serial.print((long)(s.leftCount - s.rightCount));
        Serial.print(" | L RPM: ");
        Serial.print(s.leftRPM, 1);
        Serial.print(" R RPM: ");
        Serial.println(s.rightRPM, 1);

        if (stopped) {
            Serial.print("Final distance: ");
            Serial.print(distanceTraveled, 2);
            Serial.println(" cm");
        }
    }

    // Run the drive control loop
    bot.drive.update();
}
