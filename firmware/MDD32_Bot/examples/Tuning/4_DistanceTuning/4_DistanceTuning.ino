/*
 * 4_DistanceTuning - Acceleration and distance tuning for MDD32_Bot library
 *
 * Prerequisite: PID and wheel-sync tuning must be completed
 * first (see 2_PIDTuning, 3_WheelSyncTuning).
 *
 * Adjust TUNE_MAX_ACCEL_RPM_S below (with the bot on its tuning stand or a
 * clear floor runway) to tune the acceleration ramp rate without editing
 * robot_definitions.h on every test run. Once you find a value you like, copy
 * it into src/definitions/robot_definitions.h (MAX_ACCEL_RPM_S) to make it the
 * library-wide default, or call bot.drive.setMaxAccel() in your own sketch's
 * setup().
 *
 * Key parameters:
 *   TUNE_MAX_ACCEL_RPM_S - Ramp rate (RPM/s) for acceleration. Affects how
 *                          smoothly and quickly the bot reaches cruising speed
 *                          (0 disables ramping).
 *   DECEL_DISTANCE_CM    - How far before the target to begin decelerating.
 *   MIN_SPEED_CM_S       - Minimum speed during deceleration.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

// --- Ramp Tuning Parameters (edit these for each test run) ---
const float    TUNE_MAX_ACCEL_RPM_S = MAX_ACCEL_RPM_S;  // RPM/s (0 = no ramp)
const float    DRIVE_SPEED_CM_S     = 5.0f;
const uint32_t DRIVE_DURATION_MS    = 10000;

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Distance Tuning");

    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }

    Serial.println("MDD32 Bot initialized successfully");

    // Apply the ramp rate configured at the top of this sketch
    bot.drive.setMaxAccel(TUNE_MAX_ACCEL_RPM_S);
    Serial.print("Max Accel (RPM/s): ");
    Serial.println(TUNE_MAX_ACCEL_RPM_S, 1);

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
