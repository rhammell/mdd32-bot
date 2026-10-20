/*
 * 3_WheelSyncTuning - Wheel-sync correction tuning for MDD32_Bot library
 *
 * Prerequisite: PID tuning must be completed first (see 2_PIDTuning).
 *
 * Use this sketch to tune the single wheel-sync correction gain
 * (TUNE_WHEEL_SYNC_KP below) with the bot on its tuning stand and USB
 * connected (or on a clear floor runway if verifying real-world tracking).
 * The same gain balances both a straight (wheels together) and a spin (wheels
 * opposite), so this sketch can exercise either one. Set SPIN_MODE below to
 * pick which, and adjust TUNE_WHEEL_SYNC_KP for each test run. setup()
 * automatically disables ramping (setMaxAccel(0)) so the wheels run at a
 * constant target RPM.
 *
 * Once you find a value you are happy with, either copy it into
 * src/definitions/robot_definitions.h (WHEEL_SYNC_KP) to make it the
 * library-wide default, or call bot.drive.setWheelSyncKP() in your own
 * sketch's setup().
 *
 * Both modes monitor the balance "diff" between the wheels: for a
 * straight it is (L - R) and should stay near zero as the counts
 * track together; for a spin it is (L + R) and should stay near zero
 * as the counts grow equal-and-opposite. The diff is printed over
 * Serial at 10 Hz.
 *
 * If the diff drifts steadily in one direction, TUNE_WHEEL_SYNC_KP is too
 * low and the correction is not strong enough to keep the wheels
 * matched. If the diff oscillates rapidly around zero, TUNE_WHEEL_SYNC_KP
 * is too high and the correction is overcorrecting. A good result
 * is a diff that stays near zero with no sustained drift or
 * oscillation.
 */

#include <MDD32_Bot.h>

// --- Wheel-Sync Tuning Parameters (edit these for each test run) ---
constexpr float TUNE_WHEEL_SYNC_KP = WHEEL_SYNC_KP;  // try e.g. 0.2, 0.5, 1.0
constexpr bool  SPIN_MODE          = false;          // false = straight, true = spin
constexpr float DRIVE_SPEED_CM_S   = 6.0f;

// MDD32 Bot instance
MDD32_Bot bot;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Wheel Sync Tuning");

    // Initialize the bot
    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");

    // Disable ramping for constant-RPM testing and apply the wheel-sync gain
    bot.drive.setMaxAccel(0.0f);
    bot.drive.setWheelSyncKP(TUNE_WHEEL_SYNC_KP);

    Serial.print("Wheel Sync KP: ");
    Serial.println(TUNE_WHEEL_SYNC_KP, 3);
    Serial.println(SPIN_MODE ? "Spin tuning, Diff = L + R" : "Straight tuning, Diff = L - R");

    // Start the move being tuned
    if (SPIN_MODE) {
        bot.drive.spin(DRIVE_SPEED_CM_S);
    } else {
        bot.drive.straight(DRIVE_SPEED_CM_S);
    }
}

void loop() {
    // Stop after 10 seconds
    static bool stopped = false;
    if (!stopped && millis() >= 10000) {
        bot.drive.stop();
        stopped = true;
        Serial.println("--- Stop commanded ---");
    }

    // Update the serial output at 10 Hz
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
        // Update the last print time
        lastPrintMs = millis();

        // Get the drive telemetry
        DriveTelemetry s = bot.drive.getTelemetry();

        // Balance error to drive toward zero. Straight wheels track together
        // (L - R); spin wheels run equal-and-opposite (L + R).
        int64_t diff = SPIN_MODE ? (s.leftCount + s.rightCount)
                                 : (s.leftCount - s.rightCount);

        // Average wheel speed. Use magnitudes so a spin (wheels opposite)
        // reports each wheel's tangential speed instead of ~0.
        float avgRPM = (fabsf(s.leftRPM) + fabsf(s.rightRPM)) / 2.0f;
        float speed_cm_s = (avgRPM * WHEEL_CIRCUMFERENCE_CM) / 60.0f;

        // Serial telemetry
        Serial.print("T: ");
        Serial.print(millis() / 1000.0, 1);
        Serial.print("s  Diff: ");
        Serial.print((long)diff);
        Serial.print("  L Enc: ");
        Serial.print((long)s.leftCount);
        Serial.print("  R Enc: ");
        Serial.print((long)s.rightCount);
        Serial.print("  L RPM: ");
        Serial.print(s.leftRPM, 1);
        Serial.print("  R RPM: ");
        Serial.print(s.rightRPM, 1);
        Serial.print("  Speed: ");
        Serial.print(speed_cm_s, 1);
        Serial.println(" cm/s");
    }

    // Run the drive control loop
    bot.drive.update();
}
