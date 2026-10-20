/*
 * 2_PIDTuning - PID gain tuning for MDD32_Bot library
 *
 * Run this AFTER 1_FeedforwardTuning. The recommended order is: characterize the
 * velocity feedforward first (examples/Tuning/1_FeedforwardTuning), enable it, then tune
 * the PID on top of it here. With feedforward supplying the bulk of the holding
 * PWM, the PID only has to trim the small residual, so the gains (especially
 * KI) stay low and the loop is well behaved.
 *
 * Prerequisites before tuning:
 *   - PID_FEEDFORWARD_ENABLE = 1 with a PID_FF table from 1_FeedforwardTuning
 *     (or set it to 0 to tune pure PID with no feedforward).
 *   - WHEEL_SYNC_KP = 0 and MAX_ACCEL_RPM_S = 0 (ramping off), so the PID
 *     responds to a clean step input with no other control loops active.
 *   - Leave RPM_FILTER_ALPHA at its normal value (do NOT zero it) - the RPM
 *     filter is a passive part of the system the PID must be tuned against,
 *     not a competing control loop.
 *
 * Use this sketch to tune the PID speed controller gains (PID_KP,
 * PID_KI, PID_KD) defined in robot_definitions.h. Each change
 * requires recompiling and uploading.
 *
 * The sketch commands a constant speed and prints the measured RPM
 * so you can observe how the controller responds.
 *
 * Output modes (set PLOT_MODE below):
 *   0 = Human-readable Serial Monitor output
 *   1 = Arduino Serial Plotter output (Target, L_RPM, R_RPM)
 *
 * --- Quick PID Tuning Guide ---
 *
 * 1. Set PID_KI = 0 and start with a small PID_KP (e.g. 2.0) in
 *    robot_definitions.h. Compile, upload, and open Serial Monitor.
 *
 * 2. Watch how the reported RPM approaches the target.
 *    - Too slow to reach target: increase KP.
 *    - Oscillates around target or overshoots: decrease KP.
 *    Repeat until RPM rises quickly without oscillating. A small
 *    steady-state error (offset from target) is expected with KP alone.
 *    (With feedforward enabled the wheel starts near target, so the P
 *    response is small - that is expected.)
 *
 * 3. Add a small PID_KI (e.g. 1.0) to eliminate the steady-state error.
 *    - RPM still settles below target: increase KI.
 *    - RPM overshoots then slowly settles, or oscillates: decrease KI.
 *    With feedforward doing most of the work, KI can stay low - it only
 *    trims the residual the feedforward leaves behind.
 *
 * 4. Good result: RPM reaches and holds the target with no sustained
 *    oscillation and no visible overshoot. Convergence within a few
 *    seconds is acceptable — ramping will eliminate the large step
 *    response in normal operation.
 *
 * 5. Sanity-check the tune at several speeds across your range (set
 *    DRIVE_SPEED_CM_S below), since the motor is nonlinear. When done,
 *    re-enable MAX_ACCEL_RPM_S (see examples/Tuning/4_DistanceTuning) for smooth launches.
 */

#include <MDD32_Bot.h>

// MDD32 Bot instance
MDD32_Bot bot;

// Drive speed and target RPM
// Clamp to MAX_SPEED_CM_S so the printed target matches the setpoint
// straight() actually commands (it constrains speed to this limit)
const float DRIVE_SPEED_CM_S = 8.0f;
const float CLAMPED_SPEED_CM_S = constrain(DRIVE_SPEED_CM_S, -MAX_SPEED_CM_S, MAX_SPEED_CM_S);
const float TARGET_RPM = (CLAMPED_SPEED_CM_S * 60.0f) / WHEEL_CIRCUMFERENCE_CM;

// Plot mode
// 0 = Serial Monitor, 1 = Serial Plotter
const int   PLOT_MODE = 0;  

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot PID Tuning");

    // Initialize the bot
    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");

    // Print the target RPM
    Serial.print("Target RPM: ");
    Serial.println(TARGET_RPM, 1);

    // Countdown before driving
    for (int i = 5; i > 0; i--) {
        Serial.print("Driving in ");
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }

    // Drive forward at the target speed
    bot.drive.straight(DRIVE_SPEED_CM_S);
}

void loop() {
    // Print the motor status every 100 milliseconds
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
        // Update the last print time
        lastPrintMs = millis();

        // Get the drive telemetry
        DriveTelemetry s = bot.drive.getTelemetry();

        // Print the motor status in the plot mode
        if (PLOT_MODE) {
            Serial.print("Target:");
            Serial.print(TARGET_RPM, 1);
            Serial.print(" L_RPM:");
            Serial.print(s.leftRPM, 1);
            Serial.print(" R_RPM:");
            Serial.println(s.rightRPM, 1);

        // Print the motor status in the serial monitor mode
        } else {
            Serial.print("T: ");
            Serial.print(millis() / 1000.0, 1);
            Serial.print("s  Target: ");
            Serial.print(TARGET_RPM, 1);
            Serial.print("  L RPM: ");
            Serial.print(s.leftRPM, 1);
            Serial.print("  L PWM: ");
            Serial.print(s.leftPWM, 0);
            Serial.print("  R RPM: ");
            Serial.print(s.rightRPM, 1);
            Serial.print("  R PWM: ");
            Serial.println(s.rightPWM, 0);
        }
    }

    // Update the drive subsystem
    bot.drive.update();
}
