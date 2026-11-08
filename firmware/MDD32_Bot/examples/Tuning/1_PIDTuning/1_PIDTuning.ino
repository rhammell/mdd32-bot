/*
 * 1_PIDTuning - PID gain tuning for MDD32_Bot library
 *
 * Run this first. Each wheel has its own PID speed loop (shared gains), and
 * the remaining tuning steps (wheel sync, acceleration) build on top of it.
 *
 * How this sketch works:
 *   - Place the bot on its tuning stand (wheels elevated so they spin freely)
 *     with the USB cable connected.
 *   - The sketch commands a constant speed and prints the measured RPM so you
 *     can observe how the controller responds.
 *   - Adjust TUNE_KP, TUNE_KI, and TUNE_KD right below. They start at the
 *     library defaults from robot_definitions.h. Each change still requires
 *     recompiling and uploading, but no edits to robot_definitions.h between
 *     runs.
 *   - setup() automatically disables ramping (setMaxAccel(0)) and wheel sync
 *     (setWheelSyncKP(0)) so the PID responds to a clean step input with no
 *     other control loops active.
 *   - Leave RPM_FILTER_ALPHA at its normal value (do NOT zero it) - the RPM
 *     filter is a passive part of the system the PID must be tuned against,
 *     not a competing control loop.
 *   - The goal is a set of gains that brings each wheel to the target RPM
 *     within about 0.5 seconds and holds it there without oscillating.
 *     During normal driving the ramp (MAX_ACCEL_RPM_S) takes about 1
 *     second to reach full speed, and the wheel trails the ramp by roughly
 *     the PID's own response time. A PID that settles in 0.5 seconds or less
 *     keeps that lag small, so the bot reaches its target speed shortly
 *     after the ramp ends.
 *     Adjust TUNE_KP until the wheel responds quickly with only a small
 *     initial overshoot, then adjust TUNE_KI until the remaining error
 *     closes within the 0.5 second target without oscillation. See the
 *     tuning guide below for the detailed steps.
 *   - Final gains can be applied in one of two ways: copy them into
 *     src/definitions/robot_definitions.h (PID_KP, PID_KI, PID_KD) to make
 *     them the library-wide defaults, or pass them to
 *     bot.drive.setPIDGains() in the setup() of an individual sketch.
 *
 * Output modes (set PLOT_MODE below):
 *   0 = Human-readable Serial Monitor output
 *   1 = Arduino Serial Plotter output (Target, L_RPM, R_RPM)
 *
 * --- Quick PID Tuning Guide ---
 *
 * 1. To tune from scratch, set TUNE_KI = 0 and start with a small TUNE_KP
 *    (e.g. 2.0) below. Compile, upload, and open Serial Monitor.
 *
 * 2. Tune TUNE_KP for a fast initial response. Watch how the reported RPM
 *    approaches the target.
 *    - Rises slowly or settles far below target: increase TUNE_KP.
 *    - Overshoots and bounces at the start, or oscillates: decrease TUNE_KP.
 *    With KP alone the wheel always settles below target (steady-state
 *    error), and each increase gains less. Stop once the first bounce after
 *    the start becomes large.
 *
 * 3. Tune TUNE_KI to remove the steady-state error within about 0.5
 *    seconds. The integral builds up the PWM each wheel needs to hold the
 *    target speed. Use the printed time (T) to judge how long the wheel
 *    takes to settle at the target.
 *    - Takes longer than about 0.5 seconds to reach target: increase
 *      TUNE_KI.
 *    - Overshoots past the target or oscillates: first try increasing
 *      TUNE_KP slightly to add damping; if that does not help, decrease
 *      TUNE_KI.
 *    A small overshoot here (about 5-10%) is acceptable, since the ramp
 *    used in normal driving keeps the error small and reduces it further.
 *
 * 4. Good result: RPM reaches the target within about 0.5 seconds and
 *    holds it, with little overshoot and no sustained oscillation. If the
 *    printed PWM jitters noticeably once the speed has settled, TUNE_KP is
 *    amplifying encoder noise and should be lowered. If the PWM sits at 255
 *    while the wheel is speeding up, the motor is at its limit and higher
 *    gains will not make it faster.
 *
 * 5. Sanity-check the tune at several speeds across your range (set
 *    DRIVE_SPEED_CM_S below), since the motor is nonlinear. When done, copy
 *    your final TUNE_KP / TUNE_KI / TUNE_KD into robot_definitions.h.
 */

#include <MDD32_Bot.h>

// MDD32 Bot instance
MDD32_Bot bot;

// --- PID Tuning Parameters (edit these for each test run) ---
const float TUNE_KP = PID_KP;
const float TUNE_KI = PID_KI;
const float TUNE_KD = PID_KD;

// Drive speed and target RPM
// Clamp to MAX_SPEED_CM_S so the printed target matches the setpoint
// straight() actually commands (it constrains speed to this limit)
const float DRIVE_SPEED_CM_S = 6.0f;
const float CLAMPED_SPEED_CM_S = constrain(DRIVE_SPEED_CM_S, -MAX_SPEED_CM_S, MAX_SPEED_CM_S);
const float TARGET_RPM = (CLAMPED_SPEED_CM_S * 60.0f) / WHEEL_CIRCUMFERENCE_CM;

// Plot mode
// 0 = Serial Monitor, 1 = Serial Plotter
const int   PLOT_MODE = 0;  

// Time the drive command was issued; printed times are relative to this
uint32_t driveStartMs = 0;

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

    // Isolate the per-wheel speed loop for a clean step input, and apply the
    // tuning parameters configured at the top of this sketch
    bot.drive.setMaxAccel(0.0f);
    bot.drive.setWheelSyncKP(0.0f);
    bot.drive.setPIDGains(TUNE_KP, TUNE_KI, TUNE_KD);

    // Print the active configuration and target RPM
    Serial.print("KP: ");
    Serial.print(TUNE_KP, 2);
    Serial.print("  KI: ");
    Serial.print(TUNE_KI, 2);
    Serial.print("  KD: ");
    Serial.println(TUNE_KD, 2);
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
    driveStartMs = millis();
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
            Serial.print((millis() - driveStartMs) / 1000.0, 1);
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
