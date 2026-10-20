/*
 * 2_PIDTuning - PID gain tuning for MDD32_Bot library
 *
 * Run this AFTER 1_FeedforwardTuning. The recommended order is: characterize the
 * velocity feedforward first (examples/Tuning/1_FeedforwardTuning), enable it,
 * then tune the PID on top of it here. With feedforward supplying the bulk of
 * the holding PWM, the PID only has to trim the small residual, so the gains
 * (especially KI) stay low and the loop is well behaved.
 *
 * How this sketch works:
 *   - Place the bot on its tuning stand (wheels elevated so they spin freely)
 *     with the USB cable connected.
 *   - Adjust TUNE_KP, TUNE_KI, TUNE_KD, and USE_FEEDFORWARD right below and
 *     upload to test each change - no need to edit robot_definitions.h between
 *     runs.
 *   - setup() automatically disables ramping (setMaxAccel(0)) and wheel sync
 *     (setWheelSyncKP(0)) so the PID responds to a clean step input with no
 *     other control loops active.
 *   - Once you find gains you are happy with, either copy them into
 *     src/definitions/robot_definitions.h (PID_KP, PID_KI, PID_KD) to make them
 *     the library-wide defaults, or call bot.drive.setPIDGains() in your own
 *     sketch's setup().
 *
 * Output modes (set PLOT_MODE below):
 *   0 = Human-readable Serial Monitor output
 *   1 = Arduino Serial Plotter output (Target, L_RPM, R_RPM)
 *
 * --- Quick PID Tuning Guide ---
 *
 * 1. Set TUNE_KI = 0 and start with a small TUNE_KP (e.g. 2.0) below.
 *    Compile, upload, and open Serial Monitor.
 *
 * 2. Watch how the reported RPM approaches the target.
 *    - Too slow to reach target: increase TUNE_KP.
 *    - Oscillates around target or overshoots: decrease TUNE_KP.
 *    Repeat until RPM rises quickly without oscillating. A steady-state error
 *    (offset from target) is expected with KP alone.
 *    (With feedforward enabled the wheel starts near target, so the P
 *    response is small - that is expected.)
 *
 * 3. Add a small TUNE_KI (e.g. 1.0) to eliminate the steady-state error.
 *    - RPM still settles below target: increase TUNE_KI.
 *    - RPM overshoots then slowly settles, or oscillates: decrease TUNE_KI.
 *    With feedforward doing most of the work, KI can stay low - it only
 *    trims the residual the feedforward leaves behind.
 *
 * 4. Good result: RPM reaches and holds the target with no sustained
 *    oscillation and no visible overshoot.
 *
 * 5. Sanity-check the tune at several speeds across your range (set
 *    DRIVE_SPEED_CM_S below), since the motor is nonlinear. When done, copy
 *    your final TUNE_KP / TUNE_KI / TUNE_KD into robot_definitions.h.
 */

#include <MDD32_Bot.h>

// MDD32 Bot instance
MDD32_Bot bot;

// --- PID Tuning Parameters (edit these for each test run) ---
const bool  USE_FEEDFORWARD = false;  // true = feedforward + PID, false = pure PID
const float TUNE_KP         = 2.0f;   // default with feedforward: PID_KP (6.0f)
const float TUNE_KI         = 0.0f;   // default with feedforward: PID_KI (4.0f)
const float TUNE_KD         = 0.0f;   // default with feedforward: PID_KD (0.0f)

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

    // Isolate the per-wheel speed loop for a clean step input, and apply the
    // tuning parameters configured at the top of this sketch
    bot.drive.setMaxAccel(0.0f);
    bot.drive.setWheelSyncKP(0.0f);
    bot.drive.setFeedforwardEnabled(USE_FEEDFORWARD);
    bot.drive.setPIDGains(TUNE_KP, TUNE_KI, TUNE_KD);

    // Print the active configuration and target RPM
    Serial.print("Feedforward: ");
    Serial.print(USE_FEEDFORWARD ? "ON" : "OFF");
    Serial.print("  KP: ");
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
