/*
 * StepResponseTest - Measures how quickly the wheels reach a target speed
 *
 * Commands a step from rest to each speed in TEST_SPEEDS_CM_S and measures the
 * closed-loop response of each wheel. Configure TEST_KP / TEST_KI / TEST_KD
 * right below to compare gain sets without editing robot_definitions.h.
 * setup() automatically disables ramping
 * (setMaxAccel(0)) and wheel sync (setWheelSyncKP(0)) so the step response
 * reflects only the per-wheel speed controller.
 *
 * Metrics reported per wheel, per speed (times measured from the step command):
 *   - Start:     first time the RPM exceeds 5% of target (wheel breaks away)
 *   - Rise:      first time the RPM reaches 90% of target
 *   - Settle:    time after which the RPM stays within +/-SETTLE_BAND_PCT of target
 *                ("--" if it never settles within TRIAL_MS)
 *   - Overshoot: peak RPM above target, as a percent of target
 *   - SS err:    mean RPM error over the final STEADY_WINDOW_MS of the trial
 *
 * Set PRINT_SAMPLES = true to also dump every control-tick sample as CSV
 * (lines prefixed "S,") for plotting the two responses against each other.
 *
 * SAFETY: the bot drives itself forward - give it a clear runway, or prop
 * the wheels up so it runs in place.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

// --- Controller configuration under test (edit here to compare runs) ---
const float    TEST_KP            = PID_KP;
const float    TEST_KI            = PID_KI;
const float    TEST_KD            = PID_KD;

// Step targets to test, in cm/s (clamped to MAX_SPEED_CM_S by straight())
const float    TEST_SPEEDS_CM_S[] = {4.0f, 8.0f, 10.0f};
const int      NUM_SPEEDS         = sizeof(TEST_SPEEDS_CM_S) / sizeof(TEST_SPEEDS_CM_S[0]);

const uint32_t TRIAL_MS           = 4000;  // how long to hold each step
const uint32_t REST_MS            = 2000;  // pause between trials so the wheels fully stop
const uint32_t STEADY_WINDOW_MS   = 1000;  // final window used for steady-state error
const uint32_t SAMPLE_MS          = 20;    // matches DRIVE_UPDATE_INTERVAL_US
const float    START_PCT          = 5.0f;
const float    RISE_PCT           = 90.0f;
const float    SETTLE_BAND_PCT    = 5.0f;
const bool     PRINT_SAMPLES      = false;

// Running step-response metrics for one wheel
struct WheelMetrics {
    int32_t  startMs = -1;
    int32_t  riseMs = -1;
    uint32_t lastOutsideMs = 0;
    float    peakRPM = 0.0f;
    float    steadySum = 0.0f;
    uint32_t steadyCount = 0;
};

void updateMetrics(WheelMetrics& m, float rpm, float target, uint32_t t) {
    float mag = fabsf(rpm);
    float tgt = fabsf(target);

    if (m.startMs < 0 && mag >= tgt * START_PCT / 100.0f) m.startMs = t;
    if (m.riseMs < 0 && mag >= tgt * RISE_PCT / 100.0f)   m.riseMs = t;
    if (fabsf(mag - tgt) > tgt * SETTLE_BAND_PCT / 100.0f) m.lastOutsideMs = t;
    if (mag > m.peakRPM) m.peakRPM = mag;

    if (t >= TRIAL_MS - STEADY_WINDOW_MS) {
        m.steadySum += mag;
        m.steadyCount++;
    }
}

void printMs(int32_t ms) {
    if (ms < 0) {
        Serial.print("    --");
    } else {
        char buf[12];
        snprintf(buf, sizeof(buf), "%6ld", (long)ms);
        Serial.print(buf);
    }
}

void printMetrics(const char* side, const WheelMetrics& m, float target) {
    float tgt = fabsf(target);
    float steady = m.steadyCount ? m.steadySum / m.steadyCount : 0.0f;
    float overshootPct = (m.peakRPM > tgt) ? (m.peakRPM - tgt) / tgt * 100.0f : 0.0f;

    // Never left the band after the last sample means it settled at lastOutside + 1 tick;
    // if it was still outside at the end of the trial, it did not settle.
    int32_t settleMs = (m.lastOutsideMs >= TRIAL_MS - SAMPLE_MS)
                       ? -1 : (int32_t)(m.lastOutsideMs + SAMPLE_MS);

    Serial.print("  ");
    Serial.print(side);
    Serial.print("  Start(ms):");
    printMs(m.startMs);
    Serial.print("  Rise(ms):");
    printMs(m.riseMs);
    Serial.print("  Settle(ms):");
    printMs(settleMs);
    Serial.print("  Overshoot: ");
    Serial.print(overshootPct, 1);
    Serial.print("%  SS RPM: ");
    Serial.print(steady, 1);
    Serial.print("  SS err: ");
    Serial.println(steady - tgt, 1);
}

void runTrial(float speedCmS) {
    float clamped = constrain(speedCmS, -MAX_SPEED_CM_S, MAX_SPEED_CM_S);
    float targetRPM = (clamped * 60.0f) / WHEEL_CIRCUMFERENCE_CM;

    Serial.println();
    Serial.print("=== Step to ");
    Serial.print(clamped, 1);
    Serial.print(" cm/s (");
    Serial.print(targetRPM, 1);
    Serial.println(" RPM) ===");
    if (PRINT_SAMPLES) Serial.println("S,t_ms,target,L_RPM,L_PWM,R_RPM,R_PWM");

    WheelMetrics left, right;

    bot.drive.straight(speedCmS);
    uint32_t stepMs = millis();
    uint32_t lastSampleMs = stepMs;

    while (true) {
        bot.drive.update();

        uint32_t now = millis();
        if (now - lastSampleMs < SAMPLE_MS) continue;
        lastSampleMs = now;

        uint32_t t = now - stepMs;
        if (t > TRIAL_MS) break;

        DriveTelemetry s = bot.drive.getTelemetry();
        updateMetrics(left, s.leftRPM, targetRPM, t);
        updateMetrics(right, s.rightRPM, targetRPM, t);

        if (PRINT_SAMPLES) {
            Serial.print("S,");
            Serial.print(t);
            Serial.print(",");
            Serial.print(targetRPM, 1);
            Serial.print(",");
            Serial.print(s.leftRPM, 1);
            Serial.print(",");
            Serial.print(s.leftPWM, 0);
            Serial.print(",");
            Serial.print(s.rightRPM, 1);
            Serial.print(",");
            Serial.println(s.rightPWM, 0);
        }
    }

    bot.drive.stop();

    printMetrics("L", left, targetRPM);
    printMetrics("R", right, targetRPM);

    delay(REST_MS);
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Step Response Test");

    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");

    // Isolate the per-wheel speed loop (no ramp, no wheel sync) and apply the
    // controller configuration defined at the top of this sketch
    bot.drive.setMaxAccel(0.0f);
    bot.drive.setWheelSyncKP(0.0f);
    bot.drive.setPIDGains(TEST_KP, TEST_KI, TEST_KD);

    // Label the run so logs from different gain sets can't be mixed up
    Serial.print("Gains: KP=");
    Serial.print(TEST_KP, 2);
    Serial.print(" KI=");
    Serial.print(TEST_KI, 2);
    Serial.print(" KD=");
    Serial.println(TEST_KD, 2);

    for (int i = 5; i > 0; i--) {
        Serial.print("Starting in ");
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }

    for (int i = 0; i < NUM_SPEEDS; i++) {
        runTrial(TEST_SPEEDS_CM_S[i]);
    }

    Serial.println();
    Serial.println("--- Step response test complete ---");
}

void loop() {
}
