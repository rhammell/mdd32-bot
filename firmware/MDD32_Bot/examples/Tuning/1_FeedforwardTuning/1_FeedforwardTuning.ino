/*
 * 1_FeedforwardTuning - velocity feedforward characterization for MDD32_Bot
 *
 * Run this BEFORE PID tuning. It measures your motors' open-loop map of
 * "PWM in -> steady RPM out", which is exactly the data the velocity
 * feedforward table (PID_FF_RPM / PID_FF_PWM in robot_definitions.h) needs.
 * Feedforward is a physical property of the motor + gearbox + wheel + load +
 * battery; it does NOT depend on the PID gains, so it is measured open-loop
 * here with the PID bypassed entirely.
 *
 * What it does:
 *   Steps the raw motor PWM from PWM_START to PWM_MAX in PWM_STEP increments.
 *   It holds each level for DWELL_MS, averages the measured RPM over the final
 *   AVG_MS (once the speed has settled), and logs the result. At the end it
 *   prints a paste-ready PID_FF table.
 *
 * How to use it:
 *   1. Put the bot on the ground (or under representative load) with a
 *      well-charged battery - feedforward is load- and voltage-dependent.
 *   2. Compile, upload, open the Serial Monitor at 115200, and let the full
 *      sweep run without touching the bot. It drives forward the whole time.
 *   3. Copy the printed PID_FF_RPM / PID_FF_PWM / PID_FF_POINTS block into
 *      robot_definitions.h. Thin the points if you like - the table is linearly
 *      interpolated, so ~5-6 well-spaced points across your speed range are
 *      plenty. Add extra points where the curve bends most (usually the top).
 *   4. Set PID_FEEDFORWARD_ENABLE to 1, then move on to 2_PIDTuning.
 *
 * Notes:
 *   - The lowest PWM levels may report ~0 RPM: that is the motor's deadband
 *     (stiction). Levels below MIN_TABLE_RPM are shown in the log but omitted
 *     from the paste-ready table (only the 0,0 anchor represents the low end).
 *   - PWM_MAX defaults to full scale to discover the whole curve. You only need
 *     table points up to the PWM that reaches your top speed (MAX_SPEED_CM_S);
 *     higher rows can be dropped.
 */

#include <MDD32_Bot.h>

// MDD32 Bot instance
MDD32_Bot bot;

// --- Sweep configuration ---
// Raw PWM levels tested: PWM_START, PWM_START+PWM_STEP, ... up to PWM_MAX.
const int      PWM_START = 30;     // first level (near/just below the deadband)
const int      PWM_STEP  = 15;     // increment between levels
const int      PWM_MAX   = 255;    // last level (full scale = whole curve)
const uint32_t DWELL_MS  = 2500;   // time held at each level so speed settles
const uint32_t AVG_MS    = 1000;   // final window of each dwell averaged for RPM
const float    MIN_TABLE_RPM = 2.0f;  // levels slower than this are deadband

// Max number of sweep levels we can store
const int MAX_LEVELS = 64;

// Print the paste-ready PID_FF table from the collected sweep results
void printTable(const int* pwm, const float* rpm, int n) {
    Serial.println();
    Serial.println("===== Feedforward sweep complete =====");
    Serial.println("Paste into robot_definitions.h (thin points as desired):");
    Serial.println();

    // Count usable points (above the deadband) plus the 0,0 anchor
    Serial.print("constexpr float PID_FF_RPM[]   = {0.0f");
    int points = 1;
    for (int i = 0; i < n; i++) {
        if (rpm[i] < MIN_TABLE_RPM) continue;
        Serial.print(", ");
        Serial.print(rpm[i], 1);
        Serial.print("f");
        points++;
    }
    Serial.println("};");

    Serial.print("constexpr float PID_FF_PWM[]   = {0.0f");
    for (int i = 0; i < n; i++) {
        if (rpm[i] < MIN_TABLE_RPM) continue;
        Serial.print(", ");
        Serial.print(pwm[i]);
        Serial.print(".0f");
    }
    Serial.println("};");

    Serial.print("constexpr int   PID_FF_POINTS  = ");
    Serial.print(points);
    Serial.println(";");
    Serial.println();
    Serial.println("Then set PID_FEEDFORWARD_ENABLE to 1 and run 2_PIDTuning.");
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Feedforward Tuning");

    // Initialize the bot
    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");
    Serial.print("Sweeping PWM ");
    Serial.print(PWM_START);
    Serial.print(" to ");
    Serial.print(PWM_MAX);
    Serial.print(" in steps of ");
    Serial.println(PWM_STEP);

    // Countdown before driving
    for (int i = 5; i > 0; i--) {
        Serial.print("Sweeping in ");
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }
}

void loop() {
    // Sweep state
    static int      levelIdx = 0;
    static uint32_t levelStart = 0;
    static uint32_t lastSample = 0;
    static double   lSum = 0, rSum = 0;
    static int      nAvg = 0;
    static bool     done = false;

    // Collected results
    static int   resultPWM[MAX_LEVELS];
    static float resultRPM[MAX_LEVELS];
    static int   nResults = 0;

    // Once finished, hold the motors stopped
    if (done) {
        bot.drive.stop();
        return;
    }

    // Current PWM level under test
    int pwm = PWM_START + levelIdx * PWM_STEP;

    // Drive both wheels open-loop at this PWM (self-rate-limited internally)
    bot.drive.updateOpenLoop(pwm);

    // Start-of-level bookkeeping
    if (levelStart == 0) {
        levelStart = millis();
        lSum = rSum = 0;
        nAvg = 0;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - levelStart;

    // Sample, average and log every 100 ms
    if (now - lastSample >= 100) {
        lastSample = now;
        DriveTelemetry s = bot.drive.getTelemetry();

        // Accumulate only over the settled final window of the dwell
        if (elapsed >= DWELL_MS - AVG_MS) {
            lSum += s.leftRPM;
            rSum += s.rightRPM;
            nAvg++;
        }

        // Live serial row
        Serial.print("PWM ");
        Serial.print(pwm);
        Serial.print("  L RPM: ");
        Serial.print(s.leftRPM, 1);
        Serial.print("  R RPM: ");
        Serial.println(s.rightRPM, 1);
    }

    // End of this level: record the averaged steady RPM and advance
    if (elapsed >= DWELL_MS) {
        float lAvg = nAvg ? (float)(lSum / nAvg) : 0.0f;
        float rAvg = nAvg ? (float)(rSum / nAvg) : 0.0f;
        float avg = (lAvg + rAvg) / 2.0f;

        if (nResults < MAX_LEVELS) {
            resultPWM[nResults] = pwm;
            resultRPM[nResults] = avg;
            nResults++;
        }

        Serial.print(">> LEVEL PWM ");
        Serial.print(pwm);
        Serial.print("  ->  L ");
        Serial.print(lAvg, 1);
        Serial.print("  R ");
        Serial.print(rAvg, 1);
        Serial.print("  avg ");
        Serial.print(avg, 1);
        Serial.println(" RPM");

        // Advance to the next level, or finish
        levelIdx++;
        levelStart = 0;
        if (PWM_START + levelIdx * PWM_STEP > PWM_MAX || nResults >= MAX_LEVELS) {
            bot.drive.stop();
            printTable(resultPWM, resultRPM, nResults);
            done = true;
        }
    }
}
