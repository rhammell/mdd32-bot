/*
 * MaxSpeedTest - Measures the open-loop top speed of each wheel
 *
 * Drives both motors directly at fixed PWM levels (bypassing the PID loop,
 * ramping, wheel sync, and the MAX_SPEED_CM_S clamp) and measures the
 * steady-state speed of each wheel from raw encoder counts. The final step
 * at PWM 255 is the true top speed for the current battery voltage.
 *
 * Each PWM level in TEST_PWMS is held for HOLD_MS; speed is averaged over
 * the final MEASURE_MS of the hold so the spin-up transient is excluded.
 * Levels run back-to-back (no stop in between) to keep the run short.
 *
 * Run it twice:
 *   1. Wheels propped up  -> no-load speed (upper bound)
 *   2. On the floor       -> loaded speed (what the bot can actually do)
 * Use the slower wheel's loaded speed at PWM 255 to pick MAX_SPEED_CM_S,
 * leaving headroom so the PID can still correct (see suggestion printed
 * at the end).
 *
 * Untethered runs: the test starts START_DELAY_S after power-on, and once
 * it finishes the results are reprinted every RESULT_PRINT_MS forever. Upload,
 * unplug USB, switch the bot on and set it down, then plug back in and open
 * the Serial Monitor to read the results.
 *
 * Top speed scales with battery voltage, so the voltage is logged with each
 * run - compare runs at a similar charge level.
 *
 * SAFETY: on the floor the bot drives at FULL speed for
 * NUM_PWMS * HOLD_MS - give it a long, clear runway.
 */

#include <MDD32_Bot.h>

// The drive is driven directly here, so bot.begin() is not used (it would
// claim the same motor pins and encoder PCNT units).
MDD32_Battery battery;
MDD32_Motor   leftMotor, rightMotor;
MDD32_Encoder leftEncoder, rightEncoder;

const int      TEST_PWMS[]       = {128, 192, 255};
const int      NUM_PWMS          = sizeof(TEST_PWMS) / sizeof(TEST_PWMS[0]);
const uint32_t HOLD_MS           = 2500;  // time at each PWM level
const uint32_t MEASURE_MS        = 1000;  // final window used for the average
const float    HEADROOM_PCT      = 80.0f; // suggested MAX_SPEED_CM_S as % of top speed
const int      START_DELAY_S     = 10;    // time to set the bot down after power-on
const uint32_t RESULT_PRINT_MS   = 3000;  // how often the results are reprinted

// Results, kept so they can be reprinted after the run
float rpmL[NUM_PWMS];
float rpmR[NUM_PWMS];
float startVbat = 0.0f;
float minVbat   = 0.0f;

float ticksToRPM(int64_t ticks, uint32_t ms) {
    return (ticks / (float)ENCODER_TICKS_PER_REV) * (60000.0f / ms);
}

float rpmToCmS(float rpm) {
    return rpm * WHEEL_CIRCUMFERENCE_CM / 60.0f;
}

void printWheel(const char* side, float rpm) {
    Serial.print("  ");
    Serial.print(side);
    Serial.print("  RPM: ");
    Serial.print(rpm, 1);
    Serial.print("  Speed: ");
    Serial.print(rpmToCmS(rpm), 1);
    Serial.println(" cm/s");
}

void printResults() {
    Serial.println();
    Serial.println("===== Max speed test results =====");
    Serial.print("Wheel diameter: ");
    Serial.print(WHEEL_DIAMETER_CM, 2);
    Serial.println(" cm");

    for (int i = 0; i < NUM_PWMS; i++) {
        Serial.print("PWM ");
        Serial.println(TEST_PWMS[i]);
        printWheel("L", rpmL[i]);
        printWheel("R", rpmR[i]);
    }

    float slowerCmS = rpmToCmS(min(rpmL[NUM_PWMS - 1], rpmR[NUM_PWMS - 1]));
    Serial.print("Top speed (slower wheel): ");
    Serial.print(slowerCmS, 1);
    Serial.println(" cm/s");
    Serial.print("Battery at start: ");
    Serial.print(startVbat, 2);
    Serial.print(" V  Min under load: ");
    Serial.print(minVbat, 2);
    Serial.println(" V");
    Serial.print("Suggested MAX_SPEED_CM_S (");
    Serial.print(HEADROOM_PCT, 0);
    Serial.print("% headroom): ");
    Serial.println(slowerCmS * HEADROOM_PCT / 100.0f, 1);
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Max Speed Test");

    battery.begin();

    pinMode(DRV_SLEEP, OUTPUT);
    digitalWrite(DRV_SLEEP, HIGH);

    EncoderMode mode = (ENCODER_MODE == 4) ? EncoderMode::X4 : EncoderMode::X2;
    leftMotor.begin(MOT_LEFT_IN1, MOT_LEFT_IN2, MOT_LEFT_FLIP);
    rightMotor.begin(MOT_RIGHT_IN1, MOT_RIGHT_IN2, MOT_RIGHT_FLIP);
    if (!leftEncoder.begin(MOT_LEFT_ENC_A, MOT_LEFT_ENC_B, MOT_LEFT_ENC_FLIP, mode) ||
        !rightEncoder.begin(MOT_RIGHT_ENC_A, MOT_RIGHT_ENC_B, MOT_RIGHT_ENC_FLIP, mode)) {
        Serial.println("Encoder init FAILED");
        while (1);
    }

    for (int i = START_DELAY_S; i > 0; i--) {
        Serial.print("Starting in ");
        Serial.print(i);
        Serial.println("...");
        delay(1000);
    }

    startVbat = battery.voltage();
    minVbat = startVbat;

    for (int i = 0; i < NUM_PWMS; i++) {
        int pwm = TEST_PWMS[i];
        leftMotor.setPWM(pwm);
        rightMotor.setPWM(pwm);

        delay(HOLD_MS - MEASURE_MS);
        int64_t startL = leftEncoder.getCount();
        int64_t startR = rightEncoder.getCount();
        uint32_t startMs = millis();

        // Sample battery sag under load during the measurement window
        while (millis() - startMs < MEASURE_MS) {
            minVbat = min(minVbat, battery.voltage());
            delay(50);
        }

        uint32_t elapsedMs = millis() - startMs;
        rpmL[i] = ticksToRPM(leftEncoder.getCount() - startL, elapsedMs);
        rpmR[i] = ticksToRPM(rightEncoder.getCount() - startR, elapsedMs);
    }

    leftMotor.stop();
    rightMotor.stop();
}

void loop() {
    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();
    if (lastPrintMs == 0 || now - lastPrintMs >= RESULT_PRINT_MS) {
        lastPrintMs = now;
        printResults();
    }
}
