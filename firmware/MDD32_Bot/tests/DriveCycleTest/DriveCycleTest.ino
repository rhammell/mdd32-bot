/*
 * DriveCycleTest - Repeating drive/turn cycle for MDD32_Bot
 *
 * Drives straight forward for 5 seconds, pauses, spins in place for 3
 * seconds, pauses, and repeats indefinitely. Prints each phase change
 * over Serial.
 *
 * SAFETY: the bot drives itself forward - give it a clear runway, or prop
 * the wheels up so it runs in place.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

const float    DRIVE_SPEED_CM_S = 12.0f;   // modest forward speed
const float    SPIN_SPEED_CM_S  = 6.0f;   // wheel speed while turning (+ = clockwise)
const uint32_t DRIVE_MS         = 5000;   // time spent driving forward
const uint32_t SPIN_MS          = 3000;   // time spent turning
const uint32_t PAUSE_MS         = 1000;   // stopped time between moves

enum class Phase { Drive, PauseAfterDrive, Spin, PauseAfterSpin };

Phase phase;
uint32_t phaseStartMs = 0;

void enterPhase(Phase next) {
    phase = next;
    phaseStartMs = millis();

    switch (phase) {
        case Phase::Drive:
            bot.drive.straight(DRIVE_SPEED_CM_S);
            Serial.println("--- Driving forward ---");
            break;
        case Phase::Spin:
            bot.drive.spin(SPIN_SPEED_CM_S);
            Serial.println("--- Turning ---");
            break;
        case Phase::PauseAfterDrive:
        case Phase::PauseAfterSpin:
            bot.drive.stop();
            Serial.println("--- Stopped ---");
            break;
    }
}

uint32_t phaseDuration(Phase p) {
    switch (p) {
        case Phase::Drive: return DRIVE_MS;
        case Phase::Spin:  return SPIN_MS;
        default:           return PAUSE_MS;
    }
}

Phase nextPhase(Phase p) {
    switch (p) {
        case Phase::Drive:           return Phase::PauseAfterDrive;
        case Phase::PauseAfterDrive: return Phase::Spin;
        case Phase::Spin:            return Phase::PauseAfterSpin;
        default:                     return Phase::Drive;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
    Serial.println("MDD32 Bot Drive Cycle Test");

    if (!bot.begin()) {
        Serial.println("Failed to initialize MDD32 Bot!");
        while (1);
    }
    Serial.println("MDD32 Bot initialized successfully");

    enterPhase(Phase::Drive);
}

void loop() {
    // Run the drive control loop (no-op while stopped)
    bot.drive.update();

    if (millis() - phaseStartMs >= phaseDuration(phase)) {
        enterPhase(nextPhase(phase));
    }
}
