/*
 * DriftTest - Coast-down drift measurement for MDD32_Bot library
 *
 * Drives the bot at max speed for a couple seconds, calls stop()
 * (which cuts the motors and lets them coast), and measures how far
 * the wheels keep turning before coming to rest.
 *
 * Encoder counts are captured at two moments:
 *   1. The instant stop() is executed.
 *   2. After the bot has fully settled.
 * The difference is the coast-down drift, shown in ticks and cm.
 *
 * The encoders are hardware counters, so they keep counting during the
 * coast even though the drive control loop goes idle after stop().
 *
 * Results are printed over Serial, including live counts while settling.
 *
 * SAFETY: the bot drives at full speed then coasts - give it a long,
 * clear runway, or prop the wheels up for a no-load coast measurement.
 */

#include <MDD32_Bot.h>

MDD32_Bot bot;

const float    DRIVE_SPEED_CM_S = MAX_SPEED_CM_S;  // command max speed
const uint32_t DRIVE_MS         = 2500;            // time spent driving
const uint32_t SETTLE_MS        = 3000;            // time to allow coast to rest

enum Phase { DRIVING, SETTLING, DONE };
Phase phase = DRIVING;
uint32_t phaseStartMs = 0;

int64_t countAtStopL  = 0, countAtStopR  = 0;
int64_t countSettledL = 0, countSettledR = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when USB is connected but no monitor is open
  Serial.println("MDD32 Bot Drift Test");

  if (!bot.begin()) {
    Serial.println("Failed to initialize MDD32 Bot!");
    while (1);
  }
  Serial.println("MDD32 Bot initialized successfully");

  // Start driving forward at max speed.
  bot.drive.straight(DRIVE_SPEED_CM_S);
  phase = DRIVING;
  phaseStartMs = millis();
  Serial.print("Driving at ");
  Serial.print(DRIVE_SPEED_CM_S, 1);
  Serial.println(" cm/s");
}

void printDrift() {
  int64_t driftL = countSettledL - countAtStopL;
  int64_t driftR = countSettledR - countAtStopR;
  float driftCmL = (float)driftL / TICKS_PER_CM;
  float driftCmR = (float)driftR / TICKS_PER_CM;

  Serial.print("Drift L: ");
  Serial.print((long)driftL);
  Serial.print(" ticks (");
  Serial.print(driftCmL, 2);
  Serial.print(" cm) | R: ");
  Serial.print((long)driftR);
  Serial.print(" ticks (");
  Serial.print(driftCmR, 2);
  Serial.println(" cm)");
}

void loop() {
  // Keep the drive control loop running (no-op once stopped).
  bot.drive.update();

  uint32_t now = millis();

  switch (phase) {
    case DRIVING:
      if (now - phaseStartMs >= DRIVE_MS) {
        // Capture counts at the instant of stop, then cut the motors.
        DriveTelemetry s = bot.drive.getTelemetry();
        countAtStopL = s.leftCount;
        countAtStopR = s.rightCount;
        bot.drive.stop();

        phase = SETTLING;
        phaseStartMs = now;

        Serial.print("STOP at L: ");
        Serial.print((long)countAtStopL);
        Serial.print(" R: ");
        Serial.println((long)countAtStopR);
      }
      break;

    case SETTLING: {
      // Print the counts live as the bot coasts to rest.
      static uint32_t lastRefreshMs = 0;
      if (now - lastRefreshMs >= 100) {
        lastRefreshMs = now;
        DriveTelemetry s = bot.drive.getTelemetry();
        Serial.print("  settling L: ");
        Serial.print((long)s.leftCount);
        Serial.print(" R: ");
        Serial.println((long)s.rightCount);
      }

      if (now - phaseStartMs >= SETTLE_MS) {
        DriveTelemetry s = bot.drive.getTelemetry();
        countSettledL = s.leftCount;
        countSettledR = s.rightCount;

        phase = DONE;

        Serial.print("SETTLED at L: ");
        Serial.print((long)countSettledL);
        Serial.print(" R: ");
        Serial.println((long)countSettledR);
        printDrift();
      }
      break;
    }

    case DONE:
      // Results are printed; nothing more to do.
      break;
  }
}
