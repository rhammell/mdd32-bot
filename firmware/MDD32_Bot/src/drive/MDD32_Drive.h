#pragma once

#include <Arduino.h>
#include "../definitions/pin_definitions.h"
#include "../definitions/robot_definitions.h"
#include "MDD32_Motor_Controller.h"

struct DriveTelemetry {
    float leftRPM;
    float leftPWM;
    float rightRPM;
    float rightPWM;
    int64_t leftCount;
    int64_t rightCount;
};

// Differential drive controller coordinating left and right motor controllers.
class MDD32_Drive {
public:
    // Constructor
    MDD32_Drive() = default;

    // Public methods
    bool begin();
    void straight(float speed_cm_s);
    void spin(float speed_cm_s);
    void stop();
    void update();
    bool isMoving() const;
    DriveTelemetry getTelemetry() const;

    // Feedforward characterization only (see examples/Tuning/1_FeedforwardTuning): drive
    // both wheels at a fixed open-loop PWM (-255..255) with the PID, feedforward
    // and ramp all bypassed, refreshing RPM telemetry at the normal control
    // interval. Call repeatedly from loop() to log the steady RPM each PWM level
    // sustains, then read getTelemetry(). Not used in normal driving.
    void updateOpenLoop(int pwm);

    // Cumulative straight-line distance, in cm, driven since power-on. Grows
    // for both forward and backward motion; pure in-place spins do not add to it.
    float getDistance() const;
    void resetDistance();

private:
    // Drive motion state: Idle (stopped), Straight (wheels together, translate),
    // or Spin (wheels equal-and-opposite, rotate in place).
    enum class DriveMode { Idle, Straight, Spin };

    // Initializes one motor controller with the shared PID gains and the given
    // wiring. Returns false on failure.
    bool initController(MDD32_Motor_Controller& controller,
                        uint8_t in1Pin, uint8_t in2Pin, bool motorFlip,
                        uint8_t encAPin, uint8_t encBPin, bool encFlip);

    // Shared start-of-move logic for straight() and spin(). mode selects
    // whether the wheels run together (translate) or opposite (rotate).
    void startMove(float speed_cm_s, DriveMode mode);

    // Private members
    MDD32_Motor_Controller _leftController;
    MDD32_Motor_Controller _rightController;
    DriveMode _mode = DriveMode::Idle;
    float    _targetRPM = 0;
    float    _commandedRPM = 0;   // ramped value actually sent to the wheels
    uint32_t _lastUpdateTime = 0;

    // Odometry: accumulated centerline distance plus the encoder counts from
    // the previous update() used to compute each tick's delta.
    float    _distanceCm = 0.0f;
    int64_t  _lastLeftCount = 0;
    int64_t  _lastRightCount = 0;
};
