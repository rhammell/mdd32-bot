#pragma once

#include <Arduino.h>
#include "MDD32_Motor.h"
#include "MDD32_Encoder.h"
#include "MDD32_PID.h"

// Per-side wiring for a motor controller. Behavioral tuning (PID gains,
// encoder mode) is read from robot_definitions.h inside begin(), not passed here.
struct MotorControllerConfig {
    uint8_t in1Pin, in2Pin;
    bool    motorFlip;
    uint8_t encAPin, encBPin;
    bool    encFlip;
};

// Coordinates a single motor/encoder pair with PID speed control.
class MDD32_Motor_Controller {
public:
    // Constructor
    MDD32_Motor_Controller() = default;

    // Prevent copies (encoder owns a unique PCNT unit)
    MDD32_Motor_Controller(const MDD32_Motor_Controller&) = delete;
    MDD32_Motor_Controller& operator=(const MDD32_Motor_Controller&) = delete;

    // Public methods
    bool begin(const MotorControllerConfig& config);
    void setRPM(float rpm);
    void update(float dt);
    void stop();   // Stops the motor; does NOT clear PID/encoder state - call reset() before reusing
    void reset();

    // Runtime tuning override (initialized from robot_definitions.h in begin())
    void setPIDGains(float kp, float ki, float kd);

    // Telemetry accessors
    float   getRPM() const { return _currentRPM; }
    float   getPWM() const { return _currentPWM; }
    int64_t getEncoderCount() const { return encoder.getCount(); }

private:
    // Private members
    MDD32_Motor   motor;
    MDD32_Encoder encoder;
    MDD32_PID     pid;
    float          _currentRPM = 0.0f;
    float          _currentPWM = 0.0f;
    int64_t        _lastEncoderCount = 0;
};
