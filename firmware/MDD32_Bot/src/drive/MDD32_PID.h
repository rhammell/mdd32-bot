#pragma once

#include <Arduino.h>

// Discrete PID controller.
// Accepts a setpoint and measurement (e.g. RPM), outputs a control signal
// (e.g. PWM value) clamped to a configurable range.
class MDD32_PID {
public:
    MDD32_PID() = default;

    void begin(float kp, float ki, float kd, float outputMin, float outputMax);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setSetpoint(float setpoint);

    // Compute the PID output given the current measurement and time step.
    float compute(float measurement, float dt);

    void reset();

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _setpoint = 0.0f;

    float _integral   = 0.0f;
    float _prevError  = 0.0f;

    float _outputMin = -255.0f;
    float _outputMax =  255.0f;
};
