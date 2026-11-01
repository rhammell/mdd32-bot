#include "MDD32_PID.h"

// =============================================================================
// MDD32_PID Implementation
// =============================================================================

void MDD32_PID::begin(float kp, float ki, float kd, float outputMin, float outputMax) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _outputMin = outputMin;
    _outputMax = outputMax;
    reset();
}

void MDD32_PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void MDD32_PID::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}

void MDD32_PID::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

float MDD32_PID::compute(float measurement, float dt) {
    // Check if the time delta is valid (i.e. not zero)
    if (dt <= 0.0f) return 0.0f;

    // Calculate the error between the setpoint and the measurement
    float error = _setpoint - measurement;

    // Proportional term
    float pTerm = _kp * error;

    // Integral term with anti-windup clamping, so the integral alone can never
    // demand more than the full output range
    _integral += error * dt;
    if (_ki != 0.0f) {
        _integral = constrain(_integral, _outputMin / _ki, _outputMax / _ki);
    }
    float iTerm = _ki * _integral;

    // Derivative term (on error)
    float dTerm = _kd * (error - _prevError) / dt;
    _prevError = error;

    // Sum the PID terms and clamp output
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, _outputMin, _outputMax);

    return output;
}

void MDD32_PID::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
}
