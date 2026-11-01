#include "MDD32_Motor_Controller.h"
#include "../definitions/robot_definitions.h"

// =============================================================================
// MDD32_Motor_Controller Implementation
// =============================================================================

bool MDD32_Motor_Controller::begin(const MotorControllerConfig& config) {
    // Initialize H-bridge PWM outputs (motorFlip swaps pins so +PWM = forward)
    motor.begin(config.in1Pin, config.in2Pin, config.motorFlip);

    // Initialize quadrature encoder via PCNT hardware 
    EncoderMode mode = (ENCODER_MODE == 4) ? EncoderMode::X4 : EncoderMode::X2;
    if (!encoder.begin(config.encAPin, config.encBPin, config.encFlip, mode)) {
        return false;
    }

    // Initialize PID speed controller with the shared, tuned gains
    pid.begin(PID_KP, PID_KI, PID_KD, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    return true;
}

void MDD32_Motor_Controller::setPIDGains(float kp, float ki, float kd) {
    pid.setGains(kp, ki, kd);
}

void MDD32_Motor_Controller::setRPM(float rpm) {
    pid.setSetpoint(rpm);
}

void MDD32_Motor_Controller::update(float dt) {
    // Get the current encoder count and the delta
    int64_t count = encoder.getCount();
    int64_t delta = count - _lastEncoderCount;
    _lastEncoderCount = count;

    // Compute the raw RPM, then smooth it with an exponential moving
    // average to suppress encoder quantization noise (worst at low speed)
    float revolutions = (float)delta / ENCODER_TICKS_PER_REV;
    float rawRPM = (revolutions / dt) * 60.0f;
    _currentRPM = RPM_FILTER_ALPHA * rawRPM + (1.0f - RPM_FILTER_ALPHA) * _currentRPM;

    // Run PID and apply to motor
    _currentPWM = pid.compute(_currentRPM, dt);
    motor.setPWM((int)_currentPWM);
}

void MDD32_Motor_Controller::stop() {
    // PID state is intentionally not reset here; reset() handles
    // that at the start of the next move
    setRPM(0);
    motor.stop();
    _currentRPM = 0.0f;
    _currentPWM = 0.0f;
}

void MDD32_Motor_Controller::reset() {
    pid.reset();
    encoder.clearCount();
    _lastEncoderCount = 0;
    _currentRPM = 0.0f;
    _currentPWM = 0.0f;
}
