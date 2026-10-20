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
    _ffEnabled = (PID_FEEDFORWARD_ENABLE != 0);

    return true;
}

void MDD32_Motor_Controller::setPIDGains(float kp, float ki, float kd) {
    pid.setGains(kp, ki, kd);
}

void MDD32_Motor_Controller::setFeedforwardEnabled(bool enabled) {
    _ffEnabled = enabled;
}

void MDD32_Motor_Controller::setRPM(float rpm) {
    // Store the target for the feedforward and set the PID setpoint
    _targetRPM = rpm;
    pid.setSetpoint(rpm);
}

float MDD32_Motor_Controller::feedforward(float rpm) const {
    if (!_ffEnabled || PID_FF_POINTS <= 0 || rpm == 0.0f) {
        return 0.0f;
    }

    // Interpolate the holding-PWM lookup table on |rpm|, clamping at the ends
    float mag = fabsf(rpm);
    float pwm;
    if (mag <= PID_FF_RPM[0]) {
        pwm = PID_FF_PWM[0];
    } else if (mag >= PID_FF_RPM[PID_FF_POINTS - 1]) {
        pwm = PID_FF_PWM[PID_FF_POINTS - 1];
    } else {
        pwm = PID_FF_PWM[PID_FF_POINTS - 1];
        for (int i = 1; i < PID_FF_POINTS; i++) {
            if (mag <= PID_FF_RPM[i]) {
                float span = PID_FF_RPM[i] - PID_FF_RPM[i - 1];
                float t = (span > 0.0f) ? (mag - PID_FF_RPM[i - 1]) / span : 0.0f;
                pwm = PID_FF_PWM[i - 1] + t * (PID_FF_PWM[i] - PID_FF_PWM[i - 1]);
                break;
            }
        }
    }
    // Apply the target's direction (0 stays 0)
    return (rpm < 0.0f) ? -pwm : pwm;
}

void MDD32_Motor_Controller::measureRPM(float dt) {
    // Get the current encoder count and the delta
    int64_t count = encoder.getCount();
    int64_t delta = count - _lastEncoderCount;
    _lastEncoderCount = count;

    // Compute the raw RPM, then smooth it with an exponential moving
    // average to suppress encoder quantization noise (worst at low speed)
    float revolutions = (float)delta / ENCODER_TICKS_PER_REV;
    float rawRPM = (revolutions / dt) * 60.0f;
    _currentRPM = RPM_FILTER_ALPHA * rawRPM + (1.0f - RPM_FILTER_ALPHA) * _currentRPM;
}

void MDD32_Motor_Controller::update(float dt) {
    // Refresh the filtered RPM measurement
    measureRPM(dt);

    // Run PID (with velocity feedforward from the target) and apply to motor
    _currentPWM = pid.compute(_currentRPM, dt, feedforward(_targetRPM));
    motor.setPWM((int)_currentPWM);
}

void MDD32_Motor_Controller::updateOpenLoop(float dt, int pwm) {
    // Refresh the filtered RPM measurement, then drive the raw PWM directly.
    // The PID and feedforward are bypassed so the logged RPM reflects only the
    // motor's open-loop response to this PWM. Characterization use only.
    measureRPM(dt);
    _currentPWM = (float)pwm;
    motor.setPWM(pwm);
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
