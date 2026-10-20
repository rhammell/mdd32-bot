/*
 * robot_definitions.h - Robot tuning and behavioral constants
 *
 * Constants defined in this file describe how the robot behaves
 * rather than how it is wired: physical geometry, control gains,
 * calibration factors, and default settings. These values are
 * intrinsic to the robot and its algorithms, and are shared by
 * the subsystems that need them.
 */

#pragma once

#include <math.h>

// Battery voltage divider ratio (Vbat = Vadc * ratio)
#define VBAT_DIV_RATIO      2.0f

// Wheel geometry
#define WHEEL_DIAMETER_CM       4.05
#define WHEEL_CIRCUMFERENCE_CM  (M_PI * WHEEL_DIAMETER_CM)
#define TICKS_PER_CM            (ENCODER_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_CM)

// Drive speed limits
#define MAX_SPEED_CM_S          12.0f

// Wheel-sync correction gain (tuned with examples/Tuning/3_WheelSyncTuning). Keeps the
// left/right wheels balanced: matched counts when driving straight, and
// equal-and-opposite counts when spinning in place.
#define WHEEL_SYNC_KP           0.0f

// Acceleration and deceleration (tuned with examples/Tuning/4_DistanceTuning)
// MAX_ACCEL_RPM_S limits how fast the commanded RPM slews toward the target.
// Set to 0 to disable ramping (commanded RPM jumps straight to target).
// 250 RPM/s == Alvik's 5 RPM per 20ms control tick.
#define MAX_ACCEL_RPM_S         57.0f
#define DECEL_DISTANCE_CM       3.0f
#define MIN_SPEED_CM_S          0.5f

// Motor gear ratio (150:1 gearmotors)
#define MOTOR_GEAR_RATIO 150.58

// Motor encoder settings
// ENCODER_MODE: 4 for X4 quadrature (both edges, both channels)
//               2 for X2 quadrature (both edges, channel A only)
#define ENCODER_MODE 4
#define ENCODER_PPR 3             // Pulses per revolution (pole pairs on the magnetic disc)
#define ENCODER_TICKS_PER_REV (ENCODER_PPR * ENCODER_MODE * MOTOR_GEAR_RATIO)

// PID speed controller gains (tuned with examples/Tuning/2_PIDTuning)
// With feedforward enabled, the feedforward term supplies the bulk of the
// holding PWM, so the integral only has to trim the residual - keep KI low.
#define PID_KP          6.0f
#define PID_KI          4.0f
#define PID_KD          0.0f  

// PID output limits and loop rate
#define PID_OUTPUT_MIN  -255.0f
#define PID_OUTPUT_MAX   255.0f
#define DRIVE_UPDATE_INTERVAL_US  20000  // 20ms = 50Hz

// Velocity feedforward (tuned with examples/Tuning/1_FeedforwardTuning)
// Open-loop PWM required to hold a given wheel speed, measured from the
// 1_FeedforwardTuning sweep data. Applied instantly from the target RPM so the
// wheel reaches speed quickly instead of waiting for the integral to wind up;
// the PID then only trims the residual. The curve is convex (PWM rises steeply
// near top speed), so it is stored as a lookup table and linearly interpolated.
// Set PID_FEEDFORWARD_ENABLE to 0 for pure PID (no feedforward).
#define PID_FEEDFORWARD_ENABLE 1

// |RPM| breakpoints (ascending) and the PWM needed to sustain each. Values
// between points are interpolated; magnitudes beyond the ends are clamped. The
// target's sign is applied to the result, and a target of 0 yields 0.
constexpr float PID_FF_RPM[]   = {0.0f, 15.0f, 29.0f, 38.0f, 47.0f, 57.0f};
constexpr float PID_FF_PWM[]   = {0.0f, 42.0f, 55.0f, 66.0f, 86.0f, 120.0f};
constexpr int   PID_FF_POINTS  = 6;

// Measured-RPM smoothing (exponential moving average)
// Higher alpha = less smoothing/lag; lower alpha = more smoothing/lag.
//Changing alpha alters the loop dynamics and requires re-running PID tuning.
#define RPM_FILTER_ALPHA  0.33f  
