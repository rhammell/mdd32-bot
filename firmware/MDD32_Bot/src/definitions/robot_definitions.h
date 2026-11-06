/*
 * robot_definitions.h - Robot tuning and behavioral constants
 *
 * Constants defined in this file describe how the robot behaves
 * rather than how it is wired: physical geometry, control gains,
 * calibration factors, and default settings. These values are
 * intrinsic to the robot and its algorithms, and are shared by
 * the subsystems that need them.
 *
 * The tuning constants below serve as the library-wide defaults loaded by
 * bot.begin(). They can be overridden at runtime in any sketch via
 * bot.drive.setPIDGains(), setWheelSyncKP(), and setMaxAccel(), or updated
 * here to change the defaults for all sketches.
 */

#pragma once

#include <math.h>

// Battery voltage divider ratio (Vbat = Vadc * ratio)
#define VBAT_DIV_RATIO      2.0f

// Wheel geometry
#define WHEEL_DIAMETER_CM       3.4
#define WHEEL_CIRCUMFERENCE_CM  (M_PI * WHEEL_DIAMETER_CM)
#define TICKS_PER_CM            (ENCODER_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_CM)

// Drive speed limits
#define MAX_SPEED_CM_S          10.0f

// Wheel-sync correction gain (tuned with examples/Tuning/2_WheelSyncTuning). Keeps the
// left/right wheels balanced: matched counts when driving straight, and
// equal-and-opposite counts when spinning in place.
#define WHEEL_SYNC_KP           0.0f

// Acceleration and deceleration (tuned with examples/Tuning/3_DistanceTuning)
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

// PID speed controller gains (tuned with examples/Tuning/1_PIDTuning)
// The integral supplies the holding PWM each wheel needs, so it adapts to
// differences between motors without any per-wheel calibration.
#define PID_KP          4.0f
#define PID_KI          22.0f
#define PID_KD          0.0f  

// PID output limits and loop rate
#define PID_OUTPUT_MIN  -255.0f
#define PID_OUTPUT_MAX   255.0f
#define DRIVE_UPDATE_INTERVAL_US  20000  // 20ms = 50Hz

// Measured-RPM smoothing (exponential moving average)
// Higher alpha = less smoothing/lag; lower alpha = more smoothing/lag.
//Changing alpha alters the loop dynamics and requires re-running PID tuning.
#define RPM_FILTER_ALPHA  0.33f  
