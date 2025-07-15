#include <Arduino.h>

#ifndef ODOMETRY_H
#define ODOMETRY_H

// Robot kinematics constants (matching motor_control.h)
#define WHEEL_RADIUS       0.0325   // Radius of each wheel (meters)
#define WHEEL_BASE         0.211    // Distance between wheels (meters)
#define GEAR_RATIO         46.8     // Motor gear ratio
#define PULSES_PER_REV     44       // Encoder pulses per revolution

// Structure containing wheel encoder odometry data
struct Odometry {
  float x;               // X position (meters)
  float y;               // Y position (meters)
  float theta;           // Orientation (radians)
  float linear_velocity;  // Linear velocity (m/s)
  float angular_velocity; // Angular velocity (rad/s)
};

Odometry updateOdometry(float delta_t);
void resetOdometry();

#endif // ODOMETRY_H