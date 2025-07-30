#include <Arduino.h>

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Motor Control Pin Setup
// Left Motor
#define MOTOR_LEFT_IN1     26
#define MOTOR_LEFT_IN2     33
#define MOTOR_LEFT_EN      27

// Right Motor
#define MOTOR_RIGHT_IN1    25
#define MOTOR_RIGHT_IN2    32
#define MOTOR_RIGHT_EN     14

// Encoder Pin Setup
// Right Encoder
#define ENCODER_RIGHT_A    23 
#define ENCODER_RIGHT_B    22 

// Left Encoder
#define ENCODER_LEFT_A     18 
#define ENCODER_LEFT_B     19 

// PWM Configuration
#define PWM_FREQ           5000     // PWM frequency in Hz
#define PWM_RESOLUTION     8        // 8-bit resolution
#define PWM_CHANNEL_RIGHT  0        // PWM channel for right motor
#define PWM_CHANNEL_LEFT   1        // PWM channel for left motor

// Robot Kinematics
#define GEAR_RATIO         46.8
#define PULSES_PER_REV     44
#define WHEEL_BASE         0.211    // Distance between two wheels (meters)
#define WHEEL_RADIUS       0.0325   // Radius of each wheel (meters)

// Encoder Pulse Counters
extern volatile long rightPulses;
extern volatile long leftPulses;
extern const int pulsesPerRevolution;

// Function Declarations
void motorSetUp();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();

void IRAM_ATTR handleRightEncoderB();
void IRAM_ATTR handleLeftEncoderB(); 
void IRAM_ATTR handleRightEncoderA();
void IRAM_ATTR handleLeftEncoderA();

float pulsesToRPM(int pulses);
int DistancetoPulse(float distance);
void setMotorSpeed(float left_wheel_velocity, float right_wheel_velocity);

#endif  // MOTOR_CONTROL_H