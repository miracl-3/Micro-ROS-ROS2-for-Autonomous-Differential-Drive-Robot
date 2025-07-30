#include <Arduino.h>
#include "BNO008X.h"
#include <ESP32Encoder.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>


// ==== Hardware Setup ====

// === Motor Pins ===
const int MOTOR_LEFT_IN1 = 26, MOTOR_LEFT_IN2 = 33, MOTOR_LEFT_EN = 27;
const int MOTOR_RIGHT_IN1 = 25, MOTOR_RIGHT_IN2 = 32, MOTOR_RIGHT_EN = 14;

// === Encoder Pins ===
const int ENCODER_RIGHT_A = 23, ENCODER_RIGHT_B = 22;
const int ENCODER_LEFT_A = 18, ENCODER_LEFT_B = 19;

// === PWM Configuration ===
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8;
const int PWM_CHANNEL_RIGHT = 0, PWM_CHANNEL_LEFT = 1;
// void setupMotors();

// // === Encoder Setup ===
// void setupEncoders();
// void IRAM_ATTR handleRightEncoderB();
// void IRAM_ATTR handleLeftEncoderB(); 
// void IRAM_ATTR handleRightEncoderA();
// void IRAM_ATTR handleLeftEncoderA();

// // == Encoder Pulses ===
// volatile long rightPulses = 0;
// volatile long leftPulses = 0;

// // === Robot Kinematics ===
// const float GEAR_RATIO       = 46.8f;
// const int PULSES_PER_REV     = 44;
// const float WHEEL_BASE       = 0.211f;     // meters
// const float WHEEL_RADIUS     = 0.0325f;    // meters

// // === Odometry ===
// float x_pos = 0.0, y_pos = 0.0, theta_pos = 0.0;
// float target_linear_x = 0.0, target_angular_z = 0.0;
// unsigned long lastOdomTime = 0, lastIMUTime = 0;

// void publishOdometry();
// void updateIMU();

// // === Object initialization ===
// BNO080 imu;
ESP32Encoder encoderLeft, encoderRight;


void setupMotors(){
    pinMode(MOTOR_LEFT_IN1, OUTPUT); pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT); pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_RIGHT_EN, PWM_CHANNEL_RIGHT);
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_LEFT_EN, PWM_CHANNEL_LEFT);
}

void setupEncoders(){
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachFullQuad(ENCODER_LEFT_A, ENCODER_LEFT_B);
    encoderRight.attachFullQuad(ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    encoderLeft.clearCount();
    encoderRight.clearCount();
}

void setup() {
    
}

void loop() {
    // Read and print encoder counts
    
}









