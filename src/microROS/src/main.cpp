#include <Arduino.h>
#include "BNO008X.h"
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
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
void setupMotors();
// === Encoder Pins ===
const int ENCODER_RIGHT_A = 23, ENCODER_RIGHT_B = 22;
const int ENCODER_LEFT_A = 18, ENCODER_LEFT_B = 19;
void setupEncoders();

// == Encoder Pulses ===
volatile long rightPulses = 0;
volatile long leftPulses = 0;

// === PWM Configuration ===
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8;
const int PWM_CHANNEL_RIGHT = 0, PWM_CHANNEL_LEFT = 1;

// === myIMU ===
const int SDA_PIN = 21, SCL_PIN = 15;

// === Object initialization ===
BNO080 myIMU;
ESP32Encoder encoderLeft, encoderRight;

float gyroZ_bias = 0.0, robot_yaw = 0.0;


// === Robot Kinematics ===
const float GEAR_RATIO = 46.8f;
const int PULSES_PER_REV = 44;
const float WHEEL_BASE = 0.211f;     
const float WHEEL_RADIUS = 0.0325f;    

// === Odometry ===
float x_pos = 0.0, y_pos = 0.0, theta_pos = 0.0;
float target_linear_x = 0.0, target_angular_z = 0.0;
unsigned long lastOdomTime = 0, lastIMUTime = 0;

void setupIMU();
void publishOdometry();
void updateIMU();


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

void setupIMU(){
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!myIMU.begin(0X4A, Wire)) { 
        Serial.println("Failed to find IMU chip");
        while (1) { delay(10); }
    }

    // Enable gyroscope reporting (example: every 10ms)
    myIMU.enableGyro(10);

    // Wait for sensor to be ready (optional, for stability)
    delay(100);

    float total_error = 0;
    int samples = 2000;
    int collected = 0;
    while (collected < samples) {
        if (myIMU.dataAvailable()) {
            total_error += myIMU.getGyroZ(); // Get Z-axis gyro in rad/s
            collected++;
        }
        delay(1);
    }
    gyroZ_bias = total_error / samples;
    Serial.print("IMU Gyro Z bias: ");
    Serial.println(gyroZ_bias, 6);
}

void testIMU() {
  if (myIMU.dataAvailable()) {
    float correctedGyroZ = myIMU.getGyroZ() - gyroZ_bias;
    Serial.print("IMU Gyro Z (rad/s): "); Serial.print(correctedGyroZ, 5);
    Serial.print(" | Accel X: "); Serial.print(myIMU.getAccelX());
    Serial.print(" Y: "); Serial.print(myIMU.getAccelY());
    Serial.print(" Z: "); Serial.println(myIMU.getAccelZ());
  }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    setupIMU();
}

void loop() {
    testIMU();
    delay(200);
}








