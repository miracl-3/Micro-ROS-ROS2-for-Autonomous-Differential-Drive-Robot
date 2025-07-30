#include <Arduino.h>
#include "motor_control.h"
#include <math.h>

volatile long rightPulses = 0;
volatile long leftPulses = 0;

const int ticksperRevolution = GEAR_RATIO * PULSES_PER_REV;  // 46.8 × 44

// PID control values (not currently used, reserved for future)
float kp = 10;
float ki = 5;
float kd = 1;
float proportional_error;
float integral_error; 
float derivative_error;
float total_error;
int base_pwm = 190;

int left_pwm, right_pwm;
const float wheel_circumference = 2 * PI * WHEEL_RADIUS;

void motorSetUp() {
    // Motor control pins
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    // Encoder pins
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

    // PWM setup
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_RIGHT_EN, PWM_CHANNEL_RIGHT);
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_EN, PWM_CHANNEL_LEFT);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), handleRightEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), handleLeftEncoderB, CHANGE);
}

void moveForward(int speed) { 
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void moveBackward(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnLeft(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnRight(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void stop() {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, 0);

    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

// Encoder Interrupts
void IRAM_ATTR handleRightEncoderA() {
    bool A = digitalRead(ENCODER_RIGHT_A);
    bool B = digitalRead(ENCODER_RIGHT_B);
    if (A == B) rightPulses++;
    else rightPulses--;
}

void IRAM_ATTR handleRightEncoderB() {
    bool A = digitalRead(ENCODER_RIGHT_A);
    bool B = digitalRead(ENCODER_RIGHT_B);
    if (A != B) rightPulses++;
    else rightPulses--;
}

void IRAM_ATTR handleLeftEncoderA() {
    bool A = digitalRead(ENCODER_LEFT_A);
    bool B = digitalRead(ENCODER_LEFT_B);
    if (A == B) leftPulses--;
    else leftPulses++;
}

void IRAM_ATTR handleLeftEncoderB() {
    bool A = digitalRead(ENCODER_LEFT_A);
    bool B = digitalRead(ENCODER_LEFT_B);
    if (A != B) leftPulses--;
    else leftPulses++;
}

// Utility Functions
float pulsesToRPM(int pulsesCount) {
    float revolutions = (float)pulsesCount / ticksperRevolution;
    float rpm = revolutions * 60.0;
    return rpm;
}

int DistancetoPulse(float distance) {
    return (int)(GEAR_RATIO * PULSES_PER_REV * distance / wheel_circumference);
}

void setMotorSpeed(float left_wheel_velocity, float right_wheel_velocity) {
    // Convert velocity (m/s) to PWM (0–255)
    // Wheel velocity (m/s) -> RPM -> PWM
    const float max_rpm = 100.0; // Adjust based on motor specs (e.g., max RPM at 255 PWM)
    const float max_velocity = (max_rpm / 60.0) * wheel_circumference; // Max velocity in m/s
    const int max_pwm = 255; // 8-bit PWM resolution

    // Calculate PWM for each wheel
    int left_pwm = (int)(fabs(left_wheel_velocity) / max_velocity * max_pwm);
    int right_pwm = (int)(fabs(right_wheel_velocity) / max_velocity * max_pwm);

    // Apply balance factor for calibration
    left_pwm = constrain(left_pwm, 0, max_pwm);
    right_pwm = constrain(right_pwm, 0, max_pwm);

    // Set motor direction and speed
    if (left_wheel_velocity >= 0) {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
    }
    ledcWrite(PWM_CHANNEL_LEFT, left_pwm);

    if (right_wheel_velocity >= 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    }
    ledcWrite(PWM_CHANNEL_RIGHT, right_pwm);
}