#include <Arduino.h>
#include <odometry.h>
#include <motor_control.h>

static Odometry pose = {0.0, 0.0, 0.0, 0.0, 0.0};
static long prevLeftTicks = 0;
static long prevRightTicks = 0;
const float DISTANCE_PER_TICK = (2 * PI * WHEEL_RADIUS) / (GEAR_RATIO * PULSES_PER_REV);

Odometry updateOdometry(float delta_t) {
    long leftTicks = leftPulses;
    long rightTicks = rightPulses;

    long deltaLeft = leftTicks - prevLeftTicks;
    long deltaRight = rightTicks - prevRightTicks;

    float dLeft = deltaLeft * DISTANCE_PER_TICK;
    float dRight = deltaRight * DISTANCE_PER_TICK;

    float dCenter = (dLeft + dRight) / 2.0;
    float dTheta = (dRight - dLeft) / WHEEL_BASE;

    // Update pose
    pose.x += dCenter * cos(pose.theta + dTheta / 2.0);
    pose.y += dCenter * sin(pose.theta + dTheta / 2.0);
    pose.theta += dTheta;
    pose.linear_velocity = dCenter / delta_t;
    pose.angular_velocity = dTheta / delta_t;

    // Normalize angle
    if (pose.theta > M_PI) {
        pose.theta -= 2 * M_PI;
    } else if (pose.theta < -M_PI) {
        pose.theta += 2 * M_PI;
    }

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;
    return pose;
}

void resetOdometry() {
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    pose.linear_velocity = 0.0;
    pose.angular_velocity = 0.0;
    prevLeftTicks = leftPulses;
    prevRightTicks = rightPulses;
}