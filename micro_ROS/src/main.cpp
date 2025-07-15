#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <odometry.h>
#include <motor_control.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("ROS Error: "); Serial.println(temp_rc); while(1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("ROS Soft Error: "); Serial.println(temp_rc);}}

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t cmd_vel_subscriber;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
BNO080 myIMU;

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_vel = msg->linear.x;  // m/s
  float angular_vel = msg->angular.z; // rad/s
  float v_left = linear_vel - (angular_vel * WHEEL_BASE / 2.0);
  float v_right = linear_vel + (angular_vel * WHEEL_BASE / 2.0);
  setMotorSpeed(v_left, v_right);
}

void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  Odometry pose = updateOdometry(0.1);
  int64_t millis = rmw_uros_epoch_millis();
  odom_msg.header.stamp.sec = millis / 1000;
  odom_msg.header.stamp.nanosec = (millis % 1000) * 1000000;
  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  odom_msg.child_frame_id.data = (char *)"base_link";
  odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
  odom_msg.pose.pose.position.x = pose.x;
  odom_msg.pose.pose.position.y = pose.y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(pose.theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(pose.theta / 2.0);
  odom_msg.twist.twist.linear.x = pose.linear_velocity;
  odom_msg.twist.twist.angular.z = pose.angular_velocity;
  float encoder_resolution = (2 * PI * WHEEL_RADIUS) / (GEAR_RATIO * PULSES_PER_REV);
  odom_msg.pose.covariance[0] = encoder_resolution * encoder_resolution;
  odom_msg.pose.covariance[7] = encoder_resolution * encoder_resolution;
  odom_msg.pose.covariance[35] = 0.01;
  odom_msg.twist.covariance[0] = 0.01;
  odom_msg.twist.covariance[35] = 0.01;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (myIMU.getSensorEvent()) {
    uint32_t micros = myIMU.getTimeStamp(); // Use IMU timestamp
    imu_msg.header.stamp.sec = micros / 1000000;
    imu_msg.header.stamp.nanosec = (micros % 1000000) * 1000;
    imu_msg.header.frame_id.data = (char *)"imu_link";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
    imu_msg.orientation.x = myIMU.getQuatI();
    imu_msg.orientation.y = myIMU.getQuatJ();
    imu_msg.orientation.z = myIMU.getQuatK();
    imu_msg.orientation.w = myIMU.getQuatReal();
    imu_msg.orientation_covariance[0] = myIMU.getQuatRadianAccuracy();
    imu_msg.orientation_covariance[4] = myIMU.getQuatRadianAccuracy();
    imu_msg.orientation_covariance[8] = myIMU.getQuatRadianAccuracy();
    float lin_x, lin_y, lin_z;
    uint8_t lin_accuracy;
    myIMU.getLinAccel(lin_x, lin_y, lin_z, lin_accuracy);
    imu_msg.linear_acceleration.x = lin_x;
    imu_msg.linear_acceleration.y = lin_y;
    imu_msg.linear_acceleration.z = lin_z;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  set_microros_serial_transports(Serial2);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
  RCCHECK(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); // 1 subscription + 2 timers
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(100), odom_timer_callback));
  RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(50), imu_timer_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

  Wire.begin(21, 15);
  if (!myIMU.begin(0x4A, Wire)) {
    Serial.println("IMU init failed.");
    while (1);
  }
  myIMU.enableRotationVector(50); // 20 Hz
  myIMU.enableLinearAccelerometer(50); // 20 Hz
  myIMU.calibrateAll();
  while (!myIMU.calibrationComplete()) {
    delay(10);
  }
  myIMU.saveCalibration();

  motorSetUp();
  resetOdometry();

  nav_msgs__msg__Odometry__init(&odom_msg);
  sensor_msgs__msg__Imu__init(&imu_msg);
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}