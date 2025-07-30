#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
std_msgs__msg__String msg;
char message_data[50];

void setup() {
  set_microros_serial_transports(Serial);
  Serial.begin(115200);
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_micro_ros_node", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "hello"
  );

  msg.data.data = message_data;
  msg.data.capacity = sizeof(message_data);
  msg.data.size = 0;
}

void loop() {
  const char* message_text = "Hello micro-ROS from ESP32!";
  msg.data.data = (char *)message_text;
  msg.data.size = strlen(message_text);
  msg.data.capacity = msg.data.size + 1;

  rcl_publish(&publisher, &msg, NULL);

  delay(1000);  // Publish every 1 second
}
