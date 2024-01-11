#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/multi_array_dimension.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Mobile_command.h"
#include "math.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher_callback, publisher_odom;
rcl_subscription_t subscriber_vx, subscriber_vy, subscriber_w;
std_msgs__msg__Float32MultiArray pub_msg_callback, pub_msg_odom;
std_msgs__msg__Float32 sup_msg_vx, sup_msg_vy, sup_msg_w;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

unsigned long prev_timestep_print;
unsigned long current_timestep_print;
unsigned long timestamp_print = 0;
int32_t timestep_print = 10000;
//Control loop Time (1000 Hz)
unsigned long prev_timestep;
unsigned long current_timestep;
unsigned long timestamp = 0;
int timestep = 1000;
//Control loop Time (1 Hz)
unsigned long prev_timestep_cmd;
unsigned long current_timestep_cmd;
unsigned long timestamp_cmd = 0;
int timestep_cmd = 5e6;

float vx, vy, vw = 0;

uint8_t flag = 0;
Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);

IMU_DATA imu_data;
ODOM_DATA odom_data;

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_odom, &pub_msg, NULL));
    // Allocate memory for Float32MultiArray
    // Free memory if not needed anymore
    // free(pub_msg.data.data);
  }
}


void subscription_callback_vx(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin; 
  pub_msg_callback.data.data[0] = msg -> data; 
  RCSOFTCHECK(rcl_publish(&publisher_callback, &pub_msg_callback, NULL));
}

void subscription_callback_vy(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin; 
  pub_msg_callback.data.data[1] = msg -> data; 
  RCSOFTCHECK(rcl_publish(&publisher_callback, &pub_msg_callback, NULL));
}

void subscription_callback_w(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin; 
  pub_msg_callback.data.data[2] = msg -> data; 
  RCSOFTCHECK(rcl_publish(&publisher_callback, &pub_msg_callback, NULL));
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_callback,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "response_mobile_topic"));
  
  RCCHECK(rclc_publisher_init_default(
    &publisher_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "odom_mobile_topic"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_vx,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Vx_topic"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_vy,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "Vy_topic"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_w,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "W_topic"));


  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_vx, &sup_msg_vx, &subscription_callback_vx, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_vy, &sup_msg_vy, &subscription_callback_vy, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_w, &sup_msg_w, &subscription_callback_w, ON_NEW_DATA));

  pub_msg_callback.data.capacity = 3; 
  pub_msg_callback.data.size = 3;
  pub_msg_callback.data.data = (float*)malloc(3 * sizeof(float));
  
  // Populate data
  pub_msg_callback.data.data[0] = 0.0f;
  pub_msg_callback.data.data[1] = 0.0f;
  pub_msg_callback.data.data[2] = 0.0f;

  pub_msg_odom.data.capacity = 6; 
  pub_msg_odom.data.size = 6;
  pub_msg_odom.data.data = (float*)malloc(6 * sizeof(float));

  pub_msg_odom.data.data[0] = 0.0f; 
  pub_msg_odom.data.data[1] = 0.0f; 
  pub_msg_odom.data.data[2] = 0.0f; 
  pub_msg_odom.data.data[3] = 0.0f; 
  pub_msg_odom.data.data[4] = 0.0f; 
  pub_msg_odom.data.data[5] = 0.0f; 
  pub_msg_odom.data.data[6] = 0.0f;

  Mobile.begin();

  delay(3000);
}

void loop() {
  // delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();

    imu_data = Mobile.getIMU();
    // Serial.print(odom_data.vx);
    // Serial.print(" ");
    // Serial.print(odom_data.vy);
    // Serial.print(" ");
    // Serial.print(odom_data.wz);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[2]);
    // Serial.print(" ");
    // Serial.println(Mobile.fb_qd[3]);
    pub_msg_odom.data.data[0] = odom_data.vx; 
    pub_msg_odom.data.data[1] = odom_data.vy; 
    pub_msg_odom.data.data[2] = odom_data.wz; 
    pub_msg_odom.data.data[3] = Mobile.fb_qd[0]; 
    pub_msg_odom.data.data[4] = Mobile.fb_qd[1]; 
    pub_msg_odom.data.data[5] = Mobile.fb_qd[2]; 
    pub_msg_odom.data.data[6] = Mobile.fb_qd[3]; 
  }
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    Mobile.control(pub_msg_callback.data.data[0] , pub_msg_callback.data.data[1] , pub_msg_callback.data.data[2] );

    odom_data = Mobile.getODOM();
  }

}