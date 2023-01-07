// Author: Yalmar Cardenas
// Date: 2022-12-15
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

// import vector 3 message
#include <geometry_msgs/msg/vector3.h>
// import transformstamped message
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h> // import transform message
// import quaternion message
#include <std_msgs/msg/header.h>

#include <driver/gpio.h> // import gpio driver 
#include <driver/ledc.h>
#include "driver/i2c.h"
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
// include motor driver and encoder files 
#include "motor_driver.c"
#include "encoder.c"
#include "imu.c"
// variables to velocity and position
volatile float x = 0.0;
volatile float y = 0.0;
volatile float theta = 0.0;

//---------------------------------------------Macro functions---------------------------------------------
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
//---------------------------------------------Macro functions---------------------------------------------

//----------------------------------------------ROS------------------------------------------------
rcl_publisher_t publisher; // Publisher to publish odometry message from encoder
rcl_publisher_t publisher_twist; // Publisher to publish twist message
rcl_publisher_t publisher_imu; // Publisher to publish imu message
rcl_publisher_t publisher_mag; // Publisher to publish magnetometer message
geometry_msgs__msg__Twist msg; // Message type Twist for subscriber
geometry_msgs__msg__Twist twist_msg; // Message type Twist for publisher twist
nav_msgs__msg__Odometry odom_msg; // Message type Odometry for publisher encoder odometry
sensor_msgs__msg__Imu imu_msg; // Message type Imu for publish imu data
sensor_msgs__msg__MagneticField mag_msg; // Message type MagneticField for publish magnetometer data
//----------------------------------------------ROS------------------------------------------------

//----------------------------------------------Function forward declarations------------------------------------------------
// import SetupMotor function from motor_driver.c
void publish_wheel_odom(); // Publish odometry message from encoder
void publish_imu_raw(); // Publish imu data
void publish_mag_raw(); // Publish magnetometer data
void setupRos(); // Setup ROS
void cmd_vel_callback(const void *msgin); // Callback function for subscriber cmd_vel topic 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

//----------------------------------------------Function forward declarations------------------------------------------------
// Main
void appMain(void *arg) {
    SetupMotor(); // Setup pins for motor driver
    SetupEncoder(); // Setup pins for encoder
    SetupImu(); // Setup pins for imu
    setupRos(); // Setup ROS
}
// ROS setup
void setupRos() {
    // Micro ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ros_esp32_diffdrive", "", &support));

    // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    // create publisher to publish odometry from encoder
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/wheel/odometry"));
    // create publisher to publish imu data
    RCCHECK(rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw"));
    // create publisher to publish magnetometer data
    RCCHECK(rclc_publisher_init_default(
        &publisher_mag,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "/imu/mag"));
    // create timer,
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(FRAME_TIME),
        timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SLEEP_TIME));
        usleep(SLEEP_TIME * 1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node))
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node))
    RCCHECK(rcl_publisher_fini(&publisher_mag, &node))
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}
// We don't really need the callback, because msg is set anyway
void cmd_vel_callback(const void *msgin) {

    //const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

    //printf("Message received: %f %f\n", msg->linear.x, msg->angular.z);

}
// Each frame, check msg data and set PWM channels accordingly
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN)); // Toggle LED to show that the program is running properly

    if (timer == NULL) {
        return;
    }
    // Use linear.x for forward value and angular.z for rotation from cmd_vel topic
    float linear = constrain(msg.linear.x, -1, 1);
    float angular = constrain(msg.angular.z, -1, 1);
    // SetMotorSpeed Function
    SetMotorSpeed(linear, angular);
    // Get encoder data from encoder that return linear velocity and angular velocity array float[2]
    publish_wheel_odom();
    // call normalize function
    normalize();
    // publish imu data
    publish_imu_raw();
    // publish magnetometer data
    publish_mag_raw();
    }
    
// Publish odometry from encoder
void publish_wheel_odom()
{   
    
    // calculate odometry using trapezoidal integration method 
    x = x + linear_velocity_enc*cos(theta)*FRAME_TIME/1000; // calculate x position in m
    y = y + linear_velocity_enc*sin(theta)*FRAME_TIME/1000; // calculate y position in m
    theta = theta + angular_velocity_enc*FRAME_TIME/1000; // calculate theta in rad
    // publish odometry data to ROS
    //get current time for header timestamp field in microROS message 
    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data = "base_link";
    odom_msg.header.stamp.sec = (uint32_t) (esp_timer_get_time() / 1000000);
    odom_msg.header.stamp.nanosec = (uint32_t) (esp_timer_get_time() % 1000000) * 1000;
    // set the position
    odom_msg.pose.pose.position.x = x; // x position
    odom_msg.pose.pose.position.y = y; // y position
    odom_msg.pose.pose.position.z = 0.0; // z position
    // set the orientation
    odom_msg.pose.pose.orientation.x = 0.0; // x orientation
    odom_msg.pose.pose.orientation.y = 0.0; // y orientation
    odom_msg.pose.pose.orientation.z = sin(theta/2); // z orientation
    odom_msg.pose.pose.orientation.w = cos(theta/2); // w orientation
    // set the linear velocity from cmd_vel topic
    odom_msg.twist.twist.linear.x =  linear_velocity_enc;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    // set the angular velocity
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity_enc;
    // publish odometry message
    RCSOFTCHECK(rcl_publish(&publisher, (const void*)&odom_msg, NULL));
        // get encoder data 
    GetEncoderData();
}
// Publish mpu data raw to ROS
void publish_imu_raw()
{
    // get current time for header timestamp field in microROS message 
    imu_msg.header.frame_id.data = "imu_link";
    imu_msg.header.stamp.sec = (uint32_t) (esp_timer_get_time() / 1000000);
    imu_msg.header.stamp.nanosec = (uint32_t) (esp_timer_get_time() % 1000000) * 1000;
    // set the angular velocity
    imu_msg.angular_velocity.x = normalized.gyroscope.x;
    imu_msg.angular_velocity.y = normalized.gyroscope.y;
    imu_msg.angular_velocity.z = normalized.gyroscope.z;
    // set the linear acceleration
    imu_msg.linear_acceleration.x = normalized.accelerometer.x;
    imu_msg.linear_acceleration.y = normalized.accelerometer.y;
    imu_msg.linear_acceleration.z = normalized.accelerometer.z;
    // publish imu message
    RCSOFTCHECK(rcl_publish(&publisher_imu, (const void*)&imu_msg, NULL));
}
// Publish magnetic field data raw 
void publish_mag_raw()
{
    // get current time for header timestamp field in microROS message 
    mag_msg.header.frame_id.data = "imu_link";
    mag_msg.header.stamp.sec = (uint32_t) (esp_timer_get_time() / 1000000);
    mag_msg.header.stamp.nanosec = (uint32_t) (esp_timer_get_time() % 1000000) * 1000;
    // set the magnetic field
    mag_msg.magnetic_field.x = normalized.magnetometer.x;
    mag_msg.magnetic_field.y = normalized.magnetometer.y;
    mag_msg.magnetic_field.z = normalized.magnetometer.z;
    // publish mag message
    RCSOFTCHECK(rcl_publish(&publisher_mag, (const void*)&mag_msg, NULL));
}


