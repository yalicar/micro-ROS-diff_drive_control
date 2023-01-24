// Author: Yalmar Cardenas
// Date: 2022-12-15

#pragma once

#include "motor_driver.h"
#include "encoder.h"

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h> 
#include <std_msgs/msg/header.h>

// define constants
#define FRAME_TIME 100 // 10 Hz
//int FRAME_TIME = 100; // 100ms
//---------------------------------------------Macro functions---------------------------------------------
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// define extern variables
extern encoder_position_t encoder_position;
extern encoder_velocity_t encoder_velocity;

rcl_publisher_t publisher; // Publisher to publish odometry message from encoder
rcl_publisher_t publisher_twist; // Publisher to publish twist message
rcl_publisher_t publisher_imu; // Publisher to publish imu message
rcl_publisher_t publisher_mag; // Publisher to publish magnetometer message
geometry_msgs__msg__Twist msg; // Message type Twist for subscriber
geometry_msgs__msg__Twist twist_msg; // Message type Twist for publisher twist
nav_msgs__msg__Odometry odom_msg; // Message type Odometry for publisher encoder odometry
sensor_msgs__msg__Imu imu_msg; // Message type Imu for publish imu data
sensor_msgs__msg__MagneticField mag_msg; // Message type MagneticField for publish magnetometer data

// import SetupMotor function from motor_driver.c
void setupRos(); // Setup ROS
void cmd_vel_callback(const void *msgin); // Callback function for subscriber cmd_vel topic 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time); // Callback function for timer
void PublishWheelOdom(); // Publish odometry message from encoder


