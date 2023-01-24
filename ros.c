#include "ros.h"
#include "motor_driver.h"
#include "encoder.h"

// Author: Yalmar Cardenas
// Date: 2022-12-15


// We don't really need the callback, because msg is set anyway
void cmd_vel_callback(const void *msgin) {
}

// Each frame, check msg data and set PWM channels accordingly
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    if (timer == NULL) {
        return;
    }
    // Use linear.x for forward value and angular.z for rotation from cmd_vel topic
    float linear = constrain(msg.linear.x, -1, 1);
    float angular = constrain(msg.angular.z, -1, 1);

    // Function to set motor speed and direction
    SetMotorSpeed(linear, angular);
    PublishWheelOdom();
    
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

    /*
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
    */

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
    /*
    RCCHECK(rcl_publisher_fini(&publisher_imu, &node))
    RCCHECK(rcl_publisher_fini(&publisher_mag, &node))
    RCCHECK(rcl_node_fini(&node));
    */

    vTaskDelete(NULL);
}

// wheel odometry publisher
void PublishWheelOdom()
{   
    // get encoder data
    GetEncoder();
    // get current time for header timestamp field in microROS message
    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data = "base_link";
    odom_msg.header.stamp.sec = (uint32_t) (esp_timer_get_time() / 1000000);
    odom_msg.header.stamp.nanosec = (uint32_t) (esp_timer_get_time() % 1000000) * 1000;
    // set the position
    odom_msg.pose.pose.position.x = encoder_position.x; // x position
    odom_msg.pose.pose.position.y = encoder_position.y; // y position
    odom_msg.pose.pose.position.z = 0.0; // z position
    // set the orientation
    odom_msg.pose.pose.orientation.x = 0.0; // x orientation
    odom_msg.pose.pose.orientation.y = 0.0; // y orientation
    odom_msg.pose.pose.orientation.z = 0.0; // z orientation
    odom_msg.pose.pose.orientation.w = 1.0; // w orientation
    // set the linear velocity from cmd_vel topic
    odom_msg.twist.twist.linear.x = encoder_velocity.linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    // set the angular velocity
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = encoder_velocity.angular;
    // publish odometry message
    RCSOFTCHECK(rcl_publish(&publisher, (const void*)&odom_msg, NULL));
}

/*
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
*/