// Motor driver and encoder code for ESP32
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
// include motor driver and encoder files
#include "ros.h"
#include "motor_driver.h"
#include "encoder.h"
//#include "encoder.h"
//#include "imu.c"

// Global variables
motor_setup_t motor_setup = {
    .LED_BUILTIN = 33,
    .PIN_LEFT_FORWARD = 12,
    .PIN_LEFT_BACKWARD = 13,
    .PIN_RIGHT_FORWARD = 15,
    .PIN_RIGHT_BACKWARD = 14,
    .PWM_LEFT_FORWARD = 2,
    .PWM_LEFT_BACKWARD = 3,
    .PWM_RIGHT_FORWARD = 4,
    .PWM_RIGHT_BACKWARD = 5,
    .PWM_FREQUENCY = 50,
    .PWM_RESOLUTION = LEDC_TIMER_12_BIT,
    .PWM_TIMER = LEDC_TIMER_1,
    .PWM_MODE = LEDC_HIGH_SPEED_MODE,
    .PWM_MOTOR_MIN = 750,
    .PWM_MOTOR_MAX = 4095
};

encoder_setup_t encoder_setup = {
    .PIN_A = 34,
    .PIN_B = 35,
    .RESOLUTION = 12,
    .PULSES_PER_REV = 40,
    .FRAME_TIME_MS = FRAME_TIME,
    .WHEEL_DIAMETER = 0.065,
    .WHEEL_BASE = 0.13
};
//----------------------------------------------Function declarations------------------------------------------------
// Main
void appMain(void *arg) {
    // Initialize motor driver
    InitMotorDriver(motor_setup);
    // Initialize encoder
    InitEncoder(encoder_setup);
    // Setup ROS
    setupRos(); // Setup ROS
}