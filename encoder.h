
#ifndef ENCODER_H // include guard to prevent multiple definitions of the same thing (in this case, the class) 
#define ENCODER_H // if the macro ENCODER_H is not defined, define it now

// include the Arduino standard library
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "driver/i2c.h"
#ifdef ESP_PLATFORM // ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


// Encoder gpio pins 34
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35
#define ENCODER_resolution 12
volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;
// Pi Constants
#define PI 3.14159265358979323846
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
// Motor gpio pins
// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5
// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// encoder state
volatile int8_t encoder_left_state = 0;
volatile int8_t encoder_right_state = 0;
volatile int8_t encoder_left_state_last = 0;
volatile int8_t encoder_right_state_last = 0;
// timer 
volatile uint32_t encoder_left_time = 0;
volatile uint32_t encoder_left_time_last = 0;
volatile uint32_t encoder_left_dt = 0;
volatile uint32_t encoder_right_time = 0;
volatile uint32_t encoder_right_time_last = 0;
volatile uint32_t encoder_right_dt = 0;
// direction of rotation
volatile int8_t left_direction = 0;
volatile int8_t right_direction = 0;
// number of pulses per revolution
#define ENCODER_PPR 40
// encoder speed
volatile float encoder_left_speed = 0;
volatile float encoder_right_speed = 0;
// wheel radius
#define WHEEL_RADIUS 0.0325 // m
// wheel base
#define WHEEL_BASE 0.13 // m
// linear and angular velocity
volatile float linear_velocity_enc = 0.0;
volatile float angular_velocity_enc = 0.0;