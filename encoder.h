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

// Encoder gpio pins
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35
#define ENCODER_resolution 12
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
// number of pulses per revolution
#define ENCODER_PPR 40
// wheel radius
#define WHEEL_RADIUS 0.0325 // m
// wheel base
#define WHEEL_BASE 0.13 // m

// Struct for encoder speed
typedef struct {
  float left;
  float right;
} encoder_speed;
// struct for encoder count
typedef struct {
  int32_t left;
  int32_t right;
} encoder_count;
// struct for linear and angular velocity
typedef struct {
  float linear;
  float angular;
} velocity_enc;
// struct for direction of rotation
typedef struct {
  int8_t left;
  int8_t right;
} direction;
// struct for encoder state 
typedef struct {
  int8_t left;
  int8_t right;
  int8_t left_last;
  int8_t right_last;
} encoder_state;

// encoder interrupt service routine for left encoder
void IRAM_ATTR encoder_left_isr_handler(void *arg);
// encoder interrupt service routine for right encoder
void IRAM_ATTR encoder_right_isr_handler(void *arg);
// encoder initialization
void EncoderInit(void);
// function to get linear and angular velocity
void GetVelocity(void);

// 