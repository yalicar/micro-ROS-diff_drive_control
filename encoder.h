#pragma once

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#include "driver/i2c.h"
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

typedef struct {
  int PIN_A;
  int PIN_B;
  int RESOLUTION;
  int PULSES_PER_REV;
  float WHEEL_DIAMETER;
  float WHEEL_BASE;
} encoder_setup_t;

// typedefs struct for encoder state
typedef struct {
  int left;
  int left_last;
  int right;
  int right_last;
} encoder_state_t;

// typedefs struct for encoder count
typedef struct {
  int left;
  int right;
} encoder_count_t;

// typedefs struct for encoder direction
typedef struct {
  int left;
  int right;
} encoder_direction_t;

// typedefs struct speed
typedef struct {
  float left;
  float right;
} encoder_speed_t;

// typedefs struct for encoder linear velocity (m/s) and angular velocity (rad/s)
typedef struct {
  float linear;
  float angular;
} encoder_velocity_t;

// typedefs struct for encoder position (m) with x, y, theta (rad) and last position
typedef struct {
  float x;
  float y;
  float theta;
  float x_last;
  float y_last;
  float theta_last;
} encoder_position_t;

// define function prototypes with pointer arguments
void InitEncoder(encoder_setup_t encoder_setup);
void IRAM_ATTR encoder_left_isr_handler(void* arg);
void IRAM_ATTR encoder_right_isr_handler(void* arg);
void encoder_count_reset(encoder_count_t* encoder_count);
void encoder_direction_();
/*
void encoder_speed_(encoder_count_t *encoder_count, encoder_direction_t *encoder_direction, encoder_speed_t *encoder_speed, encoder_setup_t *encoder_setup);
void encoder_velocity_(encoder_speed_t *encoder_speed, encoder_velocity_t *encoder_velocity, encoder_setup_t *encoder_setup);
void encoder_position_(encoder_velocity_t *encoder_velocity, encoder_position_t *encoder_position, encoder_setup_t *encoder_setup);
// define main function
void encoder_main(encoder_position_t* encoder_position, encoder_velocity_t* encoder_velocity,encoder_count_t* encoder_count,
                    encoder_state_t* encoder_state,encoder_setup_t* encoder_setup,encoder_direction_t* encoder_direction,
                    encoder_speed_t* encoder_speed);
                
*/