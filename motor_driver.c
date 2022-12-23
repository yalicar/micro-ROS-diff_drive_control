// Author: Yanmar Cardenas
// Date: 2022-12-15
// Drive a robot with 4 motors using PWM and a L298N motor driver board (Diff Drive Robot)
// Import libraries
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
// Constants
#define FRAME_TIME 200 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10
// Pi Constants
#define PI 3.14159265358979323846
// PINS
#define LED_BUILTIN 33
#define PIN_LEFT_FORWARD 12
#define PIN_LEFT_BACKWARD 13
#define PIN_RIGHT_FORWARD 15
#define PIN_RIGHT_BACKWARD 14
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
// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 4095   // Full speed (2^12 - 1)
// fmap() function
float fmap(float val, float in_min, float in_max, float out_min, float out_max);
//Setup pins for motor driver
void SetupMotor() {
    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 4 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}
// Set motor speed
void SetMotorSpeed(float linear, float angular) {
    // Convert the linear and angular speeds to left and right speeds
    float left_speed = (linear - angular) / 2.0f;
    float right_speed = (linear + angular) / 2.0f;
    // Map the speed to the PWM range
    uint16_t left_pwm = (uint16_t) fmap(fabs(left_speed), 0.0, 1.0, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    uint16_t right_pwm = (uint16_t) fmap(fabs(right_speed), 0.0, 1.0, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    // Set the direction
    ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, left_pwm * (left_speed > 0));
    ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, left_pwm * (left_speed < 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, right_pwm * (right_speed > 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_BACKWARD, right_pwm * (right_speed < 0));
    // Update the PWM channels
    ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_BACKWARD);
}
// fmap function
float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
