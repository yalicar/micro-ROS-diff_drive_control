// Author: Yalmar Cardenas
// Date: 2022-12-15
// Drive a robot with 4 motors using PWM and a L298N motor driver board (Diff Drive Robot)

#include "motor_driver.h"

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

motor_setup_t motor_setup;

//Function to initialize the motor driver
void InitMotorDriver(motor_setup_t motor_setup) {
    motor_setup = motor_setup;
    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(motor_setup.LED_BUILTIN);
    gpio_set_direction(motor_setup.LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT); // Set the pin as output
    gpio_set_level(motor_setup.LED_BUILTIN, !gpio_get_level(motor_setup.LED_BUILTIN));

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = motor_setup.PWM_RESOLUTION,
        .freq_hz = motor_setup.PWM_FREQUENCY,
        .speed_mode = motor_setup.PWM_MODE,
        .timer_num = motor_setup.PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 4 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = motor_setup.PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = motor_setup.PIN_LEFT_FORWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = motor_setup.PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = motor_setup.PIN_LEFT_BACKWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = motor_setup.PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = motor_setup.PIN_RIGHT_FORWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = motor_setup.PWM_RIGHT_BACKWARD,
            .duty       = 0,
            .gpio_num   = motor_setup.PIN_RIGHT_BACKWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

// Set motor speed function (linear and angular) from cmd_vel topic (geometry_msgs/Twist)
void SetMotorSpeed(float linear, float angular) {
    // Convert the linear and angular speeds to left and right speeds
    float left_speed = (linear - angular) / 2.0f;
    float right_speed = (linear + angular) / 2.0f;
    // Map the speed to the PWM range
    uint16_t left_pwm = (uint16_t) fmap(fabs(left_speed), 0.0, 1.0, motor_setup.PWM_MOTOR_MIN, motor_setup.PWM_MOTOR_MAX);
    uint16_t right_pwm = (uint16_t) fmap(fabs(right_speed), 0.0, 1.0, motor_setup.PWM_MOTOR_MIN, motor_setup.PWM_MOTOR_MAX);
    // Set the direction
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_FORWARD, left_pwm * (left_speed > 0));
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_BACKWARD, left_pwm * (left_speed < 0));
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_FORWARD, right_pwm * (right_speed > 0));
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_BACKWARD, right_pwm * (right_speed < 0));
    // Update the PWM channels
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_FORWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_BACKWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_FORWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_BACKWARD);
}

// fmap function
float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
