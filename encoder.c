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

//----------------------------------------------Encoder------------------------------------------------
// encoder interrupt service routine for left encoder
void IRAM_ATTR encoder_left_isr_handler(void* arg)
{
	encoder_left_state = gpio_get_level(ENCODER_A_PIN);
    // count encoder pulses from left encoder
	if (encoder_left_state != encoder_left_state_last)
	{
		encoder_left_count++;
	}
	// save last state
	encoder_left_state_last = encoder_left_state;
}
// encoder handler for count pulses from right encoder
void IRAM_ATTR encoder_right_isr_handler(void* arg)
{
    encoder_right_state = gpio_get_level(ENCODER_B_PIN);
    // count encoder pulses from right encoder
    if (encoder_right_state != encoder_right_state_last)
    {
        encoder_right_count++;
    }
    // save last state
    encoder_right_state_last = encoder_right_state; 
}
// setup encoder pins and interrupt handlers for left and right encoder
void encoder_left_init(void)
{
    gpio_pad_select_gpio(ENCODER_A_PIN); // select gpio 34
    gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT); // set gpio 34 as input
    gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY); // enable pull-up mode
    gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_ANYEDGE); // enable interrupt on both edges
    gpio_install_isr_service(0); 
    gpio_isr_handler_add(ENCODER_A_PIN, encoder_left_isr_handler, (void*) ENCODER_A_PIN); // attach the interrupt service routine
}
void encoder_right_init(void)
{
    gpio_pad_select_gpio(ENCODER_B_PIN); // select gpio 35
    gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT); // set gpio 35 as input
    gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY); // enable pull-up mode
    gpio_set_intr_type(ENCODER_B_PIN, GPIO_INTR_ANYEDGE); // enable interrupt on both edges
    gpio_install_isr_service(0); 
    gpio_isr_handler_add(ENCODER_B_PIN, encoder_right_isr_handler, (void*) ENCODER_B_PIN); // attach the interrupt service routine
}
void SetupEncoder(void)
{
    encoder_left_init();
    encoder_right_init();
}
// define an array of 2 elements 
float vel[2] = {0.0,0.0};
// Function to get encoder data and return linear and angular velocity
float* GetEncoderData(void)
{
    // get direccion of rotation from motor pwm value
    if (ledc_get_duty(PWM_MODE, PWM_LEFT_FORWARD) > 0)
    {
        left_direction = 1;
    }
    else if (ledc_get_duty(PWM_MODE, PWM_LEFT_BACKWARD) > 0)
    {
        left_direction = -1;
    }
    else
    {
        left_direction = 0;
    }
    // get direccion of rotation from motor pwm value
    if (ledc_get_duty(PWM_MODE, PWM_RIGHT_FORWARD) > 0)
    {
        right_direction = 1;
    }
    else if (ledc_get_duty(PWM_MODE, PWM_RIGHT_BACKWARD) > 0)
    {
        right_direction = -1;
    }
    else
    {
        right_direction = 0;
    }
    // calculate speed of left encoder
	encoder_left_speed = encoder_left_count*60000/(FRAME_TIME*ENCODER_PPR); // speed in RPM
    encoder_left_speed = encoder_left_speed*2*PI*WHEEL_RADIUS/60; // speed in m/s
    encoder_left_speed = encoder_left_speed*left_direction; // direction of rotation
    // calculate speed of right encoder
    encoder_right_speed = encoder_right_count*60000/(FRAME_TIME*ENCODER_PPR); // speed in RPM
    encoder_right_speed = encoder_right_speed*2*PI*WHEEL_RADIUS/60; // speed in m/s
    encoder_right_speed = encoder_right_speed*right_direction; // direction of rotation
    // reset encoder count
    encoder_left_count = 0;
    encoder_right_count = 0;
    // calculate linear and angular velocity
    linear_velocity_enc = (encoder_right_speed + encoder_left_speed)/2; 
    angular_velocity_enc = (encoder_right_speed - encoder_left_speed)/(WHEEL_BASE); // angular velocity in rad/s
    // save linear and angular velocity in array
    vel[0] = linear_velocity_enc;
    vel[1] = angular_velocity_enc;
    // return array
    return vel;
}
