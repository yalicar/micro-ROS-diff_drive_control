#include "encoder.h"
//----------------------------------------------Encoder------------------------------------------------
// encoder interrupt service routine for left encoder
void IRAM_ATTR encoder_left_isr_handler(void* arg)
{
    encoder_state.left = gpio_get_level(ENCODER_A_PIN);
    // count encoder pulses from left encoder
    if (encoder_state.left != encoder_state.left_last)
    {
        encoder_count.left++;
    }
    // save last state
    encoder_state.left_last = encoder_state.left;
}
// encoder handler for count pulses from right encoder
void IRAM_ATTR encoder_right_isr_handler(void* arg)
{
    encoder_state.right = gpio_get_level(ENCODER_B_PIN);
    // count encoder pulses from right encoder
    if (encoder_state.right != encoder_state.right_last)
    {
        encoder_count.right++;
    }
    // save last state
    encoder_state.right_last = encoder_state.right;
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
void EncoderInit(void)
{
    encoder_left_init();
    encoder_right_init();
}
// Function to get direction of rotation from motor pwm value
void GetDirection(void)
{
    // get direccion of rotation from motor pwm value
    if (ledc_get_duty(PWM_MODE, PWM_LEFT_FORWARD) > 0)
    {
        direction.left = 1;
    }
    else if (ledc_get_duty(PWM_MODE, PWM_LEFT_BACKWARD) > 0)
    {
        direction.left = -1;
    }
    else
    {
        direction.left = 0;
    }
    // get direccion of rotation from motor pwm value
    if (ledc_get_duty(PWM_MODE, PWM_RIGHT_FORWARD) > 0)
    {
        direction.right = 1;
    }
    else if (ledc_get_duty(PWM_MODE, PWM_RIGHT_BACKWARD) > 0)
    {
        direction.right = -1;
    }
    else
    {
        direction.right = 0;
    }
}
// function to get linear and angular velocity from encoder data
void GetVelocity(void)
{
    // get direction of rotation
    GetDirection();
    // calculate speed of left encoder
    encoder_speed.left = encoder_count.left*60000/(FRAME_TIME*ENCODER_PPR); // speed in RPM
    encoder_speed.left = encoder_speed.left*2*PI*WHEEL_RADIUS/60; // speed in m/s
    encoder_speed.left = encoder_speed.left*direction.left; // direction of rotation
    // calculate speed of right encoder
    encoder_speed.right = encoder_count.right*60000/(FRAME_TIME*ENCODER_PPR); // speed in RPM
    encoder_speed.right = encoder_speed.right*2*PI*WHEEL_RADIUS/60; // speed in m/s
    encoder_speed.right = encoder_speed.right*direction.right; // direction of rotation
    // calculate linear and angular velocity
    velocity_enc.linear = (encoder_speed.left + encoder_speed.right)/2; // linear velocity
    velocity_enc.angular = (encoder_speed.right - encoder_speed.left)/WHEEL_BASE; // angular velocity
    // reset encoder count
    encoder_count.left = 0;
    encoder_count.right = 0;
}