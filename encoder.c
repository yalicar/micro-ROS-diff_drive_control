#include "encoder.h"

// Global variables struct for encoder state with pointers
encoder_state_t encoder_state = {
    .left = 0,
    .left_last = 0,
    .right = 0,
    .right_last = 0
};
encoder_count_t encoder_count = {
    .left = 0,
    .right = 0
};
encoder_setup_t encoder_setup;  
// encoder_init function
void InitEncoder(encoder_setup_t encoder_setup)
{
    encoder_setup = encoder_setup;
    // define encoder pins
    gpio_pad_select_gpio(encoder_setup.PIN_A); // select gpio
    gpio_set_direction(encoder_setup.PIN_A, GPIO_MODE_INPUT); // set gpio as input
    gpio_set_pull_mode(encoder_setup.PIN_A, GPIO_PULLUP_ONLY); // enable pull-up mode
    gpio_set_intr_type(encoder_setup.PIN_A, GPIO_INTR_ANYEDGE); // enable interrupt on both edges
    gpio_install_isr_service(0);
    //define ISR handler for left encoder pin A with pointer to pin A
    gpio_isr_handler_add(encoder_setup.PIN_A,encoder_left_isr_handler, (void*)encoder_setup.PIN_A);
    gpio_pad_select_gpio(encoder_setup.PIN_B); // select gpio
    gpio_set_direction(encoder_setup.PIN_B, GPIO_MODE_INPUT); // set gpio as input
    gpio_set_pull_mode(encoder_setup.PIN_B, GPIO_PULLUP_ONLY); // enable pull-up mode
    gpio_set_intr_type(encoder_setup.PIN_B, GPIO_INTR_ANYEDGE); // enable interrupt on both edges
    gpio_install_isr_service(0);
    //define ISR handler for right encoder pin B with pointer to pin B
    gpio_isr_handler_add(encoder_setup.PIN_B,encoder_right_isr_handler, (void*)encoder_setup.PIN_B);
    // encoder state variables and encoder count variables
}

// encoder_left_isr_handler function (interrupt service routine)
void IRAM_ATTR encoder_left_isr_handler(void* arg) {
    encoder_state.left = gpio_get_level(encoder_setup.PIN_A);
    // count encoder pulses from left encoder
    if (encoder_state.left != encoder_state.left_last)
    {
        encoder_count.left++;
    }
    // save last state
    encoder_state.left_last = encoder_state.left;
}

// encoder_right_isr_handler function (interrupt service routine)
void IRAM_ATTR encoder_right_isr_handler(void* arg) {
    encoder_state.right = gpio_get_level(encoder_setup.PIN_B);
    // count encoder pulses from right encoder
    if (encoder_state.right != encoder_state.right_last)
    {
        encoder_count.right++;
    }
    // save last state
    encoder_state.right_last = encoder_state.right;
}

// encoder_count_reset function
void encoder_count_reset(encoder_count_t* encoder_count)
{
    encoder_count->left = 0;
    encoder_count->right = 0;
}

// encoder_direction function
void encoder_direction_()
{
    // left encoder
    if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, encoder_setup.left_forward) > 0)
    {
        encoder_state.left = 1;
    }
    else if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, encoder_setup.left_backward) > 0)
    {
        encoder_state.left = -1;
    }
    else
    {
        encoder_state.left = 0;
    }
    // right encoder
    if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, encoder_setup.right_forward) > 0)
    {
        encoder_state.right = 1;
    }
    else if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, encoder_setup.right_backward) > 0)
    {
        encoder_state.right = -1;
    }
    else
    {
        encoder_state.right = 0;
    }
}
/*
// encoder speed function
void encoder_speed_(encoder_count_t* encoder_count, encoder_direction_t* encoder_direction, encoder_speed_t* encoder_speed, encoder_setup_t* encoder_setup)
{
    // left encoder speed calculation (rpm)
    encoder_speed->left = (encoder_count->left * encoder_direction->left * 60) / (encoder_setup->ppr * encoder_setup->resolution * encoder_setup->FRAME_TIME);
    // right encoder speed calculation (rpm)
    encoder_speed->right = (encoder_count->right * encoder_direction->right * 60) / (encoder_setup->ppr * encoder_setup->resolution * encoder_setup->FRAME_TIME);
}
// encoder velocity function
void encoder_velocity_(encoder_speed_t* encoder_speed, encoder_velocity_t* encoder_velocity, encoder_setup_t* encoder_setup)
{
    // linear velocity calculation (m/s)
    encoder_velocity->linear = (encoder_speed->left + encoder_speed->right) * (PI * encoder_setup->wheel_diameter) / (2 * 60);
    // angular velocity calculation (rad/s)
    encoder_velocity->angular = (encoder_speed->right - encoder_speed->left) * (PI * encoder_setup->wheel_diameter) / (encoder_setup->wheel_base * 60);
}
// encoder position function
void encoder_position_(encoder_velocity_t* encoder_velocity, encoder_position_t* encoder_position, encoder_setup_t* encoder_setup)
{
   // x position calculation (m)
    encoder_position->x = encoder_position->x + encoder_velocity->linear * cos(encoder_position->theta) * encoder_setup->FRAME_TIME;
    // y position calculation (m)
    encoder_position->y = encoder_position->y + encoder_velocity->linear * sin(encoder_position->theta) * encoder_setup->FRAME_TIME;
    // theta position calculation (rad)
    encoder_position->theta = encoder_position->theta + encoder_velocity->angular * encoder_setup->FRAME_TIME;
}

// main function
void encoder_main(encoder_position_t* encoder_position, encoder_velocity_t* encoder_velocity,encoder_count_t* encoder_count,
                    encoder_state_t* encoder_state,encoder_setup_t* encoder_setup,encoder_direction_t* encoder_direction,
                    encoder_speed_t* encoder_speed)
{
    // reset encoder count
    encoder_count_reset(encoder_count);
    // set encoder direction
    encoder_direction_(encoder_state, encoder_setup, encoder_direction);
    // calculate encoder speed
    encoder_speed_(encoder_count, encoder_direction, encoder_speed, encoder_setup);
    // calculate encoder velocity
    encoder_velocity_(encoder_speed, encoder_velocity, encoder_setup);
    // calculate encoder position
    encoder_position_(encoder_velocity, encoder_position, encoder_setup);
}
*/