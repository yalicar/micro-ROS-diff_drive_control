#include "encoder.h"

// Global variables struct for encoder initialization
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

encoder_direction_t encoder_direction = {
    .left = 0,
    .right = 0
};
encoder_speed_t encoder_speed = {
    .left = 0,
    .right = 0
};
encoder_velocity_t encoder_velocity = {
    .linear = 0,
    .angular = 0
};
encoder_position_t encoder_position = {
    .x = 1,
    .y = 0,
    .theta = 0,
    .x_last = 0,
    .y_last = 0,
    .theta_last = 0
};

// encoder_init function
void InitEncoder(encoder_setup_t encoder_setup) {
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
void IRAM_ATTR encoder_left_isr_handler() {
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
void IRAM_ATTR encoder_right_isr_handler() {
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
void encoder_direction_(encoder_direction_t* encoder_direction)
{
    // left encoder direction
    if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, motor_setup.PWM_LEFT_FORWARD) > 0)
    {
        encoder_direction->left = 1;
    }
    else if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, motor_setup.PWM_LEFT_BACKWARD) > 0)
    {
        encoder_direction->left = -1;
    }
    else
    {
        encoder_direction->left = 0;
    }
    // right encoder direction
    if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, motor_setup.PWM_RIGHT_FORWARD) > 0)
    {
        encoder_direction->right = 1;
    }
    else if (ledc_get_duty(LEDC_HIGH_SPEED_MODE, motor_setup.PWM_RIGHT_BACKWARD) > 0)
    {
        encoder_direction->right = -1;
    }
    else
    {
        encoder_direction->right = 0;
    }
}

// encoder speed function
void encoder_speed_(encoder_speed_t* encoder_speed)
{
    // left encoder speed calculation (rpm)
    encoder_speed->left = (encoder_count.left * encoder_direction.left * 60) / (encoder_setup.PULSES_PER_REV * encoder_setup.RESOLUTION * encoder_setup.FRAME_TIME_MS);
    // right encoder speed calculation (rpm)
    encoder_speed->right = (encoder_count.right * encoder_direction.right * 60) / (encoder_setup.PULSES_PER_REV * encoder_setup.RESOLUTION * encoder_setup.FRAME_TIME_MS);
}

// encoder velocity function
void encoder_velocity_(encoder_velocity_t* encoder_velocity)
{
    // linear velocity calculation (m/s)
    encoder_velocity->linear = (encoder_speed.left + encoder_speed.right) * (PI * encoder_setup.WHEEL_DIAMETER) / (2 * 60);
    // angular velocity calculation (rad/s)
    encoder_velocity->angular = (encoder_speed.right - encoder_speed.left) * (PI * encoder_setup.WHEEL_DIAMETER) / (encoder_setup.WHEEL_BASE * 60);
}

// encoder position function
void encoder_position_(encoder_position_t* encoder_position)
{
   // x position calculation (m)
    encoder_position->x = encoder_position->x + encoder_velocity.linear * cos(encoder_position->theta) * encoder_setup.FRAME_TIME_MS;
    // y position calculation (m)
    encoder_position->y = encoder_position->y + encoder_velocity.linear * sin(encoder_position->theta) * encoder_setup.FRAME_TIME_MS;
    // theta position calculation (rad)
    encoder_position->theta = encoder_position->theta + encoder_velocity.angular * encoder_setup.FRAME_TIME_MS;
}
// main function
void GetEncoder()
{
    // reset encoder count
    encoder_count_reset(&encoder_count);
    // get encoder direction
    encoder_direction_(&encoder_direction);
    // get encoder speed
    encoder_speed_(&encoder_speed);
    // get encoder velocity
    encoder_velocity_(&encoder_velocity);
    // get encoder position
    encoder_position_(&encoder_position);
}
