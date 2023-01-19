#pragma once

// Constants
#define SLEEP_TIME 10
#define PI 3.14159265358979323846

// struct Motor Setup
typedef struct {
    int LED_BUILTIN;
    int PIN_LEFT_FORWARD;
    int PIN_LEFT_BACKWARD;
    int PIN_RIGHT_FORWARD;
    int PIN_RIGHT_BACKWARD;
    int PWM_LEFT_FORWARD;
    int PWM_LEFT_BACKWARD;
    int PWM_RIGHT_FORWARD;
    int PWM_RIGHT_BACKWARD;
    int PWM_FREQUENCY;
    char PWM_RESOLUTION;
    char PWM_TIMER;
    char PWM_MODE;
    int PWM_MOTOR_MIN;
    int PWM_MOTOR_MAX;
} motor_setup_t;

// linear and angular variables
float linear;
float angular;
/*Function prototypes*/
float fmap(float val, float in_min, float in_max, float out_min, float out_max);
void InitMotorDriver(motor_setup_t motor_setup);
void SetMotorSpeed(motor_setup_t motor_setup, float linear, float angular);