#pragma once
// driver for reading IMU data from GY-91 sensor (MPU9250 + bmp280)
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

#define MPU9250_IMU_ADDR 0x68
#define MPU9250_MAG_ADDR 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACCEL_FULL_SCALE_2_G 0x00
#define ACCEL_FULL_SCALE_4_G 0x08
#define ACCEL_FULL_SCALE_8_G 0x10
#define ACCEL_FULL_SCALE_16_G 0x18

#define TEMPERATURE_OFFSET 21.0 
#define G 9.80665
#define PI 3.14159265358979323846

// struct for setup imu
struct imu_setup_t {
    
} imu_setup;

// struct for storing gyroscope data raw
struct gyroscope_raw {
    int16_t x;
    int16_t y;
    int16_t z;
} gyroscope;
// struct for storing accelerometer data raw
struct accelerometer_raw {
    int16_t x;
    int16_t y;
    int16_t z;
} accelerometer;
// struct for storing magnetometer data raw
struct magnetometer_raw {
    int16_t x;
    int16_t y;
    int16_t z;
    struct {
        int8_t x;
        int8_t y;
        int8_t z;
    } adjustment;
} magnetometer;
// struct for storing temperature data raw
struct temperature_raw {
    int16_t value;
} temperature;
// struct for storing normalized data
struct normalized_data{
    struct {
        float x;
        float y;
        float z;
    } accelerometer, gyroscope, magnetometer;
    float temperature;
} normalized;

unsigned long last_update = 0;

// define function prototypes with pointer arguments
void InitImu()