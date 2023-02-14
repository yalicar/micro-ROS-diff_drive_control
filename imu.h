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
typedef struct {
    int SDA;
    int SCL;
    //int GYRO_FULL_SCALE;
    //int ACCEL_FULL_SCALE;
} imu_setup_t;
// struct for storing gyro data raw
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} gyro_raw_t;

// struct for storing accelerometer data raw
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_raw_t;

// struct for storing magnetometer data raw with anidated struct
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    struct {
        int8_t x;
        int8_t y;
        int8_t z;
    } adjustment;
} mag_raw_t;

// struct for storing temperature data raw
typedef struct {
    int16_t value;
} temp_raw_t;

// struct for storing normalized data
typedef struct{
    struct {
        float x;
        float y;
        float z;
    } accelerometer, gyroscope, magnetometer;
    float temperature;
} normalized_data_t;

//unsigned long last_update = 0;

// define function prototypes with pointer arguments
void InitImu(imu_setup_t imu_setup);
void read_imu_raw();
void read_magnetometer_raw();
void adjust_magnetometer();
void normalize();
void GetImu();