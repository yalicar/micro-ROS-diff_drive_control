#include "imu.h"

void SetupImu() {
    // setup i2c
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER; 
    conf.sda_io_num = GPIO_NUM_21;  
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22; 
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    // setup gyroscope range to 1000 dps (deg/s)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1B, true);
    i2c_master_write_byte(cmd, GYRO_FULL_SCALE_1000_DPS, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // setup accelerometer range to 2g
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1C, true); // 0x1C = ACCEL_CONFIG register address 
    i2c_master_write_byte(cmd, ACCEL_FULL_SCALE_2_G, true); // 0x00 = 2g range 
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // setup magnetometer
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0A, true); // 0x0A = CNTL register address 
    i2c_master_write_byte(cmd, 0x16, true); // 0x16 = 100Hz, 16-bit, continuous measurement mode
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // set by pass mode for magnetometer
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x37, true); // 0x37 = INT_PIN_CFG register address
    i2c_master_write_byte(cmd, 0x02, true); // 0x02 = 0x02 set by pass mode for the magnetometer
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // enable interrupt pin for raw data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x38, true); // 0x38 = INT_ENABLE register address
    i2c_master_write_byte(cmd, 0x01, true); // 0x01 = enable data ready (bit 0) interrupt
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}
// read raw data from IMU
void read_imu_raw() {
    // create data buffer
    uint8_t data[14];
    // read data from IMU
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_IMU_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    // read accelerometer data
    accelerometer.x = (data[0] << 8) | data[1];
    accelerometer.y = (data[2] << 8) | data[3];
    accelerometer.z = (data[4] << 8) | data[5];

    // read temperature data
    temperature.value = (data[6] << 8) | data[7];

    // read gyroscope data
    gyroscope.x = (data[8] << 8) | data[9];
    gyroscope.y = (data[10] << 8) | data[11];
    gyroscope.z = (data[12] << 8) | data[13];

    
    }
// read raw data from magnetometer
void read_magnetometer_raw() {
    // create data buffe
    uint8_t data[7];
    // start magnetometer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0A, true); // 0x0A = CNTL register address
    i2c_master_write_byte(cmd, 0x12, true); // 0x12 = 100Hz, 16-bit, single measurement mode
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS); // 1000 / portTICK_RATE_MS = 1000ms / 1ms = 1s timeout
    i2c_cmd_link_delete(cmd);
    // read data from magnetometer
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x03, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // read magnetometer data
    magnetometer.x = (data[2] << 8) | data[1];
    magnetometer.y = (data[4] << 8) | data[3];
    magnetometer.z = (data[6] << 8) | data[5];
}
// magnetometer sensitivity adjustment
void adjust_magnetometer() {
    // create data buffer
    uint8_t data[3];
    // read magnetometer sensitivity adjustment data from 0x10 register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_WRITE, true); 
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU9250_MAG_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 3, I2C_MASTER_LAST_NACK);    
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    magnetometer.adjustment.x = ((data[0] - 128) / 256.0) + 1;
    magnetometer.adjustment.y = ((data[1] - 128) / 256.0) + 1;
    magnetometer.adjustment.z = ((data[2] - 128) / 256.0) + 1;
}
// normalize accelerometer, gyroscope and magnetometer data
void normalize() {
    read_imu_raw();
    read_magnetometer_raw();
    adjust_magnetometer();
    // normalize accelerometer data to g (9.81 m/s^2)
    normalized.accelerometer.x = accelerometer.x * G / 16384.0; //
    normalized.accelerometer.y = accelerometer.y * G / 16384.0;
    normalized.accelerometer.z = accelerometer.z * G / 16384.0;
    // normalize gyroscope data to degrees per second
    normalized.gyroscope.x = gyroscope.x / 32.8;
    normalized.gyroscope.y = gyroscope.y / 32.8;
    normalized.gyroscope.z = gyroscope.z / 32.8;
    // convert degrees per second to radians per second
    normalized.gyroscope.x = normalized.gyroscope.x * PI / 180.0;
    normalized.gyroscope.y = normalized.gyroscope.y * PI / 180.0;
    normalized.gyroscope.z = normalized.gyroscope.z * PI / 180.0;
    // normalize magnetometer data to microteslas
    normalized.magnetometer.x = magnetometer.x * 0.15 * magnetometer.adjustment.x;
    normalized.magnetometer.y = magnetometer.y * 0.15 * magnetometer.adjustment.y;
    normalized.magnetometer.z = magnetometer.z * 0.15 * magnetometer.adjustment.z;
    //convert microteslas to Tesla
    normalized.magnetometer.x = normalized.magnetometer.x * 0.000001;
    normalized.magnetometer.y = normalized.magnetometer.y * 0.000001;
    normalized.magnetometer.z = normalized.magnetometer.z * 0.000001;
    // normalize temperature data to degrees celsius
    normalized.temperature = (temperature.value / 333.87) + 21.0;
}

