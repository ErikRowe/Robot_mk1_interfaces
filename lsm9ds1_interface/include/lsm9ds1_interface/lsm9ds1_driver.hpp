#ifndef LSM9DS1_DRIVER_HPP_
#define LSM9DS1_DRIVER_HPP_

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define LSM9DS1_I2C_ADDR 0x6b   // Address of the LSM9DS1 accelerometer and gyroscope
#define LSM9DS1_ADDRESS_MAG 0x1E// Address of the LSM9DS1 magnetometer
#define LSM9DS1_ACCEL_X_L 0x28  // Register for reading the accelerometer X-axis data
#define LSM9DS1_ACCEL_X_H 0x29
#define LSM9DS1_ACCEL_Y_L 0x2A  // Register for reading the accelerometer Y-axis data
#define LSM9DS1_ACCEL_Y_H 0x2B
#define LSM9DS1_ACCEL_Z_L 0x2C
#define LSM9DS1_ACCEL_Z_H 0x2D
#define LSM9DS1_GYRO_X_L 0x18   // Register for reading the gyroscope X-axis data
#define LSM9DS1_GYRO_X_H 0x19
#define LSM9DS1_GYRO_Y_L 0x1A
#define LSM9DS1_GYRO_Y_H 0x1B
#define LSM9DS1_GYRO_Z_L 0x1C
#define LSM9DS1_GYRO_Z_H 0x1D

#define SCALE_GYRO 245 // Full-scale range of the gyroscope in degrees per second
#define SCALE_ACCEL 2 // Full-scale range of the accelerometer in g
#define SCALE_MAG 4 // Full-scale range of the magnetometer in gauss

struct ImuData {
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;
};

class LSM9DS1Driver {
public:
  LSM9DS1Driver(const std::string &i2c_dev);
  ~LSM9DS1Driver();

  ImuData readImuData();

private:
  int i2c_file_;
  int16_t readRegister16(uint8_t reg_low, uint8_t reg_high);
  uint8_t readByte(uint8_t reg);
};
#endif /* LSM9DS1_DRIVER_HPP_ */
