#include "lsm9ds1_driver.hpp"

LSM9DS1Driver::LSM9DS1Driver(const std::string &i2c_dev) {
  i2c_file_ = open(i2c_dev.c_str(), O_RDWR);
  if (i2c_file_ < 0) {
    throw std::runtime_error("Failed to open I2C");
  }

  // Set I2C slave address for LSM9DS1
  if (ioctl(i2c_file_, I2C_SLAVE, LSM9DS1_I2C_ADDR) < 0) {
    throw std::runtime_error("Failed to set I2C slave address");
  }
}

LSM9DS1Driver::~LSM9DS1Driver() {
  if (i2c_file_ >= 0) {
    close(i2c_file_);
  }
}

// Function to read byte
uint8_t LSM9DS1Driver::readByte(uint8_t reg) {
    if (write(i2c_file_, &reg, 1) != 1) {
        std::cerr << "Failed to write register address." << std::endl;
        exit(1);
    }

    uint8_t data;
    if (read(i2c_file_, &data, 1) != 1) {
        std::cerr << "Failed to read data from the sensor." << std::endl;
        exit(1);
    }
    return data;
}

int16_t LSM9DS1Driver::readRegister16(uint8_t reg_low, uint8_t reg_high) {
  uint8_t low = readByte(reg_low);
  uint8_t high = readByte(reg_high);
  return (int16_t)(high << 8 | low);
}

ImuData LSM9DS1Driver::readImuData() {
  ImuData imu_data;
  imu_data.accelerometer_x = readRegister16(LSM9DS1_ACCEL_X_L, LSM9DS1_ACCEL_X_H) * 0.061 / 1000.0;  // Sensitivity factor
  imu_data.accelerometer_y = readRegister16(LSM9DS1_ACCEL_Y_L, LSM9DS1_ACCEL_Y_H) * 0.061 / 1000.0;  // Sensitivity factor
  imu_data.accelerometer_z = readRegister16(LSM9DS1_ACCEL_Z_L, LSM9DS1_ACCEL_Z_H) * 0.061 / 1000.0;  // Sensitivity factor
  imu_data.gyroscope_x = readRegister16(LSM9DS1_GYRO_X_L, LSM9DS1_GYRO_X_H) * 8.75 / 1000.0;  // Sensitivity factor
  imu_data.gyroscope_y = readRegister16(LSM9DS1_GYRO_Y_L, LSM9DS1_GYRO_Y_H) * 8.75 / 1000.0;  // Sensitivity factor
  imu_data.gyroscope_z = readRegister16(LSM9DS1_GYRO_Z_L, LSM9DS1_GYRO_Z_H) * 8.75 / 1000.0;  // Sensitivity factor
  return imu_data;
}
