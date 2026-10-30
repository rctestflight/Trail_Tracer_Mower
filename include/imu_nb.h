// The MPU6050 values from I2C transfers that never hold the loop (test build IMU_I2C_NB, lib/i2c_nb).
// The same getters, scales, offsets and conversion as MPU6050_light fetchData(), so the firmware sees the same values.
// The library's angle filter in update() is left out: nothing in the firmware reads its outputs.
#pragma once
#include <stdint.h>

class ImuNb {
 public:
  static const uint8_t ADDR = 0x68;        // MPU6050_ADDR
  static const uint8_t REG_ACCEL_OUT = 0x3B;  // 14 bytes: accel x y z, temperature, gyro x y z, big-endian
#ifdef IMU_SAVE
  // IMU_SAVE: gyro z at each sample (the steering uses it each step); the other axes from the raw words when asked
  // (the log, 20 a second; the roll, once a frame). The same expressions, so the same values.
  float getAccX() const { return ((float)raw[0]) / acc_lsb_to_g - accXoffset; }
  float getAccY() const { return ((float)raw[1]) / acc_lsb_to_g - accYoffset; }
  float getAccZ() const { return ((float)raw[2]) / acc_lsb_to_g - accZoffset; }
  float getGyroX() const { return ((float)raw[4]) / gyro_lsb_to_degsec - gyroXoffset; }
  float getGyroY() const { return ((float)raw[5]) / gyro_lsb_to_degsec - gyroYoffset; }
#else
  float getAccX() const { return accX; }
  float getAccY() const { return accY; }
  float getAccZ() const { return accZ; }
  float getGyroX() const { return gyroX; }
  float getGyroY() const { return gyroY; }
#endif
  float getGyroZ() const { return gyroZ; }
  int16_t rawGyroZ() const { return raw[6]; }
  float gyroLsb() const { return gyro_lsb_to_degsec; }
  float getGyroXoffset() const { return gyroXoffset; }
  float getGyroYoffset() const { return gyroYoffset; }
  float getGyroZoffset() const { return gyroZoffset; }
  void setGyroOffsets(float x, float y, float z) { gyroXoffset = x; gyroYoffset = y; gyroZoffset = z; }
  void setAccOffsets(float x, float y, float z) { accXoffset = x; accYoffset = y; accZoffset = z; }
  // The scale of MPU6050_light setGyroConfig() / setAccConfig() for config_num 0..3; the register write is separate.
  // Returns the register value, or -1 for a number out of range (the scale stays).
  int gyroScale(int config_num) {
    static const float k[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    if (config_num < 0 || config_num > 3) return -1;
    gyro_lsb_to_degsec = k[config_num];
    return config_num << 3;
  }
  int accScale(int config_num) {
    static const float k[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    if (config_num < 0 || config_num > 3) return -1;
    acc_lsb_to_g = k[config_num];
    return config_num << 3;
  }
  // MPU6050_light fetchData() after the bus read: d = the 14 bytes from REG_ACCEL_OUT.
  void convert(const uint8_t* d) {
    for (int i = 0; i < 7; i++) raw[i] = (int16_t)(d[2 * i] << 8 | d[2 * i + 1]);
    gyroZ = ((float)raw[6]) / gyro_lsb_to_degsec - gyroZoffset;
#ifndef IMU_SAVE
    accX = ((float)raw[0]) / acc_lsb_to_g - accXoffset;
    accY = ((float)raw[1]) / acc_lsb_to_g - accYoffset;
    accZ = ((float)raw[2]) / acc_lsb_to_g - accZoffset;
    gyroX = ((float)raw[4]) / gyro_lsb_to_degsec - gyroXoffset;
    gyroY = ((float)raw[5]) / gyro_lsb_to_degsec - gyroYoffset;
#endif
  }

 private:
  float gyro_lsb_to_degsec = 65.5f, acc_lsb_to_g = 16384.0f;
  float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
  float accXoffset = 0, accYoffset = 0, accZoffset = 0;
  float accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
  int16_t raw[7] = {};  // the last 14 bytes as words: accel x y z, temperature, gyro x y z
};
