#include <Arduino.h>
#include "MPU6050.h"

MPU6050::MPU6050(const TwoWire& wire, uint8_t addr) :
    wire_(wire),
    addr_(addr),
    accel_x_(0.0),
    accel_y_(0.0),
    accel_z_(0.0),
    gyro_x_(0.0),
    gyro_y_(0.0),
    gyro_z_(0.0),
    temp_(0.0) {
}

void MPU6050::begin() {
  uint8_t addr = read(MPU6050_REG_WHO_AM_I, true);
  if (addr != addr_) {
    Serial.print("WHO_AM_I error: ");
    Serial.println(addr);
    while (true) ;
  }
  write(MPU6050_REG_SMPLRT_DIV, 7);   // sample rate: 8k 
  write(MPU6050_REG_CONFIG, 0);       // output rate: 8k
  write(MPU6050_REG_GYRO_CONFIG, 0);  // gyro range: 250 dps
  write(MPU6050_REG_ACCEL_CONFIG, 0); // accel range: 2g

  // wake up the sensor
  write(MPU6050_REG_POWER_MGNT_1, 0);

  // wait the sensor to be stabilized
  delay(2000);

  // update sensor value
  update();
}

void MPU6050::write(uint8_t reg, uint8_t data) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(data);
  wire_.endTransmission();
}

uint8_t MPU6050::read(uint8_t reg, bool stop) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.endTransmission(false);
  wire_.requestFrom((int)addr_, (int)1, (int)false);
  uint8_t data = wire_.read();
  wire_.endTransmission(stop);
  return data;
}

void MPU6050::update() {
  wire_.beginTransmission(addr_);
  wire_.write(MPU6050_REG_ACCEL_XOUT_H);
  wire_.endTransmission(false);
  wire_.requestFrom((int)addr_, (int)14, (int)true);

  while (wire_.available() < 14) ;
  
  int16_t raw_ax   = wire_.read() << 8 | wire_.read();
  int16_t raw_ay   = wire_.read() << 8 | wire_.read();
  int16_t raw_az   = wire_.read() << 8 | wire_.read();
  int16_t raw_temp = wire_.read() << 8 | wire_.read();
  int16_t raw_gx   = wire_.read() << 8 | wire_.read();
  int16_t raw_gy   = wire_.read() << 8 | wire_.read();
  int16_t raw_gz   = wire_.read() << 8 | wire_.read();

  accel_x_ = (float) raw_ax / 16384.0;
  accel_y_ = (float) raw_ay / 16384.0;
  accel_z_ = (float) raw_az / 16384.0;

  temp_ = (float) raw_temp / 340.0 + 36.35;

  gyro_x_ = (float) raw_gx / 131.0;
  gyro_y_ = (float) raw_gy / 131.0;
  gyro_z_ = (float) raw_gz / 131.0;
}

