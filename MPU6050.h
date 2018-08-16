#ifndef MPU6050_H_INCLUDED
#define MPU6050_H_INCLUDED

#include <Wire.h>

// MPU6050 I2C Address
#define MPU6050_ADDR_LO           (0x68)
#define MPU6050_ADDR_HI           (0x69)

// Typical MPU6050 registers
#define MPU6050_REG_SMPLRT_DIV    (0x19)
#define MPU6050_REG_CONFIG        (0x1A)
#define MPU6050_REG_GYRO_CONFIG   (0x1B)
#define MPU6050_REG_ACCEL_CONFIG  (0x1C)
#define MPU6050_REG_ACCEL_XOUT_H  (0x3B)
#define MPU6050_REG_POWER_MGNT_1  (0X6B)
#define MPU6050_REG_WHO_AM_I      (0x75)

// MPU6050 class using complimentary filter
class MPU6050 {
  public:
    // create a MPU6050 object
    MPU6050(const TwoWire& wire, uint8_t addr);

    // set up sensor
    void begin();

    // update sensor value
    void update();
    
    // write the data into register
    void write(uint8_t reg, uint8_t data);

    // read a data from the register
    uint8_t read(uint8_t reg, bool stop = false);

    // getters
    float accelX() const { return accel_x_; }
    float accelY() const { return accel_y_; }
    float accelZ() const { return accel_z_; }

    float gyroX() const { return gyro_x_; }
    float gyroY() const { return gyro_y_; }
    float gyroZ() const { return gyro_z_; }

    float temp() const { return temp_; }
    
  private:
    TwoWire wire_;
    uint8_t addr_;

    // accelormeter values
    float accel_x_;
    float accel_y_;
    float accel_z_;

    // gyroscope values
    float gyro_x_;
    float gyro_y_;
    float gyro_z_;

    // temperature value
    float temp_;
};

#endif // MPU6050_H_INCLUDED
