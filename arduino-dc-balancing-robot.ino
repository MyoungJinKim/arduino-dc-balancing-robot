#include <Wire.h>
#include "MPU6050.h"
#include "L298P.h"

// MPU6050 sensor
MPU6050 mpu6050(Wire, MPU6050_ADDR_LO);

// Motor driver shield
L298P motors(10, 11, 12, 13);

// update timestamp
unsigned long curr_time = 0;
unsigned long last_time = 0;

// filter values
float pitch_calib = 0.0f;
float gyro_calib = 0.0f;
float pitch = 0.0f;

void calribrate(int num_samples) {
  for (int i = 0; i < num_samples; ++i) {
    mpu6050.update();
    
    float ax = mpu6050.accelX();
    float ay = mpu6050.accelY();
    float az = mpu6050.accelZ();

    pitch_calib += atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    gyro_calib += mpu6050.gyroY();
  }

  pitch_calib /= num_samples;
  gyro_calib  /= num_samples;

  Serial.print("calibrate: ");
  Serial.print(pitch_calib);
  Serial.print(", ");
  Serial.print(gyro_calib);
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // set up motor driver shield
  motors.begin();

  // set up MPU6050 sensor
  Wire.begin();
  mpu6050.begin();

  // calibration
  calribrate(100);
  
  // reset update time
  last_time = millis();
}

float target_y = 0.0f;

float Kp = 12.0;
float Ki =  0.1;
float Kd =  5.0;

float errorP = 0.0f;
float errorI = 0.0f;
float errorD = 0.0f;

void loop() {
  curr_time = millis();
  if (curr_time - last_time > 100) {
    
    // get the sensor value    
    mpu6050.update();

    float ax = mpu6050.accelX();
    float ay = mpu6050.accelY();
    float az = mpu6050.accelZ();

    float accel_angle_y = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    accel_angle_y -= pitch_calib;
    float dt = (curr_time - last_time) * 0.001;
    float gyro_angle_y  = (mpu6050.gyroY() - gyro_calib) * dt;

    pitch = 0.98 * (pitch + gyro_angle_y) + 0.02 * accel_angle_y;

    if (abs(pitch) > 30.0) {
      Serial.println("fall down! stop the robot!");
      motors.drive(0, 0);
      while (true) {}
    }

    errorP  = -(target_y - pitch) / 90.0; // -90~90   => -1 ~ 1
    errorI += errorP;
    errorD  = gyro_angle_y / 250.0;       // -250~250 => -1 ~ 1
    
    float power = Kp * errorP + Ki * errorI + Kd * errorD;

    power = max(-1, min(1, power)); // -1 ~ 1

    int speed = (int)(power * 100);
    motors.drive(speed, speed);

    Serial.print(errorP); Serial.print(", ");
    Serial.print(errorI); Serial.print(", ");
    Serial.print(errorD); Serial.print(", ");
    Serial.print(power); Serial.println(", ");

    last_time = curr_time;
  }
}
