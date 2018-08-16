#include <Arduino.h>
#include "L298P.h"

Motor::Motor(int8_t pwm_pin,
             int8_t dir_pin,
             int8_t min_pwm) :
  pwm_pin_(pwm_pin),
  dir_pin_(dir_pin),
  min_pwm_(min_pwm) {
}

void Motor::begin() {
  pinMode(pwm_pin_, OUTPUT);
  pinMode(dir_pin_, OUTPUT);
}

void Motor::drive(int speed) {
  digitalWrite(dir_pin_, (speed > 0) ? HIGH : LOW);
  speed = abs(speed);
  int pwm = (speed > 0) ? map(speed, 1, 100, min_pwm_, 255) : 0;
  analogWrite(pwm_pin_, pwm);
}


L298P::L298P(int8_t a_pwm_pin,
             int8_t b_pwm_pin,
             int8_t a_dir_pin,
             int8_t b_dir_pin,
             int8_t a_min_pwm,
             int8_t b_min_pwm) :
    motor_a_(a_pwm_pin, a_dir_pin, a_min_pwm),
    motor_b_(b_pwm_pin, b_dir_pin, b_min_pwm) {
}

void L298P::begin() {
  motor_a_.begin();
  motor_b_.begin();  
}

void L298P::drive(int speed_a, int speed_b) {
  motor_a_.drive(speed_a);
  motor_b_.drive(speed_b);
}

