#ifndef L298P_H_INCLUDED
#define L298P_H_INCLUDED

class Motor {
  public:
    Motor(int8_t pwm_pin,
          int8_t dir_pin,
          int8_t min_pwm);

    void begin();
    void drive(int speed);

  private:
    int8_t pwm_pin_;
    int8_t dir_pin_;
    int8_t min_pwm_;
};

class L298P {
  public:
    L298P(int8_t a_pwm_pin,
          int8_t b_pwm_pin,
          int8_t a_dir_pin,
          int8_t b_dir_pin,
          int8_t a_min_pwm = 80, 
          int8_t b_min_pwm = 80);

    void begin();
    void drive(int speed_a, int speed_b);
    
  private:
    Motor motor_a_;
    Motor motor_b_;
};

#endif // L298P_H_INCLUDED
