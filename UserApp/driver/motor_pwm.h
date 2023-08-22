#ifndef __DRIVER_PWM_H__

class MotorPWM {
public:
  MotorPWM(float voltage_power_supply);
  virtual ~MotorPWM() = default;

  virtual void init() = 0;
  virtual void set_voltage(float voltage_a, float voltage_b,
                           float voltage_c) = 0;

public:
  const float voltage_power_supply;

private:
  float m_duty_A;
  float m_duty_B;
  float m_duty_C;
};

#endif // !__DRIVER_PWM_H__
