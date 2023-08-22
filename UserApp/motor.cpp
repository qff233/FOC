#include "motor.h"
#include "foc_utils.h"

#include <cmath>

Motor::Motor(int pole_pairs, MotorPWM& motor_pwm,float phase_resistance)
    : m_pole_pairs(pole_pairs), m_motor_pwm(motor_pwm),m_phase_resistance(phase_resistance) {
  m_config.control_mod = MotorControlMode::ANGLE;
  m_config.voltage_limit = 12.0f;
  m_config.velocity_limit = 12.0f;

  m_config.voltage_used_for_sensor_align = 1.0f;

  m_config.lowpass_angle.time_constant = 0.03f;
  m_config.lowpass_velocity.time_constant = 0.03f;

  m_config.pid_current_q = PIDController{3.0f, 300.0f, 0.0f, 0.0f, 12.0f};
  m_config.pid_current_d = PIDController{3.0f, 300.0f, 0.0f, 0.0f, 12.0f};
  m_config.pid_angle = PIDController{20.0f, 0.0f, 0.0f, 0.0f, 20.0f};
  m_config.pid_velocity = PIDController{0.5f, 10.0f, 0.0f, 1000.0f, 12.0f};
}

void Motor::init() {
  // TODO PWM Driver
  // TODO according to PWM Driver voltage limit to change my config limit
  if (m_rotarypos_sensor) {
    m_rotarypos_sensor->init();
    this->align_rotary_pos_sensor();
    m_rotarypos_sensor->update();
    m_state.est_angle = this->get_est_angle();
  }
}

void Motor::update() {}

void Motor::set_enable(bool enable) { m_enable = enable; }

void Motor::set_torque_limit(float val) {
  m_config.voltage_limit = val;

  if (m_config.voltage_used_for_sensor_align > m_config.voltage_limit)
    m_config.voltage_used_for_sensor_align = m_config.voltage_limit;

  m_config.pid_angle.limit = m_config.voltage_limit;
  m_config.pid_velocity.limit = m_config.voltage_limit;
}

float Motor::get_est_angle() {
  if (!m_rotarypos_sensor)
    return m_state.est_angle;
  m_state.raw_angle = m_rotarypos_sensor->get_full_angle();
  m_state.est_angle = m_config.lowpass_angle(m_state.raw_angle);
  return m_state.est_angle;
}

float Motor::get_est_velocity() {
  if (!m_rotarypos_sensor)
    return m_state.est_velocity;
  m_state.raw_velocity = m_rotarypos_sensor->get_velocity();
  m_state.est_velocity = m_config.lowpass_velocity(m_state.raw_velocity);
  return m_state.est_velocity;
}

void Motor::attach_rotary_pos_sensor(RotaryPosSensor *sensor) {
  if (!sensor)
    return;

  m_rotarypos_sensor = sensor;
}

float Motor::get_electrical_angle() {
  if (!m_rotarypos_sensor)
    return 0.0f;

  // To avoid overflow, use ((a mod b) + (c mod b)) instand of (a+c) mod b,
  // and use ((a mod b) * (c mod b)) = ac mod b
  int16_t rotation_count = m_rotarypos_sensor->get_rotation_count();
  float lap_deg = m_rotarypos_sensor->get_lap_angle();

  float rotation_term = std::fmod((float)rotation_count, _2PI);
  float pole_pairs_term = std::fmod((float)m_pole_pairs, _2PI);
  float lap_deg_term = std::fmod((float)lap_deg, _2PI);

  float output = rotation_term * pole_pairs_term + lap_deg_term;
  return output >= 0 ? output : _2PI + output;
}

void Motor::align_rotary_pos_sensor() {}

void Motor::set_phase_voltage(float voltage_q, float electrical_angle) {
  float output;
    
  voltage_q = voltage_q / m_motor_pwm.voltage_power_supply;
  electrical_angle = _normalizeAngle(electrical_angle + _PI_2);
  uint8_t sec = (uint8_t)(electrical_angle / _PI_3);
  float alph = electrical_angle - (float)sec * _PI_3;
  float beta = (float)(sec+1)*_PI_3 - electrical_angle;
  
  float K = voltage_q / _sin(_PI - alph - beta); 
  float t1 = K * _sin(beta);
  float t2 = K * _sin(alph);
  float t0_2 = (1 - t1 - t2) / 2.0f;

  float tA, tB, tC;
  switch (sec) {
    case 1:
        tA = t1 + t2 + t0_2;
        tB = t2 + t0_2;
        tC = t0_2;
        break;
    case 2:
        tA = t1 + t0_2;
        tB = t1 + t2 + t0_2;
        tC = t0_2;
        break;
    case 3:
        tA = t0_2;
        tB = t1 + t2 + t0_2;
        tC = t2 + t0_2;
        break;
    case 4:
        tA = t0_2;
        tB = t1 + t0_2;
        tC = t1 + t2 + t0_2;
        break;
    case 5:
        tA = t2 + t0_2;
        tB = t0_2;
        tC = t1 + t2 + t0_2;
        break;
    case 6:
        tA = t1 + t2 + t0_2;
        tB = t0_2;
        tC = t1 + t0_2;
        break;
    default:
        tA = 0.0f;
        tB = 0.0f;
        tC = 0.0f;
  }

  float voltage_a = tA * m_motor_pwm.voltage_power_supply;
  float voltage_b = tB * m_motor_pwm.voltage_power_supply;
  float voltage_c = tC * m_motor_pwm.voltage_power_supply;

  m_motor_pwm.set_voltage(voltage_a, voltage_b, voltage_c);
}

void Motor::on_angle_close_loop_tick(float target) {}
void Motor::on_velocity_close_loop_tick(float target) {}
void Motor::on_angle_open_loop_tick(float target) {}
void Motor::on_velocity_open_loop_tick(float target) {}
