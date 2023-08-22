#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"
#include "driver/rotary_pos.h"
#include "driver/motor_pwm.h"
#include "foc_utils.h"
#include "math/lowpass_filter.h"
#include "math/pid.h"

enum class MotorControlMode {
  // TORQUE,
  VELOCITY,
  ANGLE,
  VELOCITY_OPEN_LOOP,
  ANGLE_OPEN_LOOP,
};

struct MotorConfig {
  MotorControlMode control_mod;

  float voltage_limit;
  // float current_limit;
  float velocity_limit;

  float voltage_used_for_sensor_align;

  // LowPassFilter lowpass_current_q;
  // LowPassFilter lowpass_current_d;
  LowPassFilter lowpass_angle;
  LowPassFilter lowpass_velocity;

  PIDController pid_current_q;
  PIDController pid_current_d;
  PIDController pid_angle;
  PIDController pid_velocity;
};

struct MotorState {
  float raw_angle;
  float est_angle;
  float raw_velocity;
  float est_velocity;
};

class Motor {
public:
  Motor(int pole_pairs, MotorPWM& motor_pwm,float phase_resistance = NOT_SET);

  void update();
  void init();

  float get_est_angle();
  float get_est_velocity();

  void set_enable(bool enable);
  void set_torque_limit(float val);
  void attach_rotary_pos_sensor(RotaryPosSensor *sensor);

private:
  float get_electrical_angle();
  void align_rotary_pos_sensor();
  // void measure_phase_resistance();
  void set_phase_voltage(float voltage_q, float electrical_angle);

  void on_angle_close_loop_tick(float target);
  void on_velocity_close_loop_tick(float target);
  void on_angle_open_loop_tick(float target);
  void on_velocity_open_loop_tick(float target);

private:
  MotorConfig m_config;

  MotorState m_state;
  DQVoltage m_voltage;
  PhaseVoltage m_phase_voltage;

  int m_pole_pairs;
  float m_phase_resistance;
  float m_zero_electric_angle_offset = NOT_SET;

  bool m_enable = false;

  MotorPWM& m_motor_pwm;
  RotaryPosSensor *m_rotarypos_sensor = nullptr;
};

#endif
