#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"
#include "foc_utils.h"
#include "math/lowpass_filter.h"
#include "math/pid.h"
#include "sensor/rotary_pos.h"

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
  Motor(int pole_pairs, float phase_resistance = NOT_SET);

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
  void set_phase_voltage(float voltage_q, float voltage_d,
                         float electrical_angle);
  void angle_close_loop_tick(float target);
  void velocity_close_loop_tick(float target);
  void angle_open_loop_tick(float target);
  void velocity_open_loop_tick(float target);

private:
  MotorConfig m_config;

  MotorState m_state;
  DQVoltage m_voltage;
  PhaseVoltage m_phase_voltage;

  int m_pole_pairs;
  float m_phase_resistance;
  float m_zero_electric_angle_offset = NOTSET;

  bool m_enable = false;

  RotaryPosSensor *m_rotarypos_sensor = nullptr;
};

#endif
