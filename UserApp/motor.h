#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"
#include "driver/motor_pwm.h"
#include "driver/rotary_pos.h"
#include "foc_utils.h"
#include "math/lowpass_filter.h"
#include "math/pid.h"

enum class MotorControlMode {
    // TORQUE,
    VELOCITY,
    ANGLE,
    VELOCITY_OPEN_LOOP,
};

enum class MotorAlignError {
    None,
    POLE_PAIR_MISMATCH,
    FAILD_TO_MOVE,
};

struct MotorConfig {

    int pole_pairs;
    float phase_resistance;

    float voltage_limit;
    float current_limit;
    float velocity_limit;

    float voltage_used_for_sensor_align;

    // LowPassFilter lowpass_current_q;
    LowPassFilter lowpass_angle;
    LowPassFilter lowpass_velocity;

    PIDController pid_current_q;
    PIDController pid_angle;
    PIDController pid_velocity;
};

struct MotorSensorState {
    float raw_angle = 0.0f;
    float est_angle = 0.0f;
    float raw_velocity = 0.0f;
    float est_velocity = 0.0f;
};

class Motor {
  public:
    Motor(int pole_pairs, MotorPWM &motor_pwm,
          MotorControlMode model = MotorControlMode::ANGLE,
          float phase_resistance = 0.0);

    void update();
    void init();

    const MotorConfig &get_config() const;
    const MotorSensorState &get_sensor_value() const;
    MotorAlignError get_error() const;

    void set_target(float target);
    void set_enable(bool enable);
    void set_torque_limit(float val);
    void set_model(MotorControlMode model);
    void attach_rotary_pos_sensor(RotaryPosSensor *sensor);

  private:
    float get_electrical_angle();
    void align_rotary_pos_sensor();
    // void measure_phase_resistance();
    void set_phase_voltage(float voltage_q, float electrical_angle);

    void on_angle_close_loop_tick();
    void on_velocity_close_loop_tick();
    void on_velocity_open_loop_tick();

  private:
    MotorControlMode m_model;
    MotorConfig m_config;

    MotorSensorState m_state;
    float m_target_user = 0.0f;
    float m_target_angle = 0.0f;
    // DQCurrent m_target_current;
    // PhaseVoltage m_phase_voltage;
    float m_timestamp = 0.0f;

    float m_zero_electric_angle_offset = 0.0f;

    bool m_enable = false;

    MotorPWM &m_motor_pwm;
    RotaryPosSensor *m_rotarypos_sensor = nullptr;
    MotorAlignError m_error;
};

#endif
