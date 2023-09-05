#include "motor.h"
#include "foc_utils.h"

#include <cmath>

Motor::Motor(int pole_pairs, MotorPWM &motor_pwm, MotorControlMode model,
             float phase_resistance)
    : m_motor_pwm(motor_pwm), m_model(model) {
    m_config.pole_pairs = pole_pairs;
    m_config.phase_resistance = phase_resistance;

    m_config.voltage_limit = 12.0f;
    m_config.velocity_limit = 12.0f;

    m_config.voltage_used_for_sensor_align = 1.0f;

    m_config.lowpass_angle.time_constant = 0.03f;
    m_config.lowpass_velocity.time_constant = 0.03f;

    m_config.pid_current_q = PIDController{3.0f, 300.0f, 0.0f, 0.0f, 12.0f};
    m_config.pid_angle = PIDController{20.0f, 0.0f, 0.0f, 0.0f, 20.0f};
    m_config.pid_velocity = PIDController{0.5f, 10.0f, 0.0f, 1000.0f, 12.0f};
}

void Motor::init() {
    m_motor_pwm.init();
    if (m_rotarypos_sensor) {
        m_rotarypos_sensor->init();
        this->align_rotary_pos_sensor();
    }
}

void Motor::update() {
    if (m_rotarypos_sensor) {
        m_rotarypos_sensor->update();
        m_state.raw_angle = m_rotarypos_sensor->get_full_angle();
        m_state.est_angle = m_config.lowpass_angle(m_state.raw_angle);
        m_state.raw_velocity = m_rotarypos_sensor->get_velocity();
        m_state.est_velocity = m_config.lowpass_velocity(m_state.raw_velocity);
    }
    switch (m_model) {
    case MotorControlMode::ANGLE:
        this->on_angle_close_loop_tick();
        break;
    case MotorControlMode::VELOCITY:
    case MotorControlMode::VELOCITY_OPEN_LOOP:
    default:;
    }
}

void Motor::set_target(float target) {
    switch (m_model) {
    case MotorControlMode::ANGLE:
        m_target_angle = _normalizeAngle(target);
        break;
    case MotorControlMode::VELOCITY:
    case MotorControlMode::VELOCITY_OPEN_LOOP:
        if (std::fabs(target) > m_config.velocity_limit)
            m_target_user = target > 0.0f ? m_config.velocity_limit
                                          : -m_config.velocity_limit;
        else
            m_target_user = target;
        break;
    }
}

void Motor::set_enable(bool enable) { m_enable = enable; }

void Motor::set_torque_limit(float val) {
    m_config.voltage_limit = val;

    if (m_config.voltage_used_for_sensor_align > m_config.voltage_limit)
        m_config.voltage_used_for_sensor_align = m_config.voltage_limit;

    m_config.pid_angle.limit = m_config.voltage_limit;
    m_config.pid_velocity.limit = m_config.voltage_limit;
}

void Motor::set_model(MotorControlMode model) { m_model = model; }

const MotorConfig& Motor::get_config() const { return m_config; }

const MotorSensorState& Motor::get_sensor_value() const { return m_state; }

MotorAlignError Motor::get_error() const { return m_error; }

void Motor::attach_rotary_pos_sensor(RotaryPosSensor *sensor) {
    if (!sensor)
        return;

    m_rotarypos_sensor = sensor;
}

float Motor::get_electrical_angle() {
    if (!m_rotarypos_sensor)
        return 0.0f;

    // To avoid overflow, use ((a mod b) + (c mod b)) to instead of (a+c) mod b,
    // and use ((a mod b) * (c mod b)) = ac mod b
    int16_t rotation_count = m_rotarypos_sensor->get_rotation_count();
    float lap_deg = m_rotarypos_sensor->get_lap_angle();

    float rotation_term = std::fmod((float)rotation_count, _2PI);
    float pole_pairs_term = std::fmod((float)m_config.pole_pairs, _2PI);
    float lap_deg_term = std::fmod((float)lap_deg, _2PI);

    float output = rotation_term * pole_pairs_term + lap_deg_term;
    return _normalizeAngle(output - m_zero_electric_angle_offset);
}

void Motor::align_rotary_pos_sensor() {
    if (!m_rotarypos_sensor)
        return;

    for (int i = 0; i < 500; ++i) {
        float angle = _3PI_2 + _2PI * (float)i / 500.0f;
        this->set_phase_voltage(m_config.voltage_used_for_sensor_align, angle);
        HAL_Delay(2);
    }

    m_rotarypos_sensor->update();
    float mid_angle = m_rotarypos_sensor->get_full_angle();

    for (int i = 500; i >= 0; --i) {
        float angle = _3PI_2 + _2PI * (float)i / 500.0f;
        this->set_phase_voltage(m_config.voltage_used_for_sensor_align, angle);
        HAL_Delay(2);
    }
    m_rotarypos_sensor->update();
    float end_angle = m_rotarypos_sensor->get_full_angle();
    this->set_phase_voltage(0, 0);
    if (mid_angle == end_angle) {
        // TODO: throw error
        return;
    }

    if (mid_angle < end_angle)
        m_rotarypos_sensor->set_direction(RotaryDirection::CCW);
    else
        m_rotarypos_sensor->set_direction(RotaryDirection::CW);
    // check pole pair number
    float delta_angle = std::fabs(mid_angle - end_angle);
    if (std::fabs(delta_angle * m_config.pole_pairs - _2PI) > 0.5f) {
        // TODO: throw error
        return;
    }

    this->set_phase_voltage(m_config.voltage_used_for_sensor_align, _3PI_2);
    HAL_Delay(1000);
    m_rotarypos_sensor->update();
    m_zero_electric_angle_offset = this->get_electrical_angle();
    this->set_phase_voltage(0, 0);
}

void Motor::set_phase_voltage(float voltage_q, float electrical_angle) {
    voltage_q = voltage_q / m_motor_pwm.voltage_power_supply;
    electrical_angle = _normalizeAngle(electrical_angle + _PI_2);
    uint8_t sec = (uint8_t)(electrical_angle / _PI_3);
    float alph = electrical_angle - (float)sec * _PI_3;
    float beta = (float)(sec + 1) * _PI_3 - electrical_angle;

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

void Motor::on_angle_close_loop_tick() {
    float target_velocity =
        m_config.pid_angle(m_target_angle - m_state.est_angle);
    float q = m_config.pid_velocity(target_velocity - m_state.est_velocity);
    set_phase_voltage(q, m_state.est_angle);
}

void Motor::on_velocity_close_loop_tick() {
    float q = m_config.pid_velocity(m_target_user - m_state.est_velocity);
    set_phase_voltage(q, m_state.est_angle);
}

void Motor::on_velocity_open_loop_tick() {
    float q = m_motor_pwm.voltage_power_supply / 2.0f;
    set_phase_voltage(q, m_target_angle);
    m_target_angle = _normalizeAngle(m_target_angle + m_target_user);
}
