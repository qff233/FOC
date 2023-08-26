#include "rotary_pos.h"

#include "foc_utils.h"

void RotaryPosSensor::update() {
    float raw_angle = get_raw_angle();
    float delta_raw_angle = raw_angle - m_last_raw_angle;

    m_last_angle = m_angle;
    m_last_rotation_count = m_rotation_count;
    m_last_update_timestamp = m_update_timestamp;

    m_angle += delta_raw_angle;
    if (m_angle > _2PI) {
        m_angle -= _2PI;
        m_rotation_count++;
    } else if (m_angle < 0) {
        m_angle += _2PI;
        m_rotation_count--;
    }
    m_update_timestamp = get_microsecond();
}

void RotaryPosSensor::set_direction(RotaryDirection direction) {
    m_direction = direction;
}

float RotaryPosSensor::get_velocity() {
    float del_time =
        ((float)m_update_timestamp - (float)m_last_update_timestamp) * 1e-6f;
    if (del_time <= 0)
        del_time = 1e-3f;
    float del_rotation_deg =
        (float)(m_rotation_count - m_last_rotation_count) * _2PI;
    float del_deg = m_angle - m_last_angle;
    return (float)m_direction * (del_rotation_deg + del_deg) / del_time;
}

float RotaryPosSensor::get_lap_angle() { return (float)m_direction * m_angle; }

float RotaryPosSensor::get_full_angle() {
    return (float)m_direction * (m_angle + (float)m_rotation_count * _2PI);
}

int16_t RotaryPosSensor::get_rotation_count() {
    return (int16_t)m_direction * m_rotation_count;
}
