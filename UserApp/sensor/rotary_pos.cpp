#include "rotary_pos.h"

#include "foc_utils.h"

void RotaryPosSensor::init() {}

void RotaryPosSensor::update() {
  float raw_angle = get_raw_angle();
  float delta_raw_angle = raw_angle - m_last_raw_angle;

  m_angle += delta_raw_angle;
  if (m_angle > _2PI) {
  }
}

float RotaryPosSensor::get_velocity() {}

float RotaryPosSensor::get_lap_angle() {}

float RotaryPosSensor::get_full_angle() {}

int32_t RotaryPosSensor::get_rotation_count() {}
