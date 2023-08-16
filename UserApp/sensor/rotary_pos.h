#ifndef __SENSOR_ROTARY_POS_H__
#define __SENSOR_ROTARY_POS_H__

#include "common.h"

enum class RotaryDirection { CW = 1, CCW = -1 };

class RotaryPosSensor {
public:
  RotaryPosSensor() = default;
  virtual ~RotaryPosSensor() = default;

  virtual void init() = 0;
  void update();

  void set_direction(RotaryDirection direction);

  float get_velocity();
  float get_lap_angle();
  float get_full_angle();
  int16_t get_rotation_count();

protected:
  virtual float get_raw_angle() = 0;

protected:
  float m_last_raw_angle = 0.0f;
  float m_angle = 0.0f;
  float m_last_angle = 0.0f;
  uint16_t m_rotation_count = 0;
  uint16_t m_last_rotation_count = 0;
  uint64_t m_update_timestamp = 0l;
  uint64_t m_last_update_timestamp = 0l;

  RotaryDirection m_direction = RotaryDirection::CW;
};

#endif // !__SENSOR_ROTARY_POS_H__
