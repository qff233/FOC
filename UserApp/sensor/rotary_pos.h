#ifndef __SENSOR_ROTARY_POS_H__
#define __SENSOR_ROTARY_POS_H__

#include <cstdint>

enum class RotaryDirection { CW = 1, CCW = -1, UNKNOW = 0 };

class RotaryPosSensor {
public:
  RotaryPosSensor() = default;
  virtual ~RotaryPosSensor() = default;

  void init();
  void update();

  float get_velocity();
  float get_lap_angle();
  float get_full_angle();
  int32_t get_rotation_count();

protected:
  virtual float get_raw_angle() = 0;

protected:
  float m_last_raw_angle = 0.0f;
  float m_angle = 0.0f;
  float m_last_angle = 0.0f;
  int32_t m_rotation_count = 0;
  int32_t m_last_rotation_count = 0;
};

#endif // !__SENSOR_ROTARY_POS_H__
