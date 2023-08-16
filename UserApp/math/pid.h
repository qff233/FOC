#ifndef __MATH_PID_H__
#define __MATH_PID_H__

#include "common.h"

class PIDController {
public:
  PIDController() = default;
  PIDController(float p, float i, float d, float ramp, float limit);

  float operator()(float error);

public:
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  float output_ramp = 0.0f;
  float limit = 0;

private:
  float m_last_error = 0.0f;
  float m_last_output = 0.0f;
  float m_last_integral = 0.0f;
  uint64_t m_time_stamp = 0.0f;
};

#endif // !__MATH_PID_H__
