#ifndef __MATH_LOWPASS_FILTER_H__
#define __MATH_LOWPASS_FILTER_H__

#include "common.h"

class LowPassFilter {
public:
  LowPassFilter() = default;
  LowPassFilter(float time_constant);
  float operator()(float input);

public:
  float time_constant;

private:
  uint64_t m_time_stamp;
  float m_last_output;
};

#endif
