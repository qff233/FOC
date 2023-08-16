#include "lowpass_filter.h"

#include "foc_utils.h"

LowPassFilter::LowPassFilter(float time_constant)
    : time_constant(time_constant) {
  m_time_stamp = get_microsecond();
}

float LowPassFilter::operator()(float input) {
  uint64_t time = get_microsecond();
  float dt = ((float)time - (float)m_time_stamp) * 1e-6f;
  if (dt < 0.0f)
    dt = 1e-3f;
  else if (dt > 0.3f) {
    m_last_output = input;
    m_time_stamp = time;
    return input;
  }

  float alpha = time_constant / (time_constant + dt);
  float output = alpha * m_last_output + (1.0f - alpha) * input;
  m_last_output = output;
  m_time_stamp = time;

  return output;
}
