#include "pid.h"

#include "foc_utils.h"

PIDController::PIDController(float p, float i, float d, float ramp, float limit)
    : p(p), i(i), d(d), output_ramp(ramp), limit(limit) {
  m_time_stamp = get_microsecond();
}

float PIDController::operator()(float error) {
  auto time = get_microsecond();
  float dt = (float)(time - m_time_stamp) * 1e-6f;
  if (dt <= 0 || dt > 0.5f)
    dt = 1e-3f;

  float p_term = p * error;
  float i_term = m_last_integral + i * dt * 0.5f * (error + m_last_error);
  i_term = _constrain(i_term, -limit, limit);
  float d_term = d * (error - m_last_error) / dt;

  float output = p_term + i_term + d_term;
  output = _constrain(output, -limit, limit);

  if (output_ramp > 0) {
    float output_rate = (output - m_last_error) / dt;
    if (output_rate > output_ramp) {
      output = m_last_output + output_ramp * dt;
    } else if (output_rate < -output_ramp) {
      output = m_last_output - output_ramp * dt;
    }
  }

  m_last_integral = i_term;
  m_last_output = output;
  m_last_error = error;
  m_time_stamp = time;

  return output;
}
