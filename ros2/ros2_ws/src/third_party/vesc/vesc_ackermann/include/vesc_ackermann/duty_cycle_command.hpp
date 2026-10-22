// [robocar] Open-loop motor command, independent of ROS for regression tests.
#ifndef VESC_ACKERMANN__DUTY_CYCLE_COMMAND_HPP_
#define VESC_ACKERMANN__DUTY_CYCLE_COMMAND_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vesc_ackermann
{

inline double boundedDutyCycle(double speed, double gain, double limit)
{
  if (!std::isfinite(gain) || gain <= 0.0 ||
    !std::isfinite(limit) || limit <= 0.0 || limit > 1.0)
  {
    throw std::invalid_argument("Duty gain must be positive and duty limit in (0, 1]");
  }
  if (!std::isfinite(speed)) {
    return 0.0;
  }
  return std::max(-limit, std::min(limit, speed * gain));
}

}  // namespace vesc_ackermann
#endif  // VESC_ACKERMANN__DUTY_CYCLE_COMMAND_HPP_
