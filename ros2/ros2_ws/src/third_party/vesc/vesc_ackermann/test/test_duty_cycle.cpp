// Copyright 2026 Robocar contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "vesc_ackermann/duty_cycle_command.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void expectDuty(double speed, double expected)
{
  const double actual = vesc_ackermann::boundedDutyCycle(speed, 0.1, 0.1);
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error("Unexpected duty for command " + std::to_string(speed));
  }
}
}  // namespace

int main()
{
  expectDuty(0.0, 0.0);
  expectDuty(0.45, 0.045);
  expectDuty(-0.5, -0.05);
  expectDuty(1.0, 0.1);
  expectDuty(1000.0, 0.1);
  expectDuty(-1000.0, -0.1);
  expectDuty(std::numeric_limits<double>::max(), 0.1);
  expectDuty(std::numeric_limits<double>::quiet_NaN(), 0.0);
  expectDuty(std::numeric_limits<double>::infinity(), 0.0);
  expectDuty(-std::numeric_limits<double>::infinity(), 0.0);
  for (double limit : {0.0, -0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try {
      vesc_ackermann::boundedDutyCycle(0.5, 0.1, limit);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    if (!rejected) {
      throw std::runtime_error("Invalid duty limit accepted");
    }
  }
  for (double gain : {0.0, -0.1, std::numeric_limits<double>::infinity()}) {
    bool rejected = false;
    try {
      vesc_ackermann::boundedDutyCycle(0.5, gain, 0.1);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    if (!rejected) {
      throw std::runtime_error("Invalid gain accepted");
    }
  }
  std::cout << "Duty: neutral, reverse, saturation, non-finite commands and parameters OK\n";
}
