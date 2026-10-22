// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"
#include "vesc_ackermann/duty_cycle_command.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare parameters
  // [robocar] Match the Python manual collector: maximum absolute duty 0.10.
  declare_parameter("speed_to_duty_gain", 0.1);
  declare_parameter("max_duty_cycle", 0.1);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);
  // [robocar] The command watchdog is mandatory in duty mode.
  declare_parameter("cmd_timeout", 0.1);

  // get conversion parameters
  speed_to_duty_gain_ = get_parameter("speed_to_duty_gain").get_value<double>();
  max_duty_cycle_ = get_parameter("max_duty_cycle").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
  cmd_timeout_ = get_parameter("cmd_timeout").get_value<double>();
  boundedDutyCycle(0.0, speed_to_duty_gain_, max_duty_cycle_);  // validate parameters
  if (!std::isfinite(cmd_timeout_) || cmd_timeout_ <= 0.0 ||
    !std::isfinite(steering_to_servo_gain_) || !std::isfinite(steering_to_servo_offset_))
  {
    throw std::invalid_argument("Finite steering parameters and a positive cmd_timeout are required");
  }

  // [robocar] No RPM commands: the VESC speed PID is bypassed.
  duty_pub_ = create_publisher<Float64>("commands/motor/duty_cycle", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // [robocar] watchdog, on a steady clock so that system time jumps (chrony) do not affect it
  last_cmd_time_ = steady_clock_.now();
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&AckermannToVesc::watchdogCallback, this));
}

void AckermannToVesc::watchdogCallback()
{
  if ((steady_clock_.now() - last_cmd_time_).seconds() < cmd_timeout_) {
    return;
  }
  if (!timed_out_) {
    RCLCPP_WARN(get_logger(), "No ackermann_cmd for %.2f s, sending zero duty", cmd_timeout_);
    timed_out_ = true;
  }
  publishStop();
}

void AckermannToVesc::publishStop()
{
  Float64 duty_msg;
  duty_msg.data = 0.0;
  duty_pub_->publish(duty_msg);
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  last_cmd_time_ = steady_clock_.now();
  if (timed_out_) {
    RCLCPP_INFO(get_logger(), "ackermann_cmd received again");
    timed_out_ = false;
  }

  // [robocar] Invalid sensor/model output must never become a motor command.
  if (!std::isfinite(cmd->drive.speed) || !std::isfinite(cmd->drive.steering_angle)) {
    RCLCPP_ERROR(get_logger(), "Non-finite Ackermann command, sending zero duty");
    publishStop();
    return;
  }
  Float64 duty_msg;
  duty_msg.data = boundedDutyCycle(cmd->drive.speed, speed_to_duty_gain_, max_duty_cycle_);

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;
  if (!std::isfinite(servo_msg.data)) {
    publishStop();
    return;
  }
  servo_msg.data = std::max(0.0, std::min(1.0, servo_msg.data));

  // publish
  if (rclcpp::ok()) {
    duty_pub_->publish(duty_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
