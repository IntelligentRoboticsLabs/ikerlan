// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SPEED_LIMITER_3__SPEEDLIMITER_HPP_
#define SPEED_LIMITER_3__SPEEDLIMITER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace speed_limiter_3
{

class SpeedLimiter : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<SpeedLimiter>;
  static std::shared_ptr<SpeedLimiter> make_shared()
  {
    return std::make_shared<SpeedLimiter>();
  }

  SpeedLimiter();

protected:
  geometry_msgs::msg::Twist clamp_speed(const geometry_msgs::msg::Twist & twist_in);

private:
  void speed_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);

  double max_lin_, max_rot_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};

}  // namespace speed_limiter_3

#endif  // SPEED_LIMITER_3__SPEEDLIMITER_HPP_
