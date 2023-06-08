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


#include <algorithm>

#include "speed_limiter_3/SpeedLimiter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

namespace speed_limiter_3
{

SpeedLimiter::SpeedLimiter()
: Node("speed_limiter")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "vel_in", 10, std::bind(&SpeedLimiter::speed_callback, this, _1));

  declare_parameter("max_lin", 0.5);
  declare_parameter("max_rot", 0.5);
}

geometry_msgs::msg::Twist
SpeedLimiter::clamp_speed(const geometry_msgs::msg::Twist & twist_in)
{
  geometry_msgs::msg::Twist ret_value = twist_in;

  get_parameter("max_lin", max_lin_);
  get_parameter("max_rot", max_rot_);

  ret_value.linear.x = std::clamp(twist_in.linear.x, -max_lin_, max_lin_);
  ret_value.angular.z = std::clamp(twist_in.angular.z, -max_rot_, max_rot_);

  return ret_value;
}

void
SpeedLimiter::speed_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  vel_pub_->publish(clamp_speed(*msg));
}

}  // namespace speed_limiter_3
