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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("speed_limiter");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
    "vel_in", 10, [publisher](geometry_msgs::msg::Twist msg) {
      msg.linear.x = std::clamp(msg.linear.x, -0.5, 0.5);
      msg.angular.z = std::clamp(msg.angular.z, -0.5, 0.5);
      publisher->publish(msg);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}