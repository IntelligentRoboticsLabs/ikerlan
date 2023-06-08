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

using std::placeholders::_1;

class SpeedLimiter : public rclcpp::Node
{
public:
  SpeedLimiter()
  : Node("speed_limiter")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "vel_in", 10, std::bind(&SpeedLimiter::speed_callback, this, _1));
  }

private:
  void speed_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
  {
    auto msg_out = *msg;
    msg_out.linear.x = std::clamp(msg->linear.x, -0.5, 0.5);
    msg_out.angular.z = std::clamp(msg->angular.z, -0.5, 0.5);
    vel_pub_->publish(msg_out);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpeedLimiter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}