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

#include <utility>
#include "rclcpp/rclcpp.hpp"

#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class BumpGo : public rclcpp::Node
{
public:
  BumpGo(): Node("fsm_bumpgo"), state_(FORWARD), pressed_(false)
  {
    // vel_pub_ = create_publisher<...>(...);
    // bumper_sub_ = create_subscription<...>(...);
    timer_ = create_wall_timer(50ms, std::bind(&BumpGo::step, this));
  }

  void bumperCallback(const kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
  {
    // pressed_ = (...);

    //  ...
  }

  void step()
  {
    geometry_msgs::msg::Twist cmd;

    switch (state_) {
        case FORWARD:
        // cmd.linear.x = ...;
        // cmd.angular.z = ...;

        if (pressed_)
        {
            state_ts_ = now();
            state_ = BACK;
            RCLCPP_INFO(get_logger(), "FORWARD -> BACK");
        }
        break;

        case BACK:
        // cmd.linear.x = ...;
        // cmd.angular.z = ...;

        if ((now() - state_ts_) > BACKING_TIME ) {
            state_ts_ = now();
            state_ = TURN;
            RCLCPP_INFO(get_logger(), "BACK -> TURN");
        }

        break;

        case TURN:
        // cmd.linear.x = ...;
        // cmd.angular.z = ...;

        if ((now() - state_ts_) > TURNING_TIME ) {
            state_ = FORWARD;
            RCLCPP_INFO(get_logger(), "TURN -> FORWARD");
        }

        break;
    }

    // vel_pub_.publish(...);
  }

private:
  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;

  int state_;
  rclcpp::Time state_ts_;
  bool pressed_;

  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto bumpgo_node = std::make_shared<BumpGo>();
  rclcpp::spin(bumpgo_node);

  rclcpp::shutdown();

  return 0;
}