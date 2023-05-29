// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>

#include "bt_bumpgo/Turn.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{

using namespace std::chrono_literals;

Turn::Turn(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // create publisher
}

void
Turn::halt()
{
}

BT::NodeStatus
Turn::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  int direction;
  // get direction from blackboard

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;

  if (direction == 1) {
    // change turning direction

    vel_pub_->publish(vel_msgs);
  }

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 1s) {
    // return node status
  } else {
    // return node status
  }

  return BT::NodeStatus::IDLE; // remove this when implemented (just to compile)
}

}  // namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_bumpgo::Turn>("Turn");
}
