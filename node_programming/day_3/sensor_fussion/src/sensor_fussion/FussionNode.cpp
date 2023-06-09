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


#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_fussion/FussionNode.hpp"

using namespace std::chrono_literals;


namespace sensor_fussion
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

FussionNode::FussionNode(const rclcpp::NodeOptions & options)
: LifecycleNode("fussion_node", options)
{
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 100, std::bind(&FussionNode::laser_callback, this, _1));
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "image", 100, std::bind(&FussionNode::image_callback, this, _1));
}

CallbackReturn
FussionNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] configured", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FussionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] activated", get_name());
  timer_ = create_wall_timer(50ms, std::bind(&FussionNode::control_sycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FussionNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] deactivated", get_name());
  timer_ = nullptr;

  return CallbackReturn::SUCCESS;
}

void
FussionNode::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  last_image_ = std::move(msg);
}

void
FussionNode::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
FussionNode::control_sycle()
{
  if (last_image_ == nullptr || last_scan_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Sensor data not completelly received");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Fussing Laser [%f] and Image [%f]",
    rclcpp::Time(last_scan_->header.stamp).seconds(),
    rclcpp::Time(last_image_->header.stamp).seconds());
}

}  // namespace sensor_fussion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_fussion::FussionNode)
