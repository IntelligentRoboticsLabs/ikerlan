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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_fussion/CameraDriver.hpp"

using namespace std::chrono_literals;

namespace sensor_fussion
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CameraDriver::CameraDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("camera_driver", options)
{
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image", 100);
}

CallbackReturn
CameraDriver::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] configured", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CameraDriver::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] activated", get_name());

  image_pub_->on_activate();
  timer_ = create_wall_timer(100ms, std::bind(&CameraDriver::control_sycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CameraDriver::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] deactivated", get_name());

  image_pub_->on_deactivate();
  timer_ = nullptr;

  return CallbackReturn::SUCCESS;
}

void
CameraDriver::control_sycle()
{
  sensor_msgs::msg::Image fake_image;
  fake_image.header.stamp = now();
  fake_image.header.frame_id = "camera_frame";
  // Rest of the data are not used

  image_pub_->publish(fake_image);
}

}  // namespace sensor_fussion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_fussion::CameraDriver)
