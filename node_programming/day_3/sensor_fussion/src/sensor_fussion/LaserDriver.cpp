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


#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_fussion/LaserDriver.hpp"

using namespace std::chrono_literals;

namespace sensor_fussion
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

LaserDriver::LaserDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("laser_driver", options)
{
  laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
}

CallbackReturn
LaserDriver::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] configured", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LaserDriver::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] activated", get_name());

  laser_pub_->on_activate();
  timer_ = create_wall_timer(50ms, std::bind(&LaserDriver::control_sycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LaserDriver::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(get_logger(), "[%s] deactivated", get_name());

  laser_pub_->on_deactivate();
  timer_ = nullptr;

  return CallbackReturn::SUCCESS;
}

void
LaserDriver::control_sycle()
{
  sensor_msgs::msg::LaserScan fake_reading;
  fake_reading.header.stamp = now();
  fake_reading.header.frame_id = "laser_frame";
  fake_reading.angle_min = -M_PI;
  fake_reading.angle_max = M_PI;
  fake_reading.angle_increment = 2.0 * M_PI / 360.0;
  fake_reading.ranges = std::vector<float>(360, 5.0);
  fake_reading.ranges[fake_reading.ranges.size() / 2] = 0.3;

  laser_pub_->publish(fake_reading);
}

}  // namespace sensor_fussion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_fussion::LaserDriver)
