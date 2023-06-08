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


#include "vector_transmission_msgs/msg/vector.hpp"
#include "vector_transmission/VectorConsumer.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace vector_transmission
{

VectorConsumer::VectorConsumer(
  const rclcpp::NodeOptions & options,
  rclcpp::CallbackGroupType callback_option
)
: Node("vector_consumer", options),
  vector_(100, 0.0f)
{
  callback_group_ = create_callback_group(callback_option);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  vector_1_sub_ = create_subscription<vector_transmission_msgs::msg::Vector>(
    "vector_1", 10, std::bind(&VectorConsumer::vector_callback_1, this, _1), sub_options);
  vector_2_sub_ = create_subscription<vector_transmission_msgs::msg::Vector>(
    "vector_2", 10, std::bind(&VectorConsumer::vector_callback_2, this, _1), sub_options);
  sum_pub_ = create_publisher<std_msgs::msg::Float32>("sum_vector", 100);

  timer_ =
    create_wall_timer(10ms, std::bind(&VectorConsumer::control_cycle, this), callback_group_);
}

void
VectorConsumer::vector_callback_1(vector_transmission_msgs::msg::Vector::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Vector 1 Start [%d]", ++counter_);
  vector_ = std::vector<float>(100, 0.0f);
  rclcpp::Rate rate(100000ns);
  for (size_t i = 0; i < msg->data.size(); i++) {
    vector_[i] = msg->data[i];
    rate.sleep();
  }
  RCLCPP_INFO(get_logger(), "Vector 1 End [%d]", counter_);
}

void
VectorConsumer::vector_callback_2(vector_transmission_msgs::msg::Vector::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Vector 2 Start [%d]", ++counter_);
  vector_ = std::vector<float>(100, 0.0f);
  rclcpp::Rate rate(100000ns);
  for (size_t i = 0; i < msg->data.size(); i++) {
    vector_[i] = msg->data[i];
    // rate.sleep();
  }
  RCLCPP_INFO(get_logger(), "Vector 2 End [%d]", counter_);
}

void
VectorConsumer::control_cycle()
{
  std_msgs::msg::Float32 msg;

  float sum = 0.0f;
  for (const auto & value : vector_) {
    sum += value;
  }

  msg.data = sum;
  sum_pub_->publish(msg);
}

}  // namespace vector_transmission

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vector_transmission::VectorConsumer)
