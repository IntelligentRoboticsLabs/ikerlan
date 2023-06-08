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
#include "vector_transmission/VectorProducer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

using namespace std::chrono_literals;

namespace vector_transmission
{

VectorProducer::VectorProducer(const rclcpp::NodeOptions & options)
: Node("vector_producer", options)
{
  vector_msg_.data = std::vector<float>(100, 1.0f);
  vector_1_pub_ = create_publisher<vector_transmission_msgs::msg::Vector>("vector_1", 1000);
  vector_2_pub_ = create_publisher<vector_transmission_msgs::msg::Vector>("vector_2", 1000);

  timer_ = create_wall_timer(10ms, std::bind(&VectorProducer::control_cycle, this));
}

void
VectorProducer::control_cycle()
{
  vector_1_pub_->publish(vector_msg_);
  vector_2_pub_->publish(vector_msg_);
}

}  // namespace vector_transmission

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vector_transmission::VectorProducer)
