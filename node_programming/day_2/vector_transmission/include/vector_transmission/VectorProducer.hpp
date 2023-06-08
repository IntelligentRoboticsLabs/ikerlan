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


#ifndef VECTOR_TRANSMISSION__VECTORPRODUCER_HPP_
#define VECTOR_TRANSMISSION__VECTORPRODUCER_HPP_


#include "vector_transmission_msgs/msg/vector.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"


namespace vector_transmission
{

class VectorProducer : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(VectorProducer)

  VectorProducer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void control_cycle();

  vector_transmission_msgs::msg::Vector vector_msg_;

  rclcpp::Publisher<vector_transmission_msgs::msg::Vector>::SharedPtr vector_1_pub_;
  rclcpp::Publisher<vector_transmission_msgs::msg::Vector>::SharedPtr vector_2_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vector_transmission

#endif  // VECTOR_TRANSMISSION__VECTORPRODUCER_HPP_
