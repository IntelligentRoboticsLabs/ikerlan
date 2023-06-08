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


#ifndef VECTOR_TRANSMISSION__VECTORCONSUMER_HPP_
#define VECTOR_TRANSMISSION__VECTORCONSUMER_HPP_


#include "vector_transmission_msgs/msg/vector.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"


namespace vector_transmission
{

class VectorConsumer : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(VectorConsumer)

  VectorConsumer(
    rclcpp::CallbackGroupType callback_option = rclcpp::CallbackGroupType::MutuallyExclusive);

private:
  void vector_callback_1(vector_transmission_msgs::msg::Vector::SharedPtr msg);
  void vector_callback_2(vector_transmission_msgs::msg::Vector::SharedPtr msg);
  void control_cycle();

  std::vector<float> vector_;
  int counter_ {0};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sum_pub_;
  rclcpp::Subscription<vector_transmission_msgs::msg::Vector>::SharedPtr vector_1_sub_;
  rclcpp::Subscription<vector_transmission_msgs::msg::Vector>::SharedPtr vector_2_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace vector_transmission

#endif  // VECTOR_TRANSMISSION__VECTORCONSUMER_HPP_
