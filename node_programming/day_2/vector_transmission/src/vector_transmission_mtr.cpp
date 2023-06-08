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


#include "vector_transmission/VectorProducer.hpp"
#include "vector_transmission/VectorConsumer.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_producer = vector_transmission::VectorProducer::make_shared();
  auto node_consumer = vector_transmission::VectorConsumer::make_shared(
    rclcpp::NodeOptions(),
    rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
  exe.add_node(node_producer);
  exe.add_node(node_consumer);

  exe.spin();

  rclcpp::shutdown();
  return 0;
}
