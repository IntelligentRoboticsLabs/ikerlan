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

#include "perception_demo/PCLPubSub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create a ros2 node
  auto node = std::make_shared<perception_demo::PCLPubSub>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

// ros2 run perception_demo main_pcl_pub_sub --ros-args --remap /pointcloud_in:=/head_front_camera/depth_registered/points
