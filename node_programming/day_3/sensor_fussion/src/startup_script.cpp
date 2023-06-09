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


#include <algorithm>
#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

unsigned int
get_state(const std::string & node_name, std::chrono::seconds time_out = 1s)
{
  auto node = rclcpp::Node::make_shared("startup_node");
  auto service = node->create_client<lifecycle_msgs::srv::GetState>(
    "/" + node_name + "/get_state");

  while (!service->wait_for_service(time_out)) {
    RCLCPP_WARN(node->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  auto result = service->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    service->remove_pending_request(result);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  } else {
    return result.get()->current_state.id;
  }
}

bool
trigger_transition(
  const std::string & node_name, std::uint8_t transition, std::chrono::seconds time_out = 1s)
{
  auto node = rclcpp::Node::make_shared("startup_node");
  auto service = node->create_client<lifecycle_msgs::srv::ChangeState>(
    "/" + node_name + "/change_state");

  while (!service->wait_for_service(time_out)) {
    RCLCPP_WARN(node->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto result = service->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    service->remove_pending_request(result);
    return false;
  } else {
    if (result.get()->success) {
      RCLCPP_INFO(
        node->get_logger(), "Transition %d successfully.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
        node->get_logger(), "Failed transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::string> node_names = {"camera_driver", "laser_driver", "fussion_node"};

  for (const auto & node_name : node_names) {
    while (get_state(node_name) != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      trigger_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }
  }

  for (const auto & node_name : node_names) {
    while (get_state(node_name) != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      trigger_transition(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
  }

  rclcpp::shutdown();
  return 0;
}
