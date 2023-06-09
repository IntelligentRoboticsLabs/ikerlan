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

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>

int main(int argc, char ** argv)
{
  // Create a logger
  rclcpp::Logger logger_ = rclcpp::get_logger("image_publisher");

  // Check if the image file exists
  std::string filePath = argv[1];
  std::ifstream file(filePath);
  if (!file) {
    RCLCPP_ERROR(logger_, "File not found: '%s'", filePath.c_str());
    return -1;
  }

  // Initialize ROS and create a node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("image_publisher", options);

  // Declare a parameter of type string with a default value
  node_->declare_parameter<std::string>("topic_image", "image");

  // Get the value of the parameters
  std::string topic_image_;
  node_->get_parameter("topic_image", topic_image_);

  // Print the value of the parameters
  RCLCPP_INFO(logger_, "Publishing '%s' on '%s' topic.", filePath.c_str(), topic_image_.c_str());

  // Create a publisher using rclcpp to publish on the topic
  auto pub = node_->create_publisher<sensor_msgs::msg::Image>(topic_image_, 10);

  // Publish the image
  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    if (pub->get_subscription_count() > 0) {
      // OpenCV Mat image
      cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
      // Convert the image to a ROS message
      std_msgs::msg::Header hdr;
      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
        hdr,
        sensor_msgs::image_encodings::BGR8,
        image).toImageMsg();
      pub->publish(*msg);
    }
    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
}


// ros2 run perception_demo main_image_pub mario.jpg
