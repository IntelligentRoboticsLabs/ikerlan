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
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

std::string transport_;
auto logger_ = rclcpp::get_logger("image_transport_subscriber");

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // Show the image using OpenCV
    cv::imshow(transport_, cv_bridge::toCvShare(msg, msg->encoding.c_str())->image);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create a node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("image_transport", options);

  std::string topic_image_ = "image_transport";
  // Add a slash to the topic name if it does not have one
  if (topic_image_.find("/") != 0) {
    topic_image_.insert(0, "/");
  }

  transport_ = argv[1];

  // Create a window to show the image
  cv::namedWindow(transport_);
  cv::startWindowThread();

  // Create an ImageTransport instance, initializing it with the Node
  image_transport::Subscriber subscriber_;
  // Assign the subscriber to a specific transport
  const image_transport::TransportHints hints(node_.get(), transport_);
  try {
    auto subscription_options = rclcpp::SubscriptionOptions();
    // Create a subscription with QoS profile that will be used for the subscription.
    subscriber_ = image_transport::create_subscription(
      node_.get(),
      topic_image_,
      std::bind(&imageCallback, std::placeholders::_1),
      hints.getTransport(),
      rmw_qos_profile_sensor_data,
      subscription_options);
    RCLCPP_INFO(
      logger_, "Image received: unwrapping using '%s' transport.",
      subscriber_.getTransport().c_str());
  } catch (image_transport::TransportLoadException & e) {
    RCLCPP_ERROR(
      logger_, "Failed to create subscriber for topic %s: %s",
      topic_image_.c_str(), e.what());
    return -1;
  }

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node_);
  cv::destroyWindow(transport_);

  return 0;
}

// ros2 run perception_demo main_image_transport_sub compressed
