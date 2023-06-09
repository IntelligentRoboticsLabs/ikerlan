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

#ifndef INCLUDE_OPENCV_PUB_SUB__CVSUBSCRIBER_HPP_
#define INCLUDE_OPENCV_PUB_SUB__CVSUBSCRIBER_HPP_

#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace perception_demo
{

class OpenCVPubSub : public rclcpp::Node
{
public:
  OpenCVPubSub()
  : Node("opencv_pub_sub")
  {
    // Read camera info
    subscription_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1,
      std::bind(&OpenCVPubSub::topic_callback_info, this, std::placeholders::_1));

    // Image subscription
    subscription_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_in", 1, std::bind(&OpenCVPubSub::topic_callback_image, this, std::placeholders::_1));

    // Image publisher
    publisher_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_out", rclcpp::SensorDataQoS().reliable());
  }

private:
  void topic_callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);
  void topic_callback_image(const sensor_msgs::msg::Image::ConstSharedPtr & image_in_msg) const;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
  std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_;

};

}  // namespace perception_demo

#endif  // INCLUDE_OPENCV_PUB_SUB__CVSUBSCRIBER_HPP_
