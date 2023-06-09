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

#include "perception_demo/OpenCVPubSub.hpp"

namespace perception_demo
{

void OpenCVPubSub::topic_callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
{
  RCLCPP_INFO(get_logger(), "Camera info received");
  camera_model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  camera_model_->fromCameraInfo(*msg);
  subscription_info_ = nullptr;
}

void OpenCVPubSub::topic_callback_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_in_msg) const
{
  // Check if camera model has been received
  if (camera_model_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Camera Model not yet available");
    return;
  }

  // Check if there is any subscription to the topic
  if (publisher_image_->get_subscription_count() > 0) {
    // Convert ROS Image to OpenCV Image
    cv_bridge::CvImagePtr image_in_ptr;
    try {
      image_in_ptr = cv_bridge::toCvCopy(*image_in_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {

      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Get OpenCV Image
    cv::Mat image_in = image_in_ptr->image;

    // OpenCV processing ...
    cv::Mat image_out;
    cv::cvtColor(image_in, image_out, cv::COLOR_BGR2HSV);

    // Convert OpenCV Image to ROS Image
    cv_bridge::CvImage image_out_bridge =
      cv_bridge::CvImage(image_in_msg->header, sensor_msgs::image_encodings::BGR8, image_out);

    // from cv_bridge to sensor_msgs::Image
    sensor_msgs::msg::Image image_out_msg;
    image_out_bridge.toImageMsg(image_out_msg);

    //Publish image
    publisher_image_->publish(image_out_msg);
  }
}

}  // namespace perception_demo
