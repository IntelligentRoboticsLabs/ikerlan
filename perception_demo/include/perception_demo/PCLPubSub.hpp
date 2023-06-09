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

#ifndef INCLUDE_PCL_PUB_SUB__PCLSUBSCRIBER_HPP_
#define INCLUDE_PCL_PUB_SUB__PCLSUBSCRIBER_HPP_

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace perception_demo
{

class PCLPubSub : public rclcpp::Node
{

public:
  PCLPubSub()
  : Node("pcl_pub_sub")
  {
    // PointCloud subscription
    subscription_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_in", 1,
      std::bind(&PCLPubSub::topic_callback_pointcloud, this, std::placeholders::_1));

    // PointCloud publisher
    publisher_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud_out",
      rclcpp::SensorDataQoS().reliable());
  }

private:
  void topic_callback_pointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;

};

}  // namespace perception_demo

#endif  // INCLUDE_PCL_PUB_SUB__PCLSUBSCRIBER_HPP_
