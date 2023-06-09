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

namespace perception_demo
{

void PCLPubSub::topic_callback_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg)
{
  // Check if there is any subscription to the topic
  if (publisher_pointcloud_->get_subscription_count() > 0) {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_in;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_in);

    // PCL processing ...
    pcl::PointCloud<pcl::PointXYZHSV> pointcloud_out;
    pcl::PointCloudXYZRGBtoXYZHSV(pointcloud_in, pointcloud_out);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 out_pointcloud_msg;
    pcl::toROSMsg(pointcloud_out, out_pointcloud_msg);
    out_pointcloud_msg.header = pointcloud_msg->header;

    //Publish Pointcloud
    publisher_pointcloud_->publish(out_pointcloud_msg);
  }
}

}  // namespace perception_demo
