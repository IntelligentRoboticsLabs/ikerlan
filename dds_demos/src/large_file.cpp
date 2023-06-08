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

#include <cv_bridge/cv_bridge.h>
#include <utility>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class LargeFile : public rclcpp::Node
{
public:
  LargeFile()
  : Node("large_file")
  {
    image_pub_ = image_transport::create_publisher(
      this, "output_image",
      rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
    // 20 hz
    pub_timer_ = create_wall_timer(
      250ms, std::bind(&LargeFile::timer_callback, this));
    image_path_ = ament_index_cpp::get_package_share_directory("dds_demos") / image_folder_ /
      image_name_;
    image_.image = cv::imread(image_path_, cv::IMREAD_COLOR);
    image_.encoding = "rgb8";
    image_.toImageMsg(image_msg_);
  }
  void timer_callback()
  {
    RCLCPP_INFO(get_logger(), "Publishing image");
    image_pub_.publish(image_msg_);
  }

private:
  sensor_msgs::msg::Image image_msg_;
  std::filesystem::path image_folder_ = "data";
  std::filesystem::path image_name_ = "test_image.jpg";
  std::filesystem::path image_path_;
//   std::string package_share_directory;
  cv_bridge::CvImage image_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto large_file_node = std::make_shared<LargeFile>();
  rclcpp::spin(large_file_node);

  rclcpp::shutdown();

  return 0;
}
