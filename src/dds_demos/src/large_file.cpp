#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include <utility>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class LargeFile : public rclcpp::Node
{
public:
  LargeFile()
  : Node("large_file")
  {
    image_pub_ = image_transport::create_publisher(this, "output_image", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
    pub_timer_ = create_wall_timer(
      250ms, std::bind(&LargeFile::timer_callback, this)); //20 hz
    image_path_ = ament_index_cpp::get_package_share_directory("dds_demos") / image_folder_ / image_name_;
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

