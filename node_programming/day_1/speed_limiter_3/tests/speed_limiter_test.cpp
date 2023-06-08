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


#include "gtest/gtest.h"

#include "speed_limiter_3/SpeedLimiter.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SpeedLimiterTest : public speed_limiter_3::SpeedLimiter
{
public:
  geometry_msgs::msg::Twist
  clamp_speed_test(const geometry_msgs::msg::Twist & twist_in)
  {
    return clamp_speed(twist_in);
  }
};


TEST(speed_limiter, clamp_speed)
{
  auto speed_limiter_node = std::make_shared<SpeedLimiterTest>();
  geometry_msgs::msg::Twist in, out;

  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.0);

  in.linear.x = 0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.3);

  in.linear.x = 0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.5);

  in.linear.x = 0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.5);

  in.linear.x = -0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.3);

  in.linear.x = -0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.5);

  in.linear.x = -0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.5);

  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.0);

  in.angular.z = 0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.3);

  in.angular.z = 0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.5);

  in.angular.z = 0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.5);

  in.angular.z = -0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.3);

  in.angular.z = -0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.5);

  in.angular.z = -0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.5);
}

TEST(speed_limiter, clamp_speed_parameter)
{
  auto speed_limiter_node = std::make_shared<SpeedLimiterTest>();

  speed_limiter_node->set_parameter({"max_lin", 0.2});
  speed_limiter_node->set_parameter({"max_rot", 0.2});

  geometry_msgs::msg::Twist in, out;

  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.0);

  in.linear.x = 0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.2);

  in.linear.x = 0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.2);

  in.linear.x = 0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, 0.2);

  in.linear.x = -0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.2);

  in.linear.x = -0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.2);

  in.linear.x = -0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).linear.x, -0.2);

  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.0);

  in.angular.z = 0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.2);

  in.angular.z = 0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.2);

  in.angular.z = 0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, 0.2);

  in.angular.z = -0.3;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.2);

  in.angular.z = -0.5;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.2);

  in.angular.z = -0.8;
  ASSERT_EQ(speed_limiter_node->clamp_speed_test(in).angular.z, -0.2);
}


TEST(speed_limiter, node_functionality)
{
  auto speed_limiter_node = std::make_shared<SpeedLimiterTest>();
  geometry_msgs::msg::Twist in, out;

  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_pub = test_node->create_publisher<geometry_msgs::msg::Twist>("vel_in", 10);
  auto test_sub = test_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [&out](geometry_msgs::msg::Twist msg) {
      out = msg;
    });

  {
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, 0.0);
  }

  {
    in.linear.x = 0.3;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, 0.3);
  }

  {
    in.linear.x = 0.5;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, 0.5);
  }

  {
    in.linear.x = 0.8;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, 0.5);
  }

  {
    in.linear.x = -0.3;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, -0.3);
  }

  {
    in.linear.x = -0.5;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, -0.5);
  }

  {
    in.linear.x = -0.8;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.linear.x, -0.5);
  }

  {
    in.angular.z = 0.3;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, 0.3);
  }

  {
    in.angular.z = 0.5;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, 0.5);
  }

  {
    in.angular.z = 0.8;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, 0.5);
  }

  {
    in.angular.z = -0.3;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, -0.3);
  }

  {
    in.angular.z = -0.5;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, -0.5);
  }

  {
    in.angular.z = -0.8;
    test_pub->publish(in);

    auto start = test_node->now();
    while (rclcpp::ok() && (test_node->now() - start) < 100ms) {
      rclcpp::spin_some(test_node);
      rclcpp::spin_some(speed_limiter_node);
    }
    ASSERT_EQ(out.angular.z, -0.5);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
