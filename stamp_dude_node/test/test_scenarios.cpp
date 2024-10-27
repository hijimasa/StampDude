#include "stamp_dude/twist_to_twist_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

TEST(TwistToTwistStampedScenario, MessageConversionTest)
{
  auto node = std::make_shared<stamp_dude::TwistToTwistStamped>();

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 1.0;
  twist_msg.angular.z = 0.5;

  node->twist_callback(std::make_shared<geometry_msgs::msg::Twist>(twist_msg));
  // add validation steps here
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
