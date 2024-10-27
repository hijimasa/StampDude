#include "stamp_dude/twist_to_twist_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

TEST(TwistToTwistStampedTest, InitializationTest)
{
  auto node = std::make_shared<stamp_dude::TwistToTwistStamped>();
  ASSERT_NE(nullptr, node);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
