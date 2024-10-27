#include "stamp_dude/twist_to_twist_stamped.hpp"

namespace stamp_dude
{
  TwistToTwistStamped::TwistToTwistStamped(const rclcpp::NodeOptions & options) : Node("twist_to_twist_stamped",options)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_stamped", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "twist", 10, std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1));
  }

  void TwistToTwistStamped::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto stamped_msg = geometry_msgs::msg::TwistStamped();
    stamped_msg.header.stamp = this->get_clock()->now();
    stamped_msg.twist = *msg;
    publisher_->publish(stamped_msg);
  }
} // namespace stamp_dude
