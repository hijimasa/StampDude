#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace stamp_dude
{
  class TwistToTwistStamped : public rclcpp::Node
  {
  public:
    explicit TwistToTwistStamped(const rclcpp::NodeOptions & options);

  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  };
} // namespace stamp_dude

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude::TwistToTwistStamped)