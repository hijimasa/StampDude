#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>


#include <chrono>

namespace stamp_dude_node {

template <typename UnstampedT, typename StampedT>
class StampConverter : public rclcpp::Node {
public:
  explicit StampConverter(const rclcpp::NodeOptions & options)
  : Node("stamp_converter", options)
  {
    unstamped_sub_ = this->create_subscription<UnstampedT>(
      "input", 10,
      std::bind(&StampConverter::convert_and_publish, this, std::placeholders::_1));

    stamped_pub_ = this->create_publisher<StampedT>("output", 10);
  }

private:
  typename rclcpp::Subscription<UnstampedT>::SharedPtr unstamped_sub_;
  typename rclcpp::Publisher<StampedT>::SharedPtr stamped_pub_;

  void convert_and_publish(const typename UnstampedT::SharedPtr msg)
  {
    StampedT stamped_msg;
    stamped_msg.header.stamp = this->now();
    stamped_msg.data = msg->data;  // メンバは型に合わせて調整

    stamped_pub_->publish(stamped_msg);
  }
};

using AccelToAccelStamped = StampConverter<geometry_msgs::msg::Accel, geometry_msgs::msg::AccelStamped>;
using AccelWithCovarianceToAccelWithCovarianceStamped = StampConverter<geometry_msgs::msg::AccelWithCovariance, geometry_msgs::msg::AccelWithCovarianceStamped>;
using InertiaToInertiaStamped = StampConverter<geometry_msgs::msg::Inertia, geometry_msgs::msg::InertiaStamped>;
using PointToPointStamped = StampConverter<geometry_msgs::msg::Point, geometry_msgs::msg::PointStamped>;
using PolygonToPolygonStamped = StampConverter<geometry_msgs::msg::Polygon, geometry_msgs::msg::PolygonStamped>;
using PoseToPoseStamped = StampConverter<geometry_msgs::msg::Pose, geometry_msgs::msg::PoseStamped>;
using PoseWithCovarianceToPoseWithCovarianceStamped = StampConverter<geometry_msgs::msg::PoseWithCovariance, geometry_msgs::msg::PoseWithCovarianceStamped>;
using QuaternionToQuaternionStamped = StampConverter<geometry_msgs::msg::Quaternion, geometry_msgs::msg::QuaternionStamped>;
using TransformToTransformStamped = StampConverter<geometry_msgs::msg::Transform, geometry_msgs::msg::TransformStamped>;
using TwistToTwistStamped = StampConverter<geometry_msgs::msg::Twist, geometry_msgs::msg::TwistStamped>;
using TwistWithCovarianceToTwistWithCovarianceStamped = StampConverter<geometry_msgs::msg::TwistWithCovariance, geometry_msgs::msg::TwistWithCovarianceStamped>;
using Vector3ToVector3Stamped = StampConverter<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3Stamped>;
using WrenchToWrenchStamped = StampConverter<geometry_msgs::msg::Wrench, geometry_msgs::msg::WrenchStamped>;


}  // namespace stamp_dude_node

RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::AccelToAccelStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::AccelWithCovarianceToAccelWithCovarianceStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::InertiaToInertiaStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::PointToPointStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::PolygonToPolygonStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::PoseToPoseStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::PoseWithCovarianceToPoseWithCovarianceStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::QuaternionToQuaternionStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::TransformToTransformStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::TwistToTwistStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::TwistWithCovarianceToTwistWithCovarianceStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::Vector3ToVector3Stamped)
RCLCPP_COMPONENTS_REGISTER_NODE(stamp_dude_node::WrenchToWrenchStamped)
