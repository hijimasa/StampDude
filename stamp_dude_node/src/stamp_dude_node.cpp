#include "stamp_dude/stamp_dude_node.hpp"

namespace stamp_dude
{
template<typename MsgType, typename StampedType>
StampDudeNode<MsgType, StampedType>::StampDudeNode(const rclcpp::NodeOptions & options)
    : Node("stamp_dude_node", options)
{
    // Publisherの設定
    pub_ = this->create_publisher<StampedType>("output_" + std::string(typeid(MsgType).name()), 10);
    // Subscriberの設定
    sub_ = this->create_subscription<MsgType>(
        "input_" + std::string(typeid(MsgType).name()), 10,
        std::bind(&StampDudeNode::callback, this, std::placeholders::_1));
}

template<typename MsgType, typename StampedType>
void StampDudeNode<MsgType, StampedType>::callback(const typename MsgType::SharedPtr msg)
{
    auto stamped_msg = std::make_shared<StampedType>();
    stamped_msg->header.stamp = this->get_clock()->now();
    stamped_msg->header.frame_id = "base_link";
    stamped_msg->twist = *msg;  

    pub_->publish(*stamped_msg);
    RCLCPP_INFO(this->get_logger(), "Published Stamped Message");
}

// コンポーネントノードの登録
RCLCPP_COMPONENTS_REGISTER_NODE(TwistStampDudeNode<geometry_msgs::msg::Twist, geometry_msgs::msg::TwistStamped>)
RCLCPP_COMPONENTS_REGISTER_NODE(PoseStampDudeNode<geometry_msgs::msg::Pose, geometry_msgs::msg::PoseStamped>)

}  // namespace stamp_dude
