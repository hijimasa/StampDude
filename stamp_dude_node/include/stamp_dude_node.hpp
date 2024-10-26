#ifndef STAMP_DUDE_NODE_HPP_
#define STAMP_DUDE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace stamp_dude
{
template<typename MsgType, typename StampedType>
class StampDudeNode : public rclcpp::Node
{
public:
    explicit StampDudeNode(const rclcpp::NodeOptions & options);

private:
    void callback(const typename MsgType::SharedPtr msg);

    rclcpp::Publisher<StampedType>::SharedPtr pub_;
    rclcpp::Subscription<MsgType>::SharedPtr sub_;
};

}  // namespace stamp_dude

#endif  // STAMP_DUDE_NODE_HPP_
