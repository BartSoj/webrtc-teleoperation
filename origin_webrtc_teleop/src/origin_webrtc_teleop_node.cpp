#include "origin_webrtc_teleop_node.hpp"

#include <memory>
#include <nlohmann/json.hpp>

#include "TeleopConfigParser.hpp"

using nlohmann::json;
using std::placeholders::_1;

OriginWebrtcTeleopNode::OriginWebrtcTeleopNode(const rclcpp::NodeOptions &options)
    : Node("origin_webrtc_teleop_node", options)
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/color/image_raw", 10, std::bind(&OriginWebrtcTeleopNode::image_topic_callback, this, _1));

    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot/odom", 10, std::bind(&OriginWebrtcTeleopNode::odom_topic_callback, this, _1));

    TeleopConfigParser teleoperationConfigParser(this);
    auto teleoperationConfig = teleoperationConfigParser.parse();
    teleoperation = std::make_shared<Teleoperation>(teleoperationConfig);

    teleoperation->onChannelOpen([this]() { RCLCPP_INFO(this->get_logger(), "Channel open"); });

    teleoperation->onChannelClosed([this]() { RCLCPP_INFO(this->get_logger(), "Channel close"); });

    teleoperation->onChannelMessage([this](const std::string &message)
                                    { RCLCPP_INFO(this->get_logger(), "Message: '%s'", message.c_str()); });

    teleoperation->onChannelControlMessage(
        [this](const std::string &message)
        { RCLCPP_INFO(this->get_logger(), "Control message: '%s'", message.c_str()); });

    teleoperation->startSignaling();
}

void OriginWebrtcTeleopNode::image_topic_callback(const sensor_msgs::msg::Image &msg) const
{
    RCLCPP_INFO(this->get_logger(), "Image: '%s'", msg.header.frame_id.c_str());
    teleoperation->broadcastVideoFrame(msg.data.data(), msg.width, msg.height, static_cast<int>(msg.step));
}

void OriginWebrtcTeleopNode::odom_topic_callback(const nav_msgs::msg::Odometry &msg) const
{
    RCLCPP_INFO(this->get_logger(), "Odometry: '%s'", msg.header.frame_id.c_str());

    json odometry_data = {
        {"type", "odometry"},
        {{"position",
          {{"x", msg.pose.pose.position.x}, {"y", msg.pose.pose.position.y}, {"z", msg.pose.pose.position.z}}},
         {"orientation",
          {{"x", msg.pose.pose.orientation.x},
           {"y", msg.pose.pose.orientation.y},
           {"z", msg.pose.pose.orientation.z},
           {"w", msg.pose.pose.orientation.w}}},
         {"linear",
          {{"x", msg.twist.twist.linear.x}, {"y", msg.twist.twist.linear.y}, {"z", msg.twist.twist.linear.z}}},
         {"angular",
          {{"x", msg.twist.twist.angular.x}, {"y", msg.twist.twist.angular.y}, {"z", msg.twist.twist.angular.z}}}}};

    teleoperation->broadcastMessage(odometry_data.dump());
}