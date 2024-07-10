#include "origin_webrtc_teleop_node.hpp"

#include <memory>

using std::placeholders::_1;

OriginWebrtcTeleopNode::OriginWebrtcTeleopNode()
        : Node("origin_webrtc_teleop_node") {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10, std::bind(&OriginWebrtcTeleopNode::image_topic_callback, this, _1));
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odom", 10, std::bind(&OriginWebrtcTeleopNode::odom_topic_callback, this, _1));
}

void OriginWebrtcTeleopNode::image_topic_callback(const sensor_msgs::msg::Image &msg) const {
    RCLCPP_INFO(this->get_logger(), "Image: '%s'", msg.header.frame_id.c_str());
}

void OriginWebrtcTeleopNode::odom_topic_callback(const nav_msgs::msg::Odometry &msg) const {
    RCLCPP_INFO(this->get_logger(), "Odometry: '%s'", msg.header.frame_id.c_str());
}