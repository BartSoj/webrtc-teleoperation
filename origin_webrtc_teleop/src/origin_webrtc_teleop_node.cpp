#include "origin_webrtc_teleop_node.hpp"

#include <memory>

using std::placeholders::_1;

OriginWebrtcTeleopNode::OriginWebrtcTeleopNode()
        : Node("origin_webrtc_teleop_node") {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10, std::bind(&OriginWebrtcTeleopNode::image_topic_callback, this, _1));
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odom", 10, std::bind(&OriginWebrtcTeleopNode::odom_topic_callback, this, _1));
    teleoperation = std::make_shared<Teleoperation>("origin-1");

    teleoperation->onChannelOpen([this]() {
        RCLCPP_INFO(this->get_logger(), "Channel open");
    });

    teleoperation->onChannelClosed([this]() {
        RCLCPP_INFO(this->get_logger(), "Channel close");
    });

    teleoperation->onChannelMessage([this](const std::string &message) {
        RCLCPP_INFO(this->get_logger(), "Message: '%s'", message.c_str());
    });

    teleoperation->onChannelControlMessage([this](const std::string &message) {
        RCLCPP_INFO(this->get_logger(), "Control message: '%s'", message.c_str());
    });

    teleoperation->startSignaling();
}

OriginWebrtcTeleopNode::~OriginWebrtcTeleopNode() {
    teleoperation->close();
}

void OriginWebrtcTeleopNode::image_topic_callback(const sensor_msgs::msg::Image &msg) const {
    RCLCPP_INFO(this->get_logger(), "Image: '%s'", msg.header.frame_id.c_str());
}

void OriginWebrtcTeleopNode::odom_topic_callback(const nav_msgs::msg::Odometry &msg) const {
    RCLCPP_INFO(this->get_logger(), "Odometry: '%s'", msg.header.frame_id.c_str());
    std::string odometry_data = "{"
                                "\"position\": {"
                                "\"x\": " + std::to_string(msg.pose.pose.position.x) + ", "
                                                                                       "\"y\": " +
                                std::to_string(msg.pose.pose.position.y) + ", "
                                                                           "\"z\": " +
                                std::to_string(msg.pose.pose.position.z) + "}, "
                                                                           "\"orientation\": {"
                                                                           "\"x\": " +
                                std::to_string(msg.pose.pose.orientation.x) + ", "
                                                                              "\"y\": " +
                                std::to_string(msg.pose.pose.orientation.y) + ", "
                                                                              "\"z\": " +
                                std::to_string(msg.pose.pose.orientation.z) + ", "
                                                                              "\"w\": " +
                                std::to_string(msg.pose.pose.orientation.w) + "}, "
                                                                              "\"linear\": {"
                                                                              "\"x\": " +
                                std::to_string(msg.twist.twist.linear.x) + ", "
                                                                           "\"y\": " +
                                std::to_string(msg.twist.twist.linear.y) + ", "
                                                                           "\"z\": " +
                                std::to_string(msg.twist.twist.linear.z) + "}, "
                                                                           "\"angular\": {"
                                                                           "\"x\": " +
                                std::to_string(msg.twist.twist.angular.x) + ", "
                                                                            "\"y\": " +
                                std::to_string(msg.twist.twist.angular.y) + ", "
                                                                            "\"z\": " +
                                std::to_string(msg.twist.twist.angular.z) + "}"
                                                                            "}";

    teleoperation->broadcastMessage(odometry_data);
}