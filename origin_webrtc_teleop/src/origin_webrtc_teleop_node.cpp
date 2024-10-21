#include "origin_webrtc_teleop_node.hpp"

#include <memory>
#include <nlohmann/json.hpp>

#include "teleop_config_parser.hpp"

using nlohmann::json;
using std::placeholders::_1;

OriginWebrtcTeleopNode::OriginWebrtcTeleopNode(const rclcpp::NodeOptions &options)
    : Node("origin_webrtc_teleop_node", options)
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/robot/camera/color/image_raw", 10, std::bind(&OriginWebrtcTeleopNode::imageTopicCallback, this, _1));

    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot/odom", 10, std::bind(&OriginWebrtcTeleopNode::odomTopicCallback, this, _1));

    battery_info_subscription_ = this->create_subscription<origin_msgs::msg::BatteryInfo>(
        "/robot/battery/info", 10, std::bind(&OriginWebrtcTeleopNode::batteryInfoTopicCallback, this, _1));

    network_telemetry_subscription_ = this->create_subscription<origin_msgs::msg::NetworkTelemetry>(
        "/robot/origin_network_telemetry/network", 10,
        std::bind(&OriginWebrtcTeleopNode::networkTelemetryTopicCallback, this, _1));
}

void OriginWebrtcTeleopNode::startTeleoperation()
{
    controller_ = std::make_unique<OriginController>(shared_from_this());

    TeleopConfigParser teleoperationConfigParser(this);
    auto teleoperationConfig = teleoperationConfigParser.parse();
    teleoperation_ = std::make_shared<Teleoperation>(teleoperationConfig);

    teleoperation_->onChannelOpen([this]() { RCLCPP_INFO(this->get_logger(), "Channel open"); });

    teleoperation_->onChannelClosed([this]() { RCLCPP_INFO(this->get_logger(), "Channel close"); });

    teleoperation_->onChannelMessage([this](const std::string &message)
                                     { RCLCPP_INFO(this->get_logger(), "Message: '%s'", message.c_str()); });

    teleoperation_->onChannelControlMessage(
        [this](const std::string &message)
        {
            controller_->handleControlMessage(message);
        });

    teleoperation_->startSignaling();
}

void OriginWebrtcTeleopNode::imageTopicCallback(const sensor_msgs::msg::Image &msg) const
{
    teleoperation_->broadcastVideoFrame(msg.data.data(), msg.width, msg.height, static_cast<int>(msg.step));
}

void OriginWebrtcTeleopNode::odomTopicCallback(const nav_msgs::msg::Odometry &msg) const
{
    json odometry_data = {
        {"type", "odometry"},
        {"odometry",
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
           {{"x", msg.twist.twist.angular.x}, {"y", msg.twist.twist.angular.y}, {"z", msg.twist.twist.angular.z}}}}}};

    teleoperation_->broadcastMessage(odometry_data.dump());
}

void OriginWebrtcTeleopNode::batteryInfoTopicCallback(const origin_msgs::msg::BatteryInfo &msg) const
{
    json battery_data = {{"type", "battery"},
                         {"battery", {{"voltage", msg.voltage}, {"state_of_charge", msg.state_of_charge}}}};

    teleoperation_->broadcastMessage(battery_data.dump());
}

void OriginWebrtcTeleopNode::networkTelemetryTopicCallback(const origin_msgs::msg::NetworkTelemetry &msg) const
{
    json network_telemetry_data = {
        {"type", "network"},
        {"network", {{"cellular_strength", msg.cellular_strength}, {"wifi_strength", msg.wifi_strength}}}};

    teleoperation_->broadcastMessage(network_telemetry_data.dump());
}