#pragma once

#include <avular_mavros_msgs/msg/battery.hpp>
#include <avular_mavros_msgs/msg/state_reference.hpp>
#include <avular_mavros_msgs/srv/set_shape_example.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop_client/teleoperation.hpp"

class VertexWebrtcTeleopNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs aan VertexWebrtcTeleopNode, sets the message callbacks and initializes the teleoperation.
     * @param options Options for configuring the ROS2 node.
     */
    explicit VertexWebrtcTeleopNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~VertexWebrtcTeleopNode() override = default;

private:
    /**
     * @brief Sends a shape execution request to the SetShapeExample service.
     * @param shape_json JSON object containing the shape data.
     *
     * @return 0 on success, -EBUSY if the service is unavailable,
     *         -ETIMEDOUT on timeout, or -EIO on other errors.
     */
    int setShapeExample(const json &shape_json);

    /**
     * @brief Callback function for processing incoming battery info messages.
     * @param msg The vertex battery info message.
     *
     * Broadcasts the battery info.
     */
    void batteryInfoTopicCallback(const avular_mavros_msgs::msg::Battery &msg) const;

    /**
     * @brief Callback function for processing incoming state reference messages.
     * @param msg The vertex state reference message.
     *
     * Broadcasts the state reference.
     */
    void stateReferenceTopicCallback(const avular_mavros_msgs::msg::StateReference &msg) const;

    rclcpp::Client<avular_mavros_msgs::srv::SetShapeExample>::SharedPtr set_shape_example_service_;
    rclcpp::Subscription<avular_mavros_msgs::msg::Battery>::SharedPtr battery_info_subscription_;
    rclcpp::Subscription<avular_mavros_msgs::msg::StateReference>::SharedPtr state_reference_subscription_;
    std::shared_ptr<Teleoperation> teleoperation_;
};
