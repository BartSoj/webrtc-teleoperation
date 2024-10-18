#ifndef ORIGIN_CONTROLLER_HPP
#define ORIGIN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "origin_msgs/msg/control_mode.hpp"
#include "origin_msgs/srv/set_control_mode.hpp"
#include "origin_msgs/srv/return_control_mode.hpp"
#include <atomic>
#include <nlohmann/json.hpp>

class OriginController {
public:
    static constexpr double MAX_VELOCITY = 2.0;

    /**
     * @brief Constructor for OriginController for controlling robot using commands.
     * @param node Shared pointer to the ROS node for creating publishers and clients.
     */
    OriginController(rclcpp::Node::SharedPtr node);

    /**
     * @brief Handles incoming control messages.
     * @param message JSON-formatted string containing control commands.
     *
     * This function parses the incoming message and calls the appropriate functions to control the robot.
     */
    void handleControlMessage(const std::string& message);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<origin_msgs::msg::ControlMode>::SharedPtr control_mode_subscription_;
    rclcpp::Client<origin_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;
    rclcpp::Client<origin_msgs::srv::ReturnControlMode>::SharedPtr reset_control_mode_client_;
    rclcpp::Client<origin_msgs::srv::ReturnControlMode>::SharedPtr previous_control_mode_client_;
    std::atomic<bool> changing_control_mode_{false};
    std::atomic<uint8_t> control_mode_{0};

    /**
     * @brief Requests control mode change to user control.
     *
     * This function sends a service request to set the control mode to user control.
     */
    void requestControl();

    /**
     * @brief Resets the control mode.
     *
     * This function sends a service request to reset the control mode.
     */
    void resetControl();

    /**
     * @brief Switches to the previous control mode.
     *
     * This function sends a service request to return to the previous control mode.
     */
    void previousControl();

    /**
     * @brief Updates current control mode.
     * @param msg The Origin control mode message.
     *
     * Sets the current control mode to the one received from message.
     */
    void controlModeTopicCallback(const origin_msgs::msg::ControlMode &msg);

    /**
     * @brief Publishes velocity commands.
     * @param control_message JSON object containing velocity command data.
     *
     * This function extracts linear and angular velocity components from the control_message
     * and publishes them as a Twist message.
     */
    void publishVelocity(const nlohmann::json& control_message);

    void handleSetControlModeResponse(rclcpp::Client<origin_msgs::srv::SetControlMode>::SharedFuture future);
    void handleReturnControlModeResponse(rclcpp::Client<origin_msgs::srv::ReturnControlMode>::SharedFuture future);
};

#endif // ORIGIN_CONTROLLER_HPP