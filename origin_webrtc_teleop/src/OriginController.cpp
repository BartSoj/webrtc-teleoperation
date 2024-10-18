#include "OriginController.hpp"

using std::placeholders::_1;

OriginController::OriginController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel_user", 10);
    control_mode_subscription_ = node_->create_subscription<origin_msgs::msg::ControlMode>(
        "/robot/cmd_vel_controller/control_mode", 10, std::bind(&OriginController::controlModeTopicCallback, this, _1));
    set_control_mode_client_ = node_->create_client<origin_msgs::srv::SetControlMode>("/robot/cmd_vel_controller/set_control_mode");
    reset_control_mode_client_ = node_->create_client<origin_msgs::srv::ReturnControlMode>("/robot/cmd_vel_controller/reset_control_mode");
    previous_control_mode_client_ = node_->create_client<origin_msgs::srv::ReturnControlMode>("/robot/cmd_vel_controller/previous_control_mode");
}

void OriginController::handleControlMessage(const std::string& message)
{
    try {
        nlohmann::json control_message = nlohmann::json::parse(message);
        if (control_message["type"] == "control") {
            bool expected = false;
            if (control_message["reset_control"] && changing_control_mode_.compare_exchange_strong(expected, true)) {
                resetControl();
            }
            else if (control_message["previous_control"] && changing_control_mode_.compare_exchange_strong(expected, true)) {
                previousControl();
            }
            else if (control_message["request_control"] && changing_control_mode_.compare_exchange_strong(expected, true)) {
                requestControl();
            }
            publishVelocity(control_message);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error handling control message: %s", e.what());
    }
}

void OriginController::requestControl()
{
    auto request = std::make_shared<origin_msgs::srv::SetControlMode::Request>();
    request->mode.mode = origin_msgs::msg::ControlMode::USER;

    while (!set_control_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }

    auto result_future = set_control_mode_client_->async_send_request(request,
        std::bind(&OriginController::handleSetControlModeResponse, this, _1));
}

void OriginController::resetControl()
{
    auto request = std::make_shared<origin_msgs::srv::ReturnControlMode::Request>();
    request->mode_from.mode = control_mode_.load();;

    while (!reset_control_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }

    auto result_future = reset_control_mode_client_->async_send_request(request,
        std::bind(&OriginController::handleReturnControlModeResponse, this, _1));
}

void OriginController::previousControl()
{
    auto request = std::make_shared<origin_msgs::srv::ReturnControlMode::Request>();
    request->mode_from.mode = control_mode_.load();

    while (!previous_control_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }

    auto result_future = previous_control_mode_client_->async_send_request(request,
        std::bind(&OriginController::handleReturnControlModeResponse, this, _1));}

void OriginController::handleSetControlModeResponse(rclcpp::Client<origin_msgs::srv::SetControlMode>::SharedFuture future)
{
    auto result = future.get();
    changing_control_mode_.store(false);
    RCLCPP_INFO(node_->get_logger(), "Set control mode service call completed");
}

void OriginController::handleReturnControlModeResponse(rclcpp::Client<origin_msgs::srv::ReturnControlMode>::SharedFuture future)
{
    auto result = future.get();
    changing_control_mode_.store(false);
    RCLCPP_INFO(node_->get_logger(), "Return control mode service call completed");
}

void OriginController::publishVelocity(const nlohmann::json& control_message)
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = control_message["forward"];
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = control_message["rotation"];

    cmd_vel_publisher_->publish(twist_msg);
}

void OriginController::controlModeTopicCallback(const origin_msgs::msg::ControlMode &msg) const
{
    RCLCPP_INFO(node_->get_logger(), "Control mode: '%d'", msg.mode);
    control_mode_.store(msg.mode);
}