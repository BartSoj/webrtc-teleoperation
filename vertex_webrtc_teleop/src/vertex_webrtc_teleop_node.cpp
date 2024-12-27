#include "vertex_webrtc_teleop_node.hpp"

#include <memory>
#include <nlohmann/json.hpp>

#include "teleop_config_parser.hpp"

using nlohmann::json;
using std::placeholders::_1;

VertexWebrtcTeleopNode::VertexWebrtcTeleopNode(const rclcpp::NodeOptions &options)
    : Node("vertex_webrtc_teleop_node", options)
{
    battery_info_subscription_ = this->create_subscription<avular_mavros_msgs::msg::Battery>(
        "/robot/battery/info", 10, std::bind(&VertexWebrtcTeleopNode::batteryInfoTopicCallback, this, _1));

    set_shape_example_service_ =
        this->create_client<avular_mavros_msgs::srv::SetShapeExample>("/robot/set_shape_example");

    TeleopConfigParser teleoperationConfigParser(this);
    auto teleoperationConfig = teleoperationConfigParser.parse();
    teleoperation_ = std::make_shared<Teleoperation>(teleoperationConfig);

    teleoperation_->onChannelControlMessage(
        [this](const std::string &message)
        {
            try
            {
                json message_json = json::parse(message);
                if(message_json["type"] == "shape_example")
                {
                    this->setShapeExample(message_json["shape_json"]);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Unknown message type: %s",
                                 message_json["type"].get<std::string>().c_str());
                }
            }
            catch(const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error handling control message: %s", e.what());
            }
        });

    teleoperation_->startSignaling();
}

int VertexWebrtcTeleopNode::setShapeExample(const json &shape_json)
{
    auto request = std::make_shared<avular_mavros_msgs::srv::SetShapeExample::Request>();
    request->shape_json = shape_json.dump();

    // Check if the service is available
    if(!set_shape_example_service_->wait_for_service(std::chrono::milliseconds(200)))
    {
        RCLCPP_INFO(get_logger(), "SetShapeExample service not available");
        return -EBUSY;
    }

    // Send request
    auto result = set_shape_example_service_->async_send_request(request);

    auto status = result.wait_for(std::chrono::milliseconds(200));

    switch(status)
    {
        case std::future_status::ready:
        {
            auto response = result.get();
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "%s", response->message.c_str());
            return 0;
        }
        case std::future_status::timeout:
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "SetShapeExample request timed out!");
            return -ETIMEDOUT;
        case std::future_status::deferred:
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "SetShapeExample request deferred!");
            return -EIO;
    }
    return -EIO;
}

void VertexWebrtcTeleopNode::batteryInfoTopicCallback(const avular_mavros_msgs::msg::Battery &msg) const
{
    json battery_data = {{"type", "battery"},
                         {"battery", {{"voltage", msg.voltage}, {"state_of_charge", msg.state_of_charge}}}};

    teleoperation_->broadcastMessage(battery_data.dump());
}