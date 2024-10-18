#include "nav_msgs/msg/odometry.hpp"
#include "origin_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "teleop_client/teleoperation.hpp"

class OriginWebrtcTeleopNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs aan OriginWebrtcTeleopNode, sets the message callbacks and initializes the teleoperation.
     * @param options Options for configuring the ROS2 node.
     */
    explicit OriginWebrtcTeleopNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void startTeleoperation();
    ~OriginWebrtcTeleopNode() = default;

private:
    /**
     * @brief Callback function for processing incoming image messages.
     * @param msg The ROS2 image message.
     *
     * Broadcasts the video stream.
     */
    void imageTopicCallback(const sensor_msgs::msg::Image &msg) const;

    /**
     * @brief Callback function for processing incoming odometry messages.
     * @param msg The ROS2 odometry message.
     *
     * Broadcasts the odometry data.
     */
    void odomTopicCallback(const nav_msgs::msg::Odometry &msg) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    std::shared_ptr<Teleoperation> teleoperation_;
    std::unique_ptr<OriginController> controller_;
};
