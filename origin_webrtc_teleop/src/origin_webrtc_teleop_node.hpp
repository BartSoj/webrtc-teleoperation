#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OriginWebrtcTeleopNode : public rclcpp::Node {
public:
    OriginWebrtcTeleopNode();

private:
    void image_topic_callback(const sensor_msgs::msg::Image &msg) const;

    void odom_topic_callback(const nav_msgs::msg::Odometry &msg) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};
