#include "origin_webrtc_teleop_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OriginWebrtcTeleopNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
