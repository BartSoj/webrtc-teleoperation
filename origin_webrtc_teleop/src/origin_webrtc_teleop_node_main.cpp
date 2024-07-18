#include "origin_webrtc_teleop_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OriginWebrtcTeleopNode>());
    rclcpp::shutdown();
    return 0;
}
