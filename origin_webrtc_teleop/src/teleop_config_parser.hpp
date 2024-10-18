#pragma once

#include <rclcpp/rclcpp.hpp>

#include "teleop_client/video_encoder.hpp"
#include "teleop_client/teleoperation.hpp"

class TeleopConfigParser
{
public:
    /**
     * @brief Constructor for TeleopConfigParser for parsing ROS params and creating TeleoperationConfig.
     * @param node Pointer to the ROS node for reading ROS params.
     */
    TeleopConfigParser(rclcpp::Node* node);

    /**
     * @brief Parses the teleoperation configuration from ROS parameters.
     * @return TeleoperationConfig object containing the parsed configuration.
     *
     * Possible ROS parameters:
     * - local_id (string): Local identifier for the teleoperation client.
     * - hostname (string): Hostname for the teleoperation server.
     * - port (int): Port number for the teleoperation server.
     * - stun_server (string): STUN server address for NAT traversal.
     */
    TeleoperationConfig parse();

private:
    rclcpp::Node* node_;

    /**
     * @brief Parses the video encoder configuration from ROS parameters.
     * @return VideoEncoderConfig object containing the parsed configuration.
     *
     * Possible ROS parameters:
     * - video_encoder.bit_rate (int): Bit rate for video encoding.
     * - video_encoder.width (int): Width of the video frame.
     * - video_encoder.height (int): Height of the video frame.
     * - video_encoder.framerate (int): Frame rate of the video.
     * - video_encoder.gop_size (int): Group of Pictures (GOP) size.
     * - video_encoder.max_b_frames (int): Maximum number of B-frames between I-frames.
     * - video_encoder.refs (int): Number of reference frames.
     */
    VideoEncoderConfig parseVideoEncoderConfig();

    /**
     * @brief Parses additional video encoder options from ROS parameters.
     * @return Map of option name to option value.
     *
     * Possible ROS parameters:
     * - video_encoder.options.* : Any additional encoder-specific options.
     *   Example: video_encoder.options.preset, video_encoder.options.tune, etc.
     */
    std::map<std::string, std::string> parseVideoEncoderOptions();

    /**
     * @brief Gets a parameter value from ROS parameters.
     * @tparam T Type of the parameter value.
     * @param name Name of the parameter.
     * @param default_value Default value to use if the parameter is not set.
     * @return The parameter value of type T.
     */
    template <typename T>
    T getParameter(const std::string& name, const T& default_value);
};