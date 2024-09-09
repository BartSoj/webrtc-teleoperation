#include "TeleopConfigParser.hpp"

TeleopConfigParser::TeleopConfigParser(rclcpp::Node* node) : node_(node) {}

TeleoperationConfig TeleopConfigParser::parse()
{
    TeleoperationConfig config;

    config.localId = getParameter<std::string>("local_id", "origin");
    config.hostname = getParameter<std::string>("hostname", config.hostname);
    config.port = getParameter<std::string>("port", config.port);
    config.stunServer = getParameter<std::string>("stun_server", config.stunServer);

    RCLCPP_INFO(node_->get_logger(), "TeleoperationConfig: localId=%s, hostname=%s, port=%s, stunServer=%s",
                config.localId.c_str(), config.hostname.c_str(), config.port.c_str(), config.stunServer.c_str());

    auto encoderConfig = parseVideoEncoderConfig();
    config.videoEncoder = std::make_shared<VideoEncoder>(encoderConfig);

    return config;
}

VideoEncoderConfig TeleopConfigParser::parseVideoEncoderConfig()
{
    VideoEncoderConfig config;

    config.bit_rate = getParameter<int64_t>("video_encoder.bit_rate", config.bit_rate);
    config.width = getParameter<int>("video_encoder.width", config.width);
    config.height = getParameter<int>("video_encoder.height", config.height);
    config.time_base = {1, getParameter<int>("video_encoder.framerate", config.time_base.den)};
    config.gop_size = getParameter<int>("video_encoder.gop_size", config.gop_size);
    config.max_b_frames = getParameter<int>("video_encoder.max_b_frames", config.max_b_frames);
    config.refs = getParameter<int>("video_encoder.refs", config.refs);
    config.options = parseVideoEncoderOptions();

    std::string options_str;
    for(const auto& pair : config.options)
    {
        options_str += pair.first + "=" + pair.second + ", ";
    }
    if(!options_str.empty())
    {
        options_str.pop_back();  // Remove the trailing comma
        options_str.pop_back();  // Remove the trailing space
    }

    RCLCPP_INFO(node_->get_logger(),
                "VideoEncoderConfig: bit_rate=%ld, width=%d, height=%d, time_base=%d/%d, gop_size=%d, max_b_frames=%d, "
                "refs=%d, options={%s}",
                config.bit_rate, config.width, config.height, config.time_base.num, config.time_base.den,
                config.gop_size, config.max_b_frames, config.refs, options_str.c_str());

    return config;
}

std::map<std::string, std::string> TeleopConfigParser::parseVideoEncoderOptions()
{
    std::map<std::string, std::string> options;

    auto parameters = node_->list_parameters({"video_encoder.options"}, 1);

    for(const auto& param_name : parameters.names)
    {
        std::string option_name = param_name.substr(param_name.find_last_of('.') + 1);
        std::string option_value = node_->get_parameter(param_name).as_string();
        options[option_name] = option_value;
    }

    return options;
}

template <typename T>
T TeleopConfigParser::getParameter(const std::string& name, const T& default_value)
{
    return node_->get_parameter_or(name, rclcpp::ParameterValue(default_value)).get<T>();
}