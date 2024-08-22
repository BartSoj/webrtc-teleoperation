#ifndef LIBDATACHANNEL_APP_PEERCONNECTION_H
#define LIBDATACHANNEL_APP_PEERCONNECTION_H

#include <cstddef>
#include <iostream>
#include <memory>

#include "nlohmann/json.hpp"
#include "rtc/rtc.hpp"

using namespace std::chrono_literals;
using nlohmann::json;
using std::shared_ptr;
using std::weak_ptr;

class PeerConnection
{
public:
    struct ChannelCallbacks
    {
        std::function<void()> onChannelOpenCallback;
        std::function<void()> onChannelClosedCallback;
        std::function<void(std::string data)> onChannelMessageCallback;
    };

    struct Configuration
    {
        rtc::Configuration rtcConfig;
        weak_ptr<rtc::WebSocket> wws;
        std::string localId;
        std::string remoteId;
        ChannelCallbacks channelCallbacks;
        int64_t startTime;
    };
    PeerConnection(const Configuration &config);

    void sendMessage(const std::string &message);

    void sendVideo(const std::byte *data, size_t len, int64_t timestampMicro);

    uint32_t timestampMicroToRtp(uint64_t timestampMicro);

    void createDataChannel();

    void handleConnectionMessage(const json &message);

    void close();

private:
    void setDefaultCallbacks();
    void configureDataChannel();

    const rtc::SSRC SSRC = 42;
    std::string localId;
    std::string remoteId;
    ChannelCallbacks channelCallbacks;
    shared_ptr<rtc::PeerConnection> rtcPeerConnection;
    shared_ptr<rtc::DataChannel> dataChannel;
    shared_ptr<rtc::Track> track;
    shared_ptr<rtc::RtcpSrReporter> srReporter;
};

#endif  // LIBDATACHANNEL_APP_PEERCONNECTION_H
