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
        rtc::SSRC ssrc = 42;  // SSRC (Synchronization Source) identifier for uniquely identifying the RTP stream; can
                              // be arbitrary as there is only one media stream
        uint8_t payloadType = 96;  // Payload type 96 is a placeholder for dynamically negotiated media format
        ChannelCallbacks channelCallbacks;
        int64_t startTime;
    };

    /**
     * @brief Creates a WebRTC peer connection, sets up the video track and data channel.
     * @param config Configuration settings for the peer connection.
     */
    PeerConnection(const Configuration &config);

    /**
     * @brief Sends a text message over the data channel.
     * @param message The message to send.
     */
    void sendMessage(const std::string &message);

    /**
     * @brief Sends video data over the media track.
     * @param data Pointer to the video data.
     * @param len Length of the video data.
     * @param timestampMicro Timestamp of the video data in microseconds.
     */
    void sendVideoFrame(const std::byte *data, size_t len, int64_t timestampMicro);

    /**
     * @brief Handles incoming connection messages to establish a WebRTC peer connection.
     * @param message JSON message containing connection information.
     */
    void handleConnectionMessage(const json &message);

    ~PeerConnection();

private:
    void setDefaultCallbacks();

    void createDataChannel();

    void configureDataChannel();

    /**
     * @brief Converts a timestamp from microseconds to RTP timestamp format.
     * @param timestampMicro Timestamp in microseconds.
     * @return The equivalent RTP timestamp.
     */
    uint32_t timestampMicroToRtp(uint64_t timestampMicro);

    std::string localId;
    std::string remoteId;
    ChannelCallbacks channelCallbacks;
    shared_ptr<rtc::PeerConnection> rtcPeerConnection;
    shared_ptr<rtc::DataChannel> dataChannel;
    shared_ptr<rtc::Track> track;
    shared_ptr<rtc::RtcpSrReporter> srReporter;
};

#endif  // LIBDATACHANNEL_APP_PEERCONNECTION_H
