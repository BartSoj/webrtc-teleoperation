#pragma once

#include <future>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "peer_connection.hpp"
#include "video_encoder.hpp"

using std::future;
using std::shared_ptr;
using std::weak_ptr;

struct TeleoperationConfig
{
    /// @brief The identifier for the teleoperation session.
    std::string localId;
    // @brief The authentication password for the teleoperation session.
    std::string auth;
    /// @brief The hostname for the WebSocket connection.
    std::string hostname = "0.0.0.0";
    /// @brief The port for the WebSocket connection.
    std::string port = "8000";
    /// @brief The STUN server url for ICE candidates.
    std::string stunServer = "stun:stun.l.google.com:19302";
    /// @brief The VideoEncoder for encoding video frames and getting the encoded packets data.
    std::shared_ptr<VideoEncoder> videoEncoder;
};

class Teleoperation
{
public:
    /**
     * @brief Creates a Teleoperation, connects to WebSocket server and configurate it.
     * @param config Configuration settings for the teleoperation.
     */
    Teleoperation(const TeleoperationConfig &config);

    /**
     * @brief Starts the signaling process by opening a WebSocket connection.
     */
    void startSignaling();

    void sendOffer(const std::string &remoteId, const std::string &access);

    /**
     * @brief Sets the callback function to be called when a data channel is opened.
     * @param callback The function to call when the channel opens.
     */
    void onChannelOpen(const std::function<void()> &callback);

    /**
     * @brief Sets the callback function to be called when a data channel is closed.
     * @param callback The function to call when the channel closes.
     */
    void onChannelClosed(const std::function<void()> &callback);

    /**
     * @brief Sets the callback function to be called when a message is received on the data channel.
     * @param callback The function to call when a message is received.
     */
    void onChannelMessage(const std::function<void(std::string data)> &callback);

    /**
     * @brief Sets the callback function to be called when a control message is received on the data channel.
     * @param callback The function to call when a control message is received.
     */
    void onChannelControlMessage(const std::function<void(std::string)> &callback);

    /**
     * @brief Sends a text message to a specific remote peer.
     * @param remoteId The identifier of the remote peer.
     * @param message The message to send.
     */
    void sendMessage(const std::string &remoteId, const std::string &message);

    /**
     * @brief Broadcasts a text message to all connected peers.
     * @param message The message to broadcast.
     */
    void broadcastMessage(const std::string &message);

    /**
     * @brief Sends video frame data to a specific remote peer.
     * @param remoteId The identifier of the remote peer.
     * @param frameData Pointer to the video frame data.
     * @param width Width of the video frame.
     * @param height Height of the video frame.
     * @param step Step size for the video frame data.
     */
    void sendVideoFrame(const std::string &remoteId, const uint8_t *frameData, int width, int height, size_t step);

    /**
     * @brief Broadcasts video frame data to all connected peers.
     * @param frameData Pointer to the video frame data.
     * @param width Width of the video frame.
     * @param height Height of the video frame.
     * @param step Step size for the video frame data.
     */
    void broadcastVideoFrame(const uint8_t *frameData, int width, int height, size_t step);

    /**
     * @brief Gets the local identifier for the teleoperation.
     * @return The local ID.
     */
    const std::string &getLocalId() const { return localId_; }

private:
    shared_ptr<PeerConnection> createPeerConnection(const std::string &remoteId, const std::string &access);

    rtc::Configuration rtcConfig_;
    std::string wsUrl_;
    shared_ptr<rtc::WebSocket> ws_;
    std::promise<void> wsPromise_;
    future<void> wsFuture_;
    std::function<void()> onChannelOpenCallback_;
    std::function<void()> onChannelClosedCallback_;
    std::function<void(std::string data)> onChannelMessageCallback_;
    std::function<void(std::string)> onChannelControlMessageCallback_;
    std::string localId_;
    std::string auth_;
    std::unordered_map<std::string, shared_ptr<PeerConnection>> peerConnectionMap_;
    shared_ptr<VideoEncoder> videoEncoder_;
};