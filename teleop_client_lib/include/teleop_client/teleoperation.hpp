#ifndef LIBDATACHANNEL_APP_TELEOPERATION_H
#define LIBDATACHANNEL_APP_TELEOPERATION_H

#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "video_encoder.hpp"
#include "nlohmann/json.hpp"
#include "peer_connection.hpp"
#include "rtc/rtc.hpp"

using std::future;
using std::shared_ptr;
using std::weak_ptr;

struct TeleoperationConfig
{
    /// @brief The identifier for the teleoperation session.
    std::string localId;
    /// @brief The hostname for the WebSocket connection.
    std::string hostname = "127.0.0.1";
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
    void sendVideoFrame(const std::string &remoteId, const uint8_t *frameData, int width, int height, int step);

    /**
     * @brief Broadcasts video frame data to all connected peers.
     * @param frameData Pointer to the video frame data.
     * @param width Width of the video frame.
     * @param height Height of the video frame.
     * @param step Step size for the video frame data.
     */
    void broadcastVideoFrame(const uint8_t *frameData, int width, int height, int step);

    /**
     * @brief Adds a new peer connection to the teleoperation session.
     * @param id The identifier for the new peer.
     * @param pc Shared pointer to the PeerConnection object.
     */
    void addPeerConnection(const std::string &id, std::shared_ptr<PeerConnection> pc);

    /**
     * @brief Gets the local identifier for the teleoperation.
     * @return The local ID.
     */
    const std::string &getLocalId() const { return localId; }

    /**
     * @brief Gets the RTC configuration used for the teleoperation session.
     * @return The RTC configuration.
     */
    const rtc::Configuration &getConfig() const { return rtcConfig; }

    /**
     * @brief Gets the WebSocket connection used for signaling.
     * @return Shared pointer to the WebSocket connection.
     */
    std::shared_ptr<rtc::WebSocket> getWebSocket() const { return ws; }

    /**
     * @brief Gets the map of peer connections.
     * @return A reference to the map of peer connections.
     */
    const std::unordered_map<std::string, std::shared_ptr<PeerConnection>> &getPeerConnectionMap() const
    {
        return peerConnectionMap;
    }

private:
    rtc::Configuration rtcConfig;
    std::string wsUrl;
    shared_ptr<rtc::WebSocket> ws;
    std::promise<void> wsPromise;
    future<void> wsFuture;
    std::function<void()> onChannelOpenCallback;
    std::function<void()> onChannelClosedCallback;
    std::function<void(std::string data)> onChannelMessageCallback;
    std::function<void(std::string)> onChannelControlMessageCallback;
    std::string localId;
    std::unordered_map<std::string, shared_ptr<PeerConnection>> peerConnectionMap;
    shared_ptr<VideoEncoder> videoEncoder;
};

#endif  // LIBDATACHANNEL_APP_TELEOPERATION_H
