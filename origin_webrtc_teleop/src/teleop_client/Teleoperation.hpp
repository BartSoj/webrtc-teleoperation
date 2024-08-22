#ifndef LIBDATACHANNEL_APP_TELEOPERATION_HPP
#define LIBDATACHANNEL_APP_TELEOPERATION_HPP

#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "PeerConnection.hpp"
#include "VideoEncoder.hpp"
#include "nlohmann/json.hpp"
#include "rtc/rtc.hpp"

using std::future;
using std::shared_ptr;
using std::weak_ptr;

class Teleoperation
{
public:
    Teleoperation(const std::string &localId, const std::string &hostname = "127.0.0.1");

    void startSignaling();

    void onChannelOpen(std::function<void()> callback);

    void onChannelClosed(std::function<void()> callback);

    void onChannelMessage(std::function<void(std::string data)> callback);

    void onChannelControlMessage(std::function<void(std::string)> callback);

    void sendMessage(const std::string &remoteId, const std::string &message);

    void broadcastMessage(const std::string &message);

    void sendVideo(const std::string &remoteId, const uint8_t *frameData, int width, int height, int step);

    void broadcastVideo(const uint8_t *frameData, int width, int height, int step);

    const std::string &getLocalId() const { return localId; }

    const rtc::Configuration &getConfig() const { return config; }

    std::shared_ptr<rtc::WebSocket> getWebSocket() const { return ws; }

    const std::unordered_map<std::string, std::shared_ptr<PeerConnection>> &getPeerConnectionMap() const
    {
        return peerConnectionMap;
    }

    void addPeerConnection(const std::string &id, std::shared_ptr<PeerConnection> pc)
    {
        peerConnectionMap.emplace(id, std::move(pc));
    }

    ~Teleoperation();

private:
    rtc::Configuration config;
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

#endif  // LIBDATACHANNEL_APP_TELEOPERATION_HPP
