#ifndef LIBDATACHANNEL_APP_TELEOPERATION_HPP
#define LIBDATACHANNEL_APP_TELEOPERATION_HPP

#include "PeerConnection.hpp"

#include "rtc/rtc.hpp"

#include "nlohmann/json.hpp"

#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <unordered_map>

using std::shared_ptr;
using std::weak_ptr;
using std::future;

class Teleoperation {
public:
    Teleoperation(const std::string &localId);

    void startSignaling();

    void onChannelOpen(std::function<void()> callback);

    void onChannelClosed(std::function<void()> callback);

    void onChannelMessage(std::function<void(std::string data)> callback);

    void onChannelControlMessage(std::function<void(std::string)> callback);

    void sendMessage(const std::string &remoteId, const std::string &message);

    void broadcastMessage(const std::string &message);

    void close();

    const std::string &getLocalId() const { return localId; }

    const rtc::Configuration &getConfig() const { return config; }

    std::shared_ptr<rtc::WebSocket> getWebSocket() const { return ws; }

    const std::unordered_map<std::string, std::shared_ptr<PeerConnection>> &
    getPeerConnectionMap() const { return peerConnectionMap; }

    void addPeerConnection(const std::string &id, std::shared_ptr<PeerConnection> pc) {
        peerConnectionMap.emplace(id, std::move(pc));
    }


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
};


#endif //LIBDATACHANNEL_APP_TELEOPERATION_HPP
