#ifndef LIBDATACHANNEL_APP_PEERCONNECTION_H
#define LIBDATACHANNEL_APP_PEERCONNECTION_H

#include "rtc/rtc.hpp"

#include "nlohmann/json.hpp"

#include <cstddef>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <utility>
#include <thread>
#include <unordered_map>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

using namespace std::chrono_literals;
using std::shared_ptr;
using std::weak_ptr;
using nlohmann::json;


class PeerConnection {
public:
    PeerConnection(const rtc::Configuration &config,
                   weak_ptr<rtc::WebSocket> wws, std::string localId, std::string remoteId);

    void sendMessage(const std::string &message);

    void sendVideo(const std::byte *data, size_t len);

    void createDataChannel();

    void handleConnectionMessage(const json &message);

    void close();

private:
    void configureDataChannel();

    const rtc::SSRC SSRC = 42;
    std::string localId;
    std::string remoteId;
    shared_ptr<rtc::PeerConnection> rtcPeerConnection;
    shared_ptr<rtc::DataChannel> dataChannel;
    shared_ptr<rtc::Track> track;
};


#endif //LIBDATACHANNEL_APP_PEERCONNECTION_H
