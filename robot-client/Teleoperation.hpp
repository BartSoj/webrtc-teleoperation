#ifndef LIBDATACHANNEL_APP_TELEOPERATION_HPP
#define LIBDATACHANNEL_APP_TELEOPERATION_HPP

#include "PeerConnection.hpp"

#include "rtc/rtc.hpp"

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

using namespace std::chrono_literals;
using std::shared_ptr;
using std::weak_ptr;
using std::future;

class Teleoperation {
public:
    Teleoperation(const std::string &localId);

    void startSignaling();

    void sendMessage(const std::string &remoteId, const std::string &message);

    void broadcastMessage(const std::string &message);

    void streamVideoLoop();

    void sendCounterLoop();

    void addClientsLoop();

    void close();


private:
    rtc::Configuration config;
    std::string wsUrl;
    shared_ptr<rtc::WebSocket> ws;
    std::promise<void> wsPromise;
    future<void> wsFuture;
    const rtc::SSRC SSRC = 42;
    const int BUFFER_SIZE = 2048;
    std::string localId;
    std::unordered_map<std::string, shared_ptr<PeerConnection>> peerConnectionMap;
};


#endif //LIBDATACHANNEL_APP_TELEOPERATION_HPP
