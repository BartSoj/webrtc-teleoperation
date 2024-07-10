/**
 * libdatachannel client example
 * Copyright (c) 2019-2020 Paul-Louis Ageneau
 * Copyright (c) 2019 Murat Dogan
 * Copyright (c) 2020 Will Munn
 * Copyright (c) 2020 Nico Chatzi
 * Copyright (c) 2020 Lara Mackey
 * Copyright (c) 2020 Erik Cota-Robles
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

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

template<class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr) { return ptr; }

typedef int SOCKET;

using nlohmann::json;

const int BUFFER_SIZE = 2048;
const rtc::SSRC ssrc = 42;

std::string localId;
std::unordered_map<std::string, shared_ptr<rtc::PeerConnection>> peerConnectionMap;
std::unordered_map<std::string, shared_ptr<rtc::DataChannel>> dataChannelMap;
std::unordered_map<std::string, shared_ptr<rtc::Track>> trackMap;

shared_ptr<rtc::PeerConnection> createPeerConnection(const rtc::Configuration &config,
                                                     weak_ptr<rtc::WebSocket> wws, std::string id);

void streamVideoLoop();

void sendCounterLoop();

void addClientsLoop(const rtc::Configuration &config, shared_ptr<rtc::WebSocket> ws);

void broadcastMessage(const std::string &message);

std::string randomId(size_t length);

int main(int argc, char **argv) try {
    rtc::InitLogger(rtc::LogLevel::Info);

    rtc::Configuration config;
    std::string stunServer = "stun:stun.l.google.com:19302";
    std::cout << "STUN server is " << stunServer << std::endl;
    config.iceServers.emplace_back(stunServer);

    localId = randomId(4);
    std::cout << "The local ID is " << localId << std::endl;

    auto ws = std::make_shared<rtc::WebSocket>();

    std::promise<void> wsPromise;
    auto wsFuture = wsPromise.get_future();

    ws->onOpen([&wsPromise]() {
        std::cout << "WebSocket connected, signaling ready" << std::endl;
        wsPromise.set_value();
    });

    ws->onError([&wsPromise](std::string s) {
        std::cout << "WebSocket error" << std::endl;
        wsPromise.set_exception(std::make_exception_ptr(std::runtime_error(s)));
    });

    ws->onClosed([]() { std::cout << "WebSocket closed" << std::endl; });

    ws->onMessage([&config, wws = make_weak_ptr(ws)](auto data) {
        // data holds either std::string or rtc::binary
        if (!std::holds_alternative<std::string>(data))
            return;

        json message = json::parse(std::get<std::string>(data));

        auto it = message.find("id");
        if (it == message.end())
            return;

        auto id = it->get<std::string>();

        it = message.find("type");
        if (it == message.end())
            return;

        auto type = it->get<std::string>();

        shared_ptr<rtc::PeerConnection> pc;
        if (auto jt = peerConnectionMap.find(id); jt != peerConnectionMap.end()) {
            pc = jt->second;
        } else if (type == "offer") {
            std::cout << "Answering to " + id << std::endl;
            pc = createPeerConnection(config, wws, id);
        } else {
            return;
        }

        if (type == "offer") {
            auto sdp = message["description"].get<std::string>();
            pc->setRemoteDescription(rtc::Description(sdp, type));
            pc->setLocalDescription();
        } else if (type == "answer") {
            auto sdp = message["description"].get<std::string>();
            pc->setRemoteDescription(rtc::Description(sdp, type));
        } else if (type == "candidate") {
            auto sdp = message["candidate"].get<std::string>();
            auto mid = message["mid"].get<std::string>();
            pc->addRemoteCandidate(rtc::Candidate(sdp, mid));
        }
    });

    const std::string url = "ws://10.10.108.20:8000/" + localId; // TODO: Change to signaling server URL

    std::cout << "WebSocket URL is " << url << std::endl;
    ws->open(url);

    std::cout << "Waiting for signaling to be connected..." << std::endl;
    wsFuture.get();

    std::thread streamVideoThread(streamVideoLoop);
    std::thread sendCounterThread(sendCounterLoop);
    std::thread addClientsThread(addClientsLoop, config, ws);

    streamVideoThread.join();
    sendCounterThread.join();
    addClientsThread.join();

    std::cout << "Cleaning up..." << std::endl;

    dataChannelMap.clear();
    peerConnectionMap.clear();
    return 0;

} catch (const std::exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
    dataChannelMap.clear();
    peerConnectionMap.clear();
    return -1;
}

// Create and setup a PeerConnection
shared_ptr<rtc::PeerConnection> createPeerConnection(const rtc::Configuration &config,
                                                     weak_ptr<rtc::WebSocket> wws, std::string id) {
    auto pc = std::make_shared<rtc::PeerConnection>(config);

    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(96); // Must match the payload type of the external h264 RTP stream
    media.addSSRC(ssrc, "video-send");
    auto track = pc->addTrack(media);
    trackMap.emplace(id, track);

    pc->onStateChange(
            [](rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

    pc->onGatheringStateChange([](rtc::PeerConnection::GatheringState state) {
        std::cout << "Gathering State: " << state << std::endl;
    });

    pc->onLocalDescription([wws, id](rtc::Description description) {
        json message = {{"id",          id},
                        {"type",        description.typeString()},
                        {"description", std::string(description)}};

        if (auto ws = wws.lock())
            ws->send(message.dump());
    });

    pc->onLocalCandidate([wws, id](rtc::Candidate candidate) {
        json message = {{"id",        id},
                        {"type",      "candidate"},
                        {"candidate", std::string(candidate)},
                        {"mid",       candidate.mid()}};

        if (auto ws = wws.lock())
            ws->send(message.dump());
    });

    pc->onDataChannel([id](shared_ptr<rtc::DataChannel> dc) {
        std::cout << "DataChannel from " << id << " received with label \"" << dc->label() << "\""
                  << std::endl;

        dc->onOpen([wdc = make_weak_ptr(dc)]() {
            if (auto dc = wdc.lock())
                dc->send("Hello from " + localId);
        });

        dc->onClosed([id]() { std::cout << "DataChannel from " << id << " closed" << std::endl; });

        dc->onMessage([id](auto data) {
            // data holds either std::string or rtc::binary
            if (std::holds_alternative<std::string>(data))
                std::cout << "Message from " << id << " received: " << std::get<std::string>(data)
                          << std::endl;
            else
                std::cout << "Binary message from " << id
                          << " received, size=" << std::get<rtc::binary>(data).size() << std::endl;
        });

        dataChannelMap.emplace(id, dc);
    });

    peerConnectionMap.emplace(id, pc);
    return pc;
};

void streamVideoLoop() {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(6000);

    if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("Failed to bind UDP socket on 127.0.0.1:6000");

    int rcvBufSize = 212992;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&rcvBufSize),
               sizeof(rcvBufSize));

    // Receive from UDP
    char buffer[BUFFER_SIZE];
    int len;
    while ((len = recv(sock, buffer, BUFFER_SIZE, 0)) >= 0) {
        if (len < sizeof(rtc::RtpHeader))
            continue;

        auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
        rtp->setSsrc(ssrc);

        for (const auto &[id, track]: trackMap) {
            if (track != nullptr && track->isOpen()) {
                track->send(reinterpret_cast<const std::byte *>(buffer), len);
            }
        }
    }
}

void sendCounterLoop() {
    int counter = 0;
    while (true) {
        std::string message = "Counter: " + std::to_string(counter);
        std::cout << "Sending message " << counter << std::endl;
        broadcastMessage(message);
        std::this_thread::sleep_for(1s);
        counter++;
    }
}

void addClientsLoop(const rtc::Configuration &config, shared_ptr<rtc::WebSocket> ws) {
    while (true) {
        std::string id;
        std::cout << "Enter a remote ID to send an offer:" << std::endl;
        std::cin >> id;
        std::cin.ignore();

        if (id.empty())
            break;

        if (id == localId) {
            std::cout << "Invalid remote ID (This is the local ID)" << std::endl;
            continue;
        }

        std::cout << "Offering to " + id << std::endl;
        auto pc = createPeerConnection(config, ws, id);

        // We are the offerer, so create a data channel to initiate the process
        const std::string label = "test";
        std::cout << "Creating DataChannel with label \"" << label << "\"" << std::endl;
        auto dc = pc->createDataChannel(label);

        dc->onOpen([id, wdc = make_weak_ptr(dc)]() {
            std::cout << "DataChannel from " << id << " open" << std::endl;
            if (auto dc = wdc.lock())
                dc->send("Hello from " + localId);
        });

        dc->onClosed([id]() { std::cout << "DataChannel from " << id << " closed" << std::endl; });

        dc->onMessage([id, wdc = make_weak_ptr(dc)](auto data) {
            // data holds either std::string or rtc::binary
            if (std::holds_alternative<std::string>(data))
                std::cout << "Message from " << id << " received: " << std::get<std::string>(data)
                          << std::endl;
            else
                std::cout << "Binary message from " << id
                          << " received, size=" << std::get<rtc::binary>(data).size() << std::endl;
        });

        dataChannelMap.emplace(id, dc);

        pc->setLocalDescription();
    }
}

void broadcastMessage(const std::string &message) {
    for (const auto &[id, dc]: dataChannelMap) {
        if (dc->isOpen()) {
            dc->send(message);
        }
    }
}

// Helper function to generate a random ID
std::string randomId(size_t length) {
    using std::chrono::high_resolution_clock;
    static thread_local std::mt19937 rng(
            static_cast<unsigned int>(high_resolution_clock::now().time_since_epoch().count()));
    static const std::string characters(
            "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz");
    std::string id(length, '0');
    std::uniform_int_distribution<int> uniform(0, int(characters.size() - 1));
    std::generate(id.begin(), id.end(), [&]() { return characters.at(uniform(rng)); });
    return id;
}
