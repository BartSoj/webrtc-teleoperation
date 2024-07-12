#include "PeerConnection.hpp"

#include "rtc/rtc.hpp"

#include "nlohmann/json.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <utility>
#include <thread>

#include <sys/socket.h>

using namespace std::chrono_literals;
using std::shared_ptr;
using std::weak_ptr;
using nlohmann::json;

template<class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr) { return ptr; }


PeerConnection::PeerConnection(const rtc::Configuration &config,
                               weak_ptr<rtc::WebSocket> wws, std::string localId, std::string remoteId) {
    auto pc = std::make_shared<rtc::PeerConnection>(config);
    this->localId = localId;
    this->remoteId = remoteId;

    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(96); // Must match the payload type of the external h264 RTP stream
    media.addSSRC(this->SSRC, "video-send");
    auto track = pc->addTrack(media);
    this->track = track;

    pc->onStateChange(
            [](rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

    pc->onGatheringStateChange([](rtc::PeerConnection::GatheringState state) {
        std::cout << "Gathering State: " << state << std::endl;
    });

    pc->onLocalDescription([wws, remoteId](rtc::Description description) {
        json message = {{"id",          remoteId},
                        {"type",        description.typeString()},
                        {"description", std::string(description)}};

        if (auto ws = wws.lock())
            ws->send(message.dump());
    });

    pc->onLocalCandidate([wws, remoteId](rtc::Candidate candidate) {
        json message = {{"id",        remoteId},
                        {"type",      "candidate"},
                        {"candidate", std::string(candidate)},
                        {"mid",       candidate.mid()}};

        if (auto ws = wws.lock())
            ws->send(message.dump());
    });

    pc->onDataChannel([this, remoteId](shared_ptr<rtc::DataChannel> dc) {
        std::cout << "DataChannel from " << remoteId << " received with label \"" << dc->label() << "\""
                  << std::endl;
        this->dataChannel = dc;
        configureDataChannel();
    });

    this->rtcPeerConnection = pc;
}

void PeerConnection::configureDataChannel() {
    dataChannel->onOpen([this]() {
        auto wdc = make_weak_ptr(dataChannel);
        if (auto dc = wdc.lock())
            dc->send("Hello from " + localId);
    });

    dataChannel->onClosed([this]() { std::cout << "DataChannel from " << remoteId << " closed" << std::endl; });

    dataChannel->onMessage([this](auto data) {
        // data holds either std::string or rtc::binary
        if (std::holds_alternative<std::string>(data))
            std::cout << "Message from " << remoteId << " received: " << std::get<std::string>(data)
                      << std::endl;
        else
            std::cout << "Binary message from " << remoteId
                      << " received, size=" << std::get<rtc::binary>(data).size() << std::endl;
    });
}

void PeerConnection::sendMessage(const std::string &message) {
    if (dataChannel && dataChannel->isOpen())
        this->dataChannel->send(message);
}

void PeerConnection::sendVideo(const std::byte *data, size_t len) {
    if (track != nullptr && track->isOpen())
        track->send(data, len);
}

void PeerConnection::createDataChannel() {
    dataChannel = rtcPeerConnection->createDataChannel("test");
    configureDataChannel();
    rtcPeerConnection->setLocalDescription();
}

void PeerConnection::handleConnectionMessage(const nlohmann::json &message) {
    auto it = message.find("type");
    if (it == message.end())
        return;

    auto type = it->get<std::string>();

    if (type == "offer") {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection->setRemoteDescription(rtc::Description(sdp, type));
        rtcPeerConnection->setLocalDescription();
    } else if (type == "answer") {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection->setRemoteDescription(rtc::Description(sdp, type));
    } else if (type == "candidate") {
        auto sdp = message["candidate"].get<std::string>();
        auto mid = message["mid"].get<std::string>();
        rtcPeerConnection->addRemoteCandidate(rtc::Candidate(sdp, mid));
    }
}

void PeerConnection::close() {
    track->close();
    dataChannel->close();
    rtcPeerConnection->close();
}
