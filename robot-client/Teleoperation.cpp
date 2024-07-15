#include "Teleoperation.hpp"

using std::shared_ptr;
using std::weak_ptr;
using nlohmann::json;

template<class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr) { return ptr; }


Teleoperation::Teleoperation(const std::string &localId) {
    this->localId = localId;

    wsUrl = "ws://10.10.108.20:8000/" + localId;

    rtc::InitLogger(rtc::LogLevel::Info);

    std::string stunServer = "stun:stun.l.google.com:19302";
    config.iceServers.emplace_back(stunServer);

    std::cout << "The local ID is " << localId << std::endl;

    ws = std::make_shared<rtc::WebSocket>();

    wsFuture = wsPromise.get_future();

    ws->onOpen([this]() {
        std::cout << "WebSocket connected, signaling ready" << std::endl;
        this->wsPromise.set_value();
    });

    ws->onError([this](const std::string &s) {
        std::cout << "WebSocket error" << std::endl;
        this->wsPromise.set_exception(std::make_exception_ptr(std::runtime_error(s)));
    });

    ws->onClosed([]() { std::cout << "WebSocket closed" << std::endl; });

    ws->onMessage([wws = make_weak_ptr(ws), this](auto data) {
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

        shared_ptr<PeerConnection> pc;
        if (auto jt = peerConnectionMap.find(id); jt != peerConnectionMap.end()) {
            pc = jt->second;
        } else if (type == "offer") {
            std::cout << "Answering to " + id << std::endl;
            pc = std::make_shared<PeerConnection>(config, wws, this->localId, id);
        } else {
            return;
        }

        pc->handleConnectionMessage(message);

        this->peerConnectionMap.emplace(id, pc);
    });
}

void Teleoperation::startSignaling() {
    std::cout << "Signaling URL is " << wsUrl << std::endl;
    ws->open(wsUrl);
    std::cout << "Waiting for signaling to be connected..." << std::endl;
    wsFuture.get();
}

void Teleoperation::sendMessage(const std::string &remoteId, const std::string &message) {
    if (auto it = peerConnectionMap.find(remoteId); it != peerConnectionMap.end())
        it->second->sendMessage(message);
}

void Teleoperation::broadcastMessage(const std::string &message) {
    for (auto &[id, pc]: peerConnectionMap)
        pc->sendMessage(message);
}

void Teleoperation::close() {
    for (auto &[id, pc]: peerConnectionMap)
        pc->close();
    peerConnectionMap.clear();
}
