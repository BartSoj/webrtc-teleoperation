#include "Teleoperation.hpp"
#include "PeerConnection.hpp"

typedef int SOCKET;


using namespace std::chrono_literals;
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

void Teleoperation::streamVideoLoop() {
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
        rtp->setSsrc(SSRC);

        for (const auto &[id, pc]: peerConnectionMap) {
            pc->sendVideo(reinterpret_cast<const std::byte *>(buffer), len);
        }
    }
}

void Teleoperation::sendCounterLoop() {
    int counter = 0;
    while (counter < 1000) {
        std::string message = "Counter: " + std::to_string(counter);
        std::cout << "Sending message " << counter << std::endl;
        broadcastMessage(message);
        std::this_thread::sleep_for(1s);
        counter++;
    }
}

void Teleoperation::addClientsLoop() {
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
        auto pc = std::make_shared<PeerConnection>(config, ws, localId, id);

        // We are the offerer, so create a data channel to initiate the process
        const std::string label = "test";
        std::cout << "Creating DataChannel with label \"" << label << "\"" << std::endl;
        pc->createDataChannel();

        peerConnectionMap.emplace(id, pc);
    }
}

void Teleoperation::close() {
    for (auto &[id, pc]: peerConnectionMap)
        pc->close();
    peerConnectionMap.clear();
}
