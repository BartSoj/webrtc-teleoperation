#include "Teleoperation.hpp"

using nlohmann::json;
using std::shared_ptr;
using std::weak_ptr;

template <class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr)
{
    return ptr;
}

Teleoperation::Teleoperation(const std::string &localId, const std::string &hostname, const std::shared_ptr<VideoEncoder> &videoEncoder)
    : localId(localId), videoEncoder(videoEncoder)
{
    wsUrl = "ws://" + hostname + ":8000/" + localId;

    rtc::InitLogger(rtc::LogLevel::Info);

    std::string stunServer = "stun:stun.l.google.com:19302";
    config.iceServers.emplace_back(stunServer);

    std::cout << "The local ID is " << localId << std::endl;

    ws = std::make_shared<rtc::WebSocket>();

    wsFuture = wsPromise.get_future();

    ws->onOpen(
        [this]()
        {
            std::cout << "WebSocket connected, signaling ready" << std::endl;
            this->wsPromise.set_value();
        });

    ws->onError(
        [this](const std::string &s)
        {
            std::cout << "WebSocket error" << std::endl;
            this->wsPromise.set_exception(std::make_exception_ptr(std::runtime_error(s)));
        });

    ws->onClosed([]() { std::cout << "WebSocket closed" << std::endl; });

    ws->onMessage(
        [wws = make_weak_ptr(ws), this](auto data)
        {
            // data holds either std::string or rtc::binary
            if(!std::holds_alternative<std::string>(data)) return;

            json message = json::parse(std::get<std::string>(data));

            auto it = message.find("id");
            if(it == message.end()) return;

            auto id = it->get<std::string>();

            it = message.find("auth");
            if(it == message.end() || it->get<std::string>() != "auth0") return;

            it = message.find("access");
            if(it == message.end()) return;

            auto access = it->get<std::string>();

            it = message.find("type");
            if(it == message.end()) return;

            auto type = it->get<std::string>();

            shared_ptr<PeerConnection> pc;
            if(auto jt = peerConnectionMap.find(id); jt != peerConnectionMap.end())
            {
                pc = jt->second;
            }
            else if(type == "offer")
            {
                std::cout << "Answering to " + id << std::endl;
                PeerConnection::Configuration pcConfig;
                pcConfig.rtcConfig = config;
                pcConfig.wws = make_weak_ptr(ws);
                pcConfig.localId = this->localId;
                pcConfig.remoteId = id;
                pcConfig.channelCallbacks.onChannelOpenCallback = onChannelOpenCallback;
                pcConfig.channelCallbacks.onChannelClosedCallback = onChannelClosedCallback;
                pcConfig.startTime = this->videoEncoder->getStartTime();
                if(access == "control")
                    pcConfig.channelCallbacks.onChannelMessageCallback = onChannelControlMessageCallback;
                else
                    pcConfig.channelCallbacks.onChannelMessageCallback = onChannelMessageCallback;
                pc = std::make_shared<PeerConnection>(pcConfig);
            }
            else
            {
                return;
            }

            pc->handleConnectionMessage(message);

            this->peerConnectionMap.emplace(id, pc);
        });
}

void Teleoperation::startSignaling()
{
    std::cout << "Signaling URL is " << wsUrl << std::endl;
    ws->open(wsUrl);
    std::cout << "Waiting for signaling to be connected..." << std::endl;
    wsFuture.get();
}

void Teleoperation::onChannelOpen(std::function<void()> callback) { onChannelOpenCallback = std::move(callback); }

void Teleoperation::onChannelClosed(std::function<void()> callback) { onChannelClosedCallback = callback; }

void Teleoperation::onChannelMessage(std::function<void(std::string data)> callback)
{
    onChannelMessageCallback = callback;
}

void Teleoperation::onChannelControlMessage(std::function<void(std::string)> callback)
{
    onChannelControlMessageCallback = callback;
}

void Teleoperation::sendMessage(const std::string &remoteId, const std::string &message)
{
    if(auto it = peerConnectionMap.find(remoteId); it != peerConnectionMap.end()) it->second->sendMessage(message);
}

void Teleoperation::broadcastMessage(const std::string &message)
{
    for(auto &[id, pc] : peerConnectionMap) pc->sendMessage(message);
}

void Teleoperation::sendVideoFrame(const std::string &remoteId, const uint8_t *frameData, int width, int height,
                                   int step)
{
    videoEncoder->encodeFrame(frameData, width, height, step);
    while(videoEncoder->nextPacket())
    {
        auto data = videoEncoder->getPacketData();
        auto len = videoEncoder->getPacketSize();
        auto timestamp = videoEncoder->getElapsedTime();
        if(auto it = peerConnectionMap.find(remoteId); it != peerConnectionMap.end())
        {
            it->second->sendVideoFrame(data, len, timestamp);
        }
    }
}

void Teleoperation::broadcastVideoFrame(const uint8_t *frameData, int width, int height, int step)
{
    videoEncoder->encodeFrame(frameData, width, height, step);
    while(videoEncoder->nextPacket())
    {
        auto data = videoEncoder->getPacketData();
        auto len = videoEncoder->getPacketSize();
        auto timestamp = videoEncoder->getElapsedTime();
        for(auto &[id, pc] : peerConnectionMap)
        {
            pc->sendVideoFrame(data, len, timestamp);
        }
    }
}

void Teleoperation::addPeerConnection(const std::string &id, std::shared_ptr<PeerConnection> pc)
{
    peerConnectionMap.emplace(id, std::move(pc));
}
