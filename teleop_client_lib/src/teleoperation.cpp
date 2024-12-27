#include "teleop_client/teleoperation.hpp"

using nlohmann::json;
using std::shared_ptr;
using std::weak_ptr;

template <class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr)
{
    return ptr;
}

Teleoperation::Teleoperation(const TeleoperationConfig &config)
    : localId_(config.localId), auth_(config.auth), videoEncoder_(config.videoEncoder)
{
    wsUrl_ = "ws://" + config.hostname + ":" + config.port + "/" + localId_;

    rtc::InitLogger(rtc::LogLevel::Info);

    rtcConfig_.iceServers.emplace_back(config.stunServer);

    std::cout << "The local ID is " << localId_ << std::endl;

    ws_ = std::make_shared<rtc::WebSocket>();

    wsFuture_ = wsPromise_.get_future();

    ws_->onOpen(
        [this]()
        {
            std::cout << "WebSocket connected, signaling ready" << std::endl;
            this->wsPromise_.set_value();
        });

    ws_->onError(
        [this](const std::string &s)
        {
            std::cout << "WebSocket error" << std::endl;
            this->wsPromise_.set_exception(std::make_exception_ptr(std::runtime_error(s)));
        });

    ws_->onClosed([]() { std::cout << "WebSocket closed" << std::endl; });

    ws_->onMessage(
        [wws = make_weak_ptr(ws_), this](auto data)
        {
            // data holds either std::string or rtc::binary
            if(!std::holds_alternative<std::string>(data)) return;

            json message = json::parse(std::get<std::string>(data));

            auto it = message.find("id");
            if(it == message.end()) return;

            auto id = it->get<std::string>();

            it = message.find("auth");
            if(it == message.end() || it->get<std::string>() != auth_) return;

            it = message.find("access");
            if(it == message.end()) return;

            auto access = it->get<std::string>();

            it = message.find("type");
            if(it == message.end()) return;

            auto type = it->get<std::string>();

            shared_ptr<PeerConnection> pc;
            if(auto jt = peerConnectionMap_.find(id); jt != peerConnectionMap_.end())
            {
                pc = jt->second;
            }
            else if(type == "offer")
            {
                std::cout << "Answering to " + id << std::endl;
                pc = createPeerConnection(id, access);
            }
            else
            {
                return;
            }

            pc->handleConnectionMessage(message);
        });
}

void Teleoperation::startSignaling()
{
    std::cout << "Signaling URL is " << wsUrl_ << std::endl;
    ws_->open(wsUrl_);
    std::cout << "Waiting for signaling to be connected..." << std::endl;
    wsFuture_.get();
}

void Teleoperation::sendOffer(const std::string &remoteId, const std::string &access)
{
    std::cout << "Sending an offer to " + remoteId << std::endl;
    auto pc = createPeerConnection(remoteId, access);
    pc->createConnection();
}

void Teleoperation::onChannelOpen(const std::function<void()> &callback) { onChannelOpenCallback_ = callback; }

void Teleoperation::onChannelClosed(const std::function<void()> &callback) { onChannelClosedCallback_ = callback; }

void Teleoperation::onChannelMessage(const std::function<void(std::string data)> &callback)
{
    onChannelMessageCallback_ = callback;
}

void Teleoperation::onChannelControlMessage(const std::function<void(std::string)> &callback)
{
    onChannelControlMessageCallback_ = callback;
}

void Teleoperation::sendMessage(const std::string &remoteId, const std::string &message)
{
    if(auto it = peerConnectionMap_.find(remoteId); it != peerConnectionMap_.end()) it->second->sendMessage(message);
}

void Teleoperation::broadcastMessage(const std::string &message)
{
    for(auto &[id, pc] : peerConnectionMap_) pc->sendMessage(message);
}

void Teleoperation::sendVideoFrame(const std::string &remoteId, const uint8_t *frameData, int width, int height,
                                   size_t step)
{
    videoEncoder_->encodeFrame(frameData, width, height, step);
    while(videoEncoder_->nextPacket())
    {
        auto data = videoEncoder_->getPacketData();
        auto len = videoEncoder_->getPacketSize();
        auto timestamp = videoEncoder_->getElapsedTime();
        if(auto it = peerConnectionMap_.find(remoteId); it != peerConnectionMap_.end())
        {
            it->second->sendVideoFrame(data, len, timestamp);
        }
    }
}

void Teleoperation::broadcastVideoFrame(const uint8_t *frameData, int width, int height, size_t step)
{
    videoEncoder_->encodeFrame(frameData, width, height, step);
    while(videoEncoder_->nextPacket())
    {
        auto data = videoEncoder_->getPacketData();
        auto len = videoEncoder_->getPacketSize();
        auto timestamp = videoEncoder_->getElapsedTime();
        for(auto &[id, pc] : peerConnectionMap_)
        {
            pc->sendVideoFrame(data, len, timestamp);
        }
    }
}

shared_ptr<PeerConnection> Teleoperation::createPeerConnection(const std::string &remoteId, const std::string &access)
{
    PeerConnection::Configuration pcConfig;
    pcConfig.rtcConfig = rtcConfig_;
    pcConfig.wws = make_weak_ptr(ws_);
    pcConfig.localId = this->localId_;
    pcConfig.remoteId = remoteId;
    pcConfig.channelCallbacks.onChannelOpenCallback = onChannelOpenCallback_;
    pcConfig.channelCallbacks.onChannelClosedCallback = onChannelClosedCallback_;
    pcConfig.startTime = this->videoEncoder_->getStartTime();
    if(access == "control")
        pcConfig.channelCallbacks.onChannelMessageCallback = onChannelControlMessageCallback_;
    else
        pcConfig.channelCallbacks.onChannelMessageCallback = onChannelMessageCallback_;
    auto pc = std::make_shared<PeerConnection>(pcConfig);
    peerConnectionMap_.emplace(remoteId, pc);
    return pc;
}
