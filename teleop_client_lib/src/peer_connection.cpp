#include "teleop_client/peer_connection.hpp"

using namespace std::chrono_literals;
using nlohmann::json;
using std::make_shared;
using std::shared_ptr;
using std::weak_ptr;

template <class T>
weak_ptr<T> make_weak_ptr(shared_ptr<T> ptr)
{
    return ptr;
}

PeerConnection::PeerConnection(const Configuration &config)
    : localId_(config.localId), remoteId_(config.remoteId), channelCallbacks_(config.channelCallbacks)
{
    auto pc = std::make_shared<rtc::PeerConnection>(config.rtcConfig);

    setDefaultCallbacks();

    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(config.payloadType);  // Must match the payload type of the external h264 RTP stream
    media.addSSRC(config.ssrc, "video-send");
    auto track = pc->addTrack(media);

    // create RTP configuration
    auto rtpConfig = make_shared<rtc::RtpPacketizationConfig>(config.ssrc, "video", config.payloadType,
                                                              rtc::H264RtpPacketizer::defaultClockRate);
    // create packetizer
    auto packetizer = make_shared<rtc::H264RtpPacketizer>(rtc::H264RtpPacketizer::Separator::StartSequence, rtpConfig);
    // add RTCP SR handler
    auto srReporter = make_shared<rtc::RtcpSrReporter>(rtpConfig);
    packetizer->addToChain(srReporter);
    // add RTCP NACK handler
    auto nackResponder = make_shared<rtc::RtcpNackResponder>();
    packetizer->addToChain(nackResponder);
    // set handler
    track->setMediaHandler(packetizer);

    this->srReporter_ = srReporter;
    this->track_ = track;

    this->srReporter_->rtpConfig->startTimestamp = timestampMicroToRtp(config.startTime);

    pc->onStateChange([](rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

    pc->onGatheringStateChange([](rtc::PeerConnection::GatheringState state)
                               { std::cout << "Gathering State: " << state << std::endl; });

    pc->onLocalDescription(
        [config](rtc::Description description)
        {
            json message = {
                {"id", config.remoteId}, {"type", description.typeString()}, {"description", std::string(description)}};

            if(auto ws = config.wws.lock()) ws->send(message.dump());
        });

    pc->onLocalCandidate(
        [config](rtc::Candidate candidate)
        {
            json message = {{"id", config.remoteId},
                            {"type", "candidate"},
                            {"candidate", std::string(candidate)},
                            {"mid", candidate.mid()}};

            if(auto ws = config.wws.lock()) ws->send(message.dump());
        });

    pc->onDataChannel(
        [this](shared_ptr<rtc::DataChannel> dc)
        {
            std::cout << "DataChannel from " << this->remoteId_ << " received with label \"" << dc->label() << "\""
                      << std::endl;
            this->dataChannel_ = dc;
            configureDataChannel();
        });

    this->rtcPeerConnection_ = pc;
}

void PeerConnection::setDefaultCallbacks()
{
    if(!channelCallbacks_.onChannelOpenCallback)
    {
        channelCallbacks_.onChannelOpenCallback = [this]()
        {
            auto wdc = make_weak_ptr(dataChannel_);

            json helloMessage = {{"type", "log"}, {"message", "Hello from " + localId_}};
            if(auto dc = wdc.lock()) dc->send(helloMessage.dump());
        };
    }

    if(!channelCallbacks_.onChannelClosedCallback)
    {
        channelCallbacks_.onChannelClosedCallback = [this]()
        { std::cout << "DataChannel from " << remoteId_ << " closed" << std::endl; };
    }

    if(!channelCallbacks_.onChannelMessageCallback)
    {
        channelCallbacks_.onChannelMessageCallback = [this](auto data)
        { std::cout << "Message from " << remoteId_ << " received: " << data << std::endl; };
    }
}

void PeerConnection::configureDataChannel()
{
    dataChannel_->onOpen(channelCallbacks_.onChannelOpenCallback);
    dataChannel_->onClosed(channelCallbacks_.onChannelClosedCallback);
    dataChannel_->onMessage(
        [this](auto data)
        {
            if(std::holds_alternative<std::string>(data))
            {
                channelCallbacks_.onChannelMessageCallback(std::get<std::string>(data));
            }
        });
}

void PeerConnection::sendMessage(const std::string &message)
{
    if(dataChannel_ && dataChannel_->isOpen()) this->dataChannel_->send(message);
}

void PeerConnection::sendVideoFrame(const std::byte *data, size_t len, int64_t timestampMicro)
{
    if(track_ != nullptr && track_->isOpen())
    {
        auto elapsedTimestamp = timestampMicroToRtp(timestampMicro);
        srReporter_->rtpConfig->timestamp += srReporter_->rtpConfig->startTimestamp + elapsedTimestamp;

        try
        {
            track_->send(data, len);
        }
        catch(const std::exception &e)
        {
            std::cerr << "Unable to send video packet: " << e.what() << std::endl;
        }
    }
}

uint32_t PeerConnection::timestampMicroToRtp(uint64_t timestampMicro)
{
    auto seconds = static_cast<double>(timestampMicro) / 1000000.0;
    return srReporter_->rtpConfig->secondsToTimestamp(seconds);
}

void PeerConnection::createDataChannel()
{
    dataChannel_ = rtcPeerConnection_->createDataChannel("test");
    configureDataChannel();
    rtcPeerConnection_->setLocalDescription();
}

void PeerConnection::handleConnectionMessage(const nlohmann::json &message)
{
    auto it = message.find("type");
    if(it == message.end()) return;

    auto type = it->get<std::string>();

    if(type == "offer")
    {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection_->setRemoteDescription(rtc::Description(sdp, type));
        rtcPeerConnection_->setLocalDescription();
    }
    else if(type == "answer")
    {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection_->setRemoteDescription(rtc::Description(sdp, type));
    }
    else if(type == "candidate")
    {
        auto sdp = message["candidate"].get<std::string>();
        auto mid = message["mid"].get<std::string>();
        rtcPeerConnection_->addRemoteCandidate(rtc::Candidate(sdp, mid));
    }
}

PeerConnection::~PeerConnection()
{
    track_->close();
    dataChannel_->close();
    rtcPeerConnection_->close();
}
