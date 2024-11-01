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
    const auto pc = std::make_shared<rtc::PeerConnection>(config.rtcConfig);

    setDefaultCallbacks();

    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(config.payloadType);  // Must match the payload type of the external h264 RTP stream
    media.addSSRC(config.ssrc, "video-send");
    const auto track = pc->addTrack(static_cast<rtc::Description::Media>(media));

    // create RTP configuration
    auto rtpConfig = make_shared<rtc::RtpPacketizationConfig>(config.ssrc, "video", config.payloadType,
                                                              rtc::H264RtpPacketizer::defaultClockRate);
    // create packetizer
    const auto packetizer =
        make_shared<rtc::H264RtpPacketizer>(rtc::H264RtpPacketizer::Separator::StartSequence, rtpConfig);
    // add RTCP SR handler
    const auto srReporter = make_shared<rtc::RtcpSrReporter>(rtpConfig);
    packetizer->addToChain(srReporter);
    // set handler
    track->setMediaHandler(packetizer);

    this->srReporter_ = srReporter;
    this->track_ = track;

    this->srReporter_->rtpConfig->startTimestamp = timestampMicroToRtp(config.startTime);

    pc->onStateChange([](const rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

    pc->onGatheringStateChange([](const rtc::PeerConnection::GatheringState state)
                               { std::cout << "Gathering State: " << state << std::endl; });

    pc->onLocalDescription(
        [config](const rtc::Description &description)
        {
            const json message = {
                {"id", config.remoteId}, {"type", description.typeString()}, {"description", std::string(description)}};

            if(const auto ws = config.wws.lock()) ws->send(message.dump());
        });

    pc->onLocalCandidate(
        [config](const rtc::Candidate &candidate)
        {
            const json message = {{"id", config.remoteId},
                                  {"type", "candidate"},
                                  {"candidate", std::string(candidate)},
                                  {"mid", candidate.mid()}};

            if(const auto ws = config.wws.lock()) ws->send(message.dump());
        });

    pc->onDataChannel(
        [this](const shared_ptr<rtc::DataChannel> &dc)
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

void PeerConnection::sendMessage(const std::string &message) const
{
    if(dataChannel_ && dataChannel_->isOpen()) this->dataChannel_->send(message);
}

void PeerConnection::sendVideoFrame(const std::byte *data, const size_t len, const int64_t timestampMicro) const
{
    if(track_ != nullptr && track_->isOpen())
    {
        const auto elapsedTimestamp = timestampMicroToRtp(timestampMicro);
        srReporter_->rtpConfig->timestamp += srReporter_->rtpConfig->startTimestamp + elapsedTimestamp;

        if(const auto reportElapsedTimestamp = srReporter_->rtpConfig->timestamp - srReporter_->lastReportedTimestamp();
           srReporter_->rtpConfig->timestampToSeconds(reportElapsedTimestamp) > 1)
        {
            srReporter_->setNeedsToReport();
        }

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

uint32_t PeerConnection::timestampMicroToRtp(const uint64_t timestampMicro) const
{
    const auto seconds = static_cast<double>(timestampMicro) / 1000000.0;
    return srReporter_->rtpConfig->secondsToTimestamp(seconds);
}

void PeerConnection::createDataChannel()
{
    dataChannel_ = rtcPeerConnection_->createDataChannel("test");
    configureDataChannel();
    rtcPeerConnection_->setLocalDescription();
}

void PeerConnection::handleConnectionMessage(const json &message) const
{
    const auto it = message.find("type");
    if(it == message.end()) return;

    if(const auto type = it->get<std::string>(); type == "offer")
    {
        const auto sdp = message["description"].get<std::string>();
        rtcPeerConnection_->setRemoteDescription(rtc::Description(sdp, type));
        rtcPeerConnection_->setLocalDescription();
    }
    else if(type == "answer")
    {
        const auto sdp = message["description"].get<std::string>();
        rtcPeerConnection_->setRemoteDescription(rtc::Description(sdp, type));
    }
    else if(type == "candidate")
    {
        const auto sdp = message["candidate"].get<std::string>();
        const auto mid = message["mid"].get<std::string>();
        rtcPeerConnection_->addRemoteCandidate(rtc::Candidate(sdp, mid));
    }
}

PeerConnection::~PeerConnection()
{
    track_->close();
    dataChannel_->close();
    rtcPeerConnection_->close();
}
