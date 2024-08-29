#include "PeerConnection.hpp"

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
    : localId(config.localId), remoteId(config.remoteId), channelCallbacks(config.channelCallbacks)
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

    this->srReporter = srReporter;
    this->track = track;

    srReporter->rtpConfig->startTimestamp = timestampMicroToRtp(config.startTime);

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
            std::cout << "DataChannel from " << this->remoteId << " received with label \"" << dc->label() << "\""
                      << std::endl;
            this->dataChannel = dc;
            configureDataChannel();
        });

    this->rtcPeerConnection = pc;
}

void PeerConnection::setDefaultCallbacks()
{
    if(!channelCallbacks.onChannelOpenCallback)
    {
        channelCallbacks.onChannelOpenCallback = [this]()
        {
            auto wdc = make_weak_ptr(dataChannel);

            json helloMessage = {{"type", "log"}, {"message", "Hello from " + localId}};
            if(auto dc = wdc.lock()) dc->send(helloMessage.dump());
        };
    }

    if(!channelCallbacks.onChannelClosedCallback)
    {
        channelCallbacks.onChannelClosedCallback = [this]()
        { std::cout << "DataChannel from " << remoteId << " closed" << std::endl; };
    }

    if(!channelCallbacks.onChannelMessageCallback)
    {
        channelCallbacks.onChannelMessageCallback = [this](auto data)
        { std::cout << "Message from " << remoteId << " received: " << data << std::endl; };
    }
}

void PeerConnection::configureDataChannel()
{
    dataChannel->onOpen(channelCallbacks.onChannelOpenCallback);
    dataChannel->onClosed(channelCallbacks.onChannelClosedCallback);
    dataChannel->onMessage(
        [this](auto data)
        {
            if(std::holds_alternative<std::string>(data))
            {
                channelCallbacks.onChannelMessageCallback(std::get<std::string>(data));
            }
        });
}

void PeerConnection::sendMessage(const std::string &message)
{
    if(dataChannel && dataChannel->isOpen()) this->dataChannel->send(message);
}

void PeerConnection::sendVideoFrame(const std::byte *data, size_t len, int64_t timestampMicro)
{
    if(track != nullptr && track->isOpen())
    {
        auto elapsedTimestamp = timestampMicroToRtp(timestampMicro);
        srReporter->rtpConfig->timestamp += srReporter->rtpConfig->startTimestamp + elapsedTimestamp;

        try
        {
            track->send(data, len);
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
    return srReporter->rtpConfig->secondsToTimestamp(seconds);
}

void PeerConnection::createDataChannel()
{
    dataChannel = rtcPeerConnection->createDataChannel("test");
    configureDataChannel();
    rtcPeerConnection->setLocalDescription();
}

void PeerConnection::handleConnectionMessage(const nlohmann::json &message)
{
    auto it = message.find("type");
    if(it == message.end()) return;

    auto type = it->get<std::string>();

    if(type == "offer")
    {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection->setRemoteDescription(rtc::Description(sdp, type));
        rtcPeerConnection->setLocalDescription();
    }
    else if(type == "answer")
    {
        auto sdp = message["description"].get<std::string>();
        rtcPeerConnection->setRemoteDescription(rtc::Description(sdp, type));
    }
    else if(type == "candidate")
    {
        auto sdp = message["candidate"].get<std::string>();
        auto mid = message["mid"].get<std::string>();
        rtcPeerConnection->addRemoteCandidate(rtc::Candidate(sdp, mid));
    }
}

PeerConnection::~PeerConnection()
{
    track->close();
    dataChannel->close();
    rtcPeerConnection->close();
}
