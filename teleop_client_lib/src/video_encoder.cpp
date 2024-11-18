#include "teleop_client/video_encoder.hpp"

#include <iostream>

VideoEncoder::VideoEncoder(const VideoEncoderConfig& config)
{
    // Initialize FFmpeg encoder
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if(!codec)
    {
        std::cerr << "Error: Could not find encoder" << std::endl;
        return;
    }

    codecContext_ = avcodec_alloc_context3(codec);
    if(!codecContext_)
    {
        std::cerr << "Error: Could not allocate codec context" << std::endl;
        return;
    }
    codecContext_->bit_rate = config.bit_rate;
    codecContext_->width = config.width;
    codecContext_->height = config.height;
    codecContext_->time_base = config.time_base;
    codecContext_->gop_size = config.gop_size;
    codecContext_->pix_fmt = config.pix_fmt;
    codecContext_->flags |= config.codec_flags;
    codecContext_->max_b_frames = config.max_b_frames;
    codecContext_->refs = config.refs;

    AVDictionary* opts = nullptr;
    for(const auto& [key, value] : config.options)
    {
        av_dict_set(&opts, key.c_str(), value.c_str(), 0);
    }

    if(avcodec_open2(codecContext_, codec, &opts) < 0)
    {
        std::cerr << "Error: Could not open codec" << std::endl;
        return;
    }

    av_dict_free(&opts);
    avFrame_ = av_frame_alloc();
    if(!avFrame_)
    {
        std::cerr << "Error: Could not allocate frame" << std::endl;
        return;
    }
    avFrame_->format = codecContext_->pix_fmt;
    avFrame_->width = codecContext_->width;
    avFrame_->height = codecContext_->height;
    if(av_frame_get_buffer(avFrame_, 32) < 0)
    {
        std::cerr << "Error: Could not allocate frame buffer" << std::endl;
        return;
    }

    avPacket_ = av_packet_alloc();
    if(!avPacket_)
    {
        std::cerr << "Error: Could not allocate packet" << std::endl;
        return;
    }

    startTime_ = av_gettime_relative();
    frameIndex_ = 0;
}

void VideoEncoder::encodeFrame(const uint8_t* data, int width, int height, int step)
{
    // Convert frame to YUV420P
    SwsContext* swsCtx = sws_getContext(width, height, AV_PIX_FMT_RGB24, codecContext_->width, codecContext_->height,
                                        AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    if(!swsCtx)
    {
        std::cerr << "Error: Could not create SwsContext" << std::endl;
        return;
    }

    uint8_t* srcData[AV_NUM_DATA_POINTERS] = {
        const_cast<uint8_t*>(data), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    int srcLinesize[AV_NUM_DATA_POINTERS] = {step, 0, 0, 0, 0, 0, 0, 0};
    sws_scale(swsCtx, srcData, srcLinesize, 0, height, avFrame_->data, avFrame_->linesize);
    sws_freeContext(swsCtx);

    frameIndex_++;
    int64_t currentTime = av_gettime_relative() - startTime_;

    // Set frame timestamp either by frame index or by current time
    // avFrame_->pts = (frameIndex_ * codecContext_->time_base.den) / 5.986;
    avFrame_->pts = av_rescale_q(currentTime, {1, AV_TIME_BASE}, codecContext_->time_base);

    int ret = avcodec_send_frame(codecContext_, avFrame_);
    if(ret < 0)
    {
        std::cerr << "Error sending frame for encoding" << std::endl;
    }
}

bool VideoEncoder::nextPacket() const
{
    av_packet_unref(avPacket_);
    const int ret = avcodec_receive_packet(codecContext_, avPacket_);
    if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    {
        return false;
    }
    if(ret < 0)
    {
        std::cerr << "Error receiving encoded packet" << std::endl;
        return false;
    }
    return true;
}

const std::byte* VideoEncoder::getPacketData() const { return reinterpret_cast<const std::byte*>(avPacket_->data); }

size_t VideoEncoder::getPacketSize() const { return avPacket_->size; }

int64_t VideoEncoder::getStartTime() const { return startTime_; }

int64_t VideoEncoder::getElapsedTime() const { return av_gettime_relative() - startTime_; }

VideoEncoder::~VideoEncoder()
{
    av_packet_free(&avPacket_);
    av_frame_free(&avFrame_);
    avcodec_free_context(&codecContext_);
}
