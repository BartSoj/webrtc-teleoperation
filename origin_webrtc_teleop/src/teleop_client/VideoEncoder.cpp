#include "VideoEncoder.hpp"

#include <iostream>

VideoEncoder::VideoEncoder()
{
    // Initialize FFmpeg encoder
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if(!codec)
    {
        std::cerr << "Error: Could not find encoder" << std::endl;
        return;
    }

    codecContext = avcodec_alloc_context3(codec);
    if(!codecContext)
    {
        std::cerr << "Error: Could not allocate codec context" << std::endl;
        return;
    }
    codecContext->bit_rate = 1000000;
    codecContext->width = 640;
    codecContext->height = 480;
    codecContext->time_base = AVRational{1, 30};
    codecContext->gop_size = 10;
    codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    codecContext->flags |= AV_CODEC_FLAG_LOW_DELAY;
    codecContext->max_b_frames = 0;
    codecContext->refs = 1;

    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "preset", "ultrafast", 0);
    av_dict_set(&opts, "tune", "zerolatency", 0);

    if(avcodec_open2(codecContext, codec, &opts) < 0)
    {
        std::cerr << "Error: Could not open codec" << std::endl;
        return;
    }

    av_dict_free(&opts);
    avFrame = av_frame_alloc();
    if(!avFrame)
    {
        std::cerr << "Error: Could not allocate frame" << std::endl;
        return;
    }
    avFrame->format = codecContext->pix_fmt;
    avFrame->width = codecContext->width;
    avFrame->height = codecContext->height;
    if(av_frame_get_buffer(avFrame, 32) < 0)
    {
        std::cerr << "Error: Could not allocate frame buffer" << std::endl;
        return;
    }

    avPacket = av_packet_alloc();
    if(!avPacket)
    {
        std::cerr << "Error: Could not allocate packet" << std::endl;
        return;
    }

    startTime = av_gettime_relative();
    frameIndex = 0;
}

void VideoEncoder::encodeFrame(const uint8_t* data, int width, int height, int step)
{
    // Convert frame to YUV420P
    SwsContext* swsCtx = sws_getContext(width, height, AV_PIX_FMT_RGB24, codecContext->width, codecContext->height,
                                        AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    if(!swsCtx)
    {
        std::cerr << "Error: Could not create SwsContext" << std::endl;
        return;
    }

    uint8_t* srcData[AV_NUM_DATA_POINTERS] = {
        const_cast<uint8_t*>(data), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    int srcLinesize[AV_NUM_DATA_POINTERS] = {step, 0, 0, 0, 0, 0, 0, 0};
    sws_scale(swsCtx, srcData, srcLinesize, 0, height, avFrame->data, avFrame->linesize);
    sws_freeContext(swsCtx);

    avFrame->pts = frameIndex++;

    int ret = avcodec_send_frame(codecContext, avFrame);
    if(ret < 0)
    {
        std::cerr << "Error sending frame for encoding" << std::endl;
        return;
    }
}

bool VideoEncoder::nextPacket()
{
    av_packet_unref(avPacket);
    int ret = avcodec_receive_packet(codecContext, avPacket);
    if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    {
        return false;
    }
    else if(ret < 0)
    {
        std::cerr << "Error receiving encoded packet" << std::endl;
        return false;
    }
    return true;
}

const std::byte* VideoEncoder::getPacketData() { return reinterpret_cast<const std::byte*>(avPacket->data); }

size_t VideoEncoder::getPacketSize() { return avPacket->size; }

int64_t VideoEncoder::getStartTime() const { return startTime; }

int64_t VideoEncoder::getElapsedTime() const { return av_gettime_relative() - startTime; }

VideoEncoder::~VideoEncoder()
{
    av_packet_free(&avPacket);
    av_frame_free(&avFrame);
    avcodec_free_context(&codecContext);
}
