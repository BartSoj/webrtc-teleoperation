#include "StreamVideoCvAction.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

void StreamVideoCvAction::init()
{
    cap.open(0);
    if(!cap.isOpened())
    {
        std::cerr << "Error: Could not open camera" << std::endl;
        return;
    }

    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

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
    codecContext->time_base = (AVRational){1, 30};
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
}

bool StreamVideoCvAction::loop()
{
    cv::Mat frame;
    cap >> frame;
    if(frame.empty())
    {
        std::cerr << "Error: Could not capture frame" << std::endl;
        return false;
    }

    // Convert frame to YUV420P
    SwsContext* swsCtx =
        sws_getContext(frame.cols, frame.rows, AV_PIX_FMT_BGR24, codecContext->width, codecContext->height,
                       AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if(!swsCtx)
    {
        std::cerr << "Error: Could not create SwsContext" << std::endl;
        return false;
    }

    uint8_t* data[AV_NUM_DATA_POINTERS] = {frame.data, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    int linesize[AV_NUM_DATA_POINTERS] = {static_cast<int>(frame.step), 0, 0, 0, 0, 0, 0, 0};
    sws_scale(swsCtx, data, linesize, 0, frame.rows, avFrame->data, avFrame->linesize);
    sws_freeContext(swsCtx);

    // Encode frame
    avFrame->pts = frameIndex++;

    int ret = avcodec_send_frame(codecContext, avFrame);
    if(ret < 0)
    {
        std::cerr << "Error sending frame for encoding" << std::endl;
        return false;
    }

    while(true)
    {
        ret = avcodec_receive_packet(codecContext, avPacket);
        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        {
            break;
        }
        else if(ret < 0)
        {
            std::cerr << "Error receiving encoded packet" << std::endl;
            return false;
        }

        // Send encoded data via WebRTC immediately
        const std::byte* data = reinterpret_cast<const std::byte*>(avPacket->data);
        size_t len = avPacket->size;
        uint64_t timestamp = avPacket->pts * (1000000 / codecContext->time_base.den);

        for(const auto& [id, peerConnection] : teleoperation->getPeerConnectionMap())
        {
            peerConnection->sendVideo(data, len, timestamp);
        }

        av_packet_unref(avPacket);
    }

    return true;
}

void StreamVideoCvAction::cleanup()
{
    cap.release();

    av_packet_free(&avPacket);
    av_frame_free(&avFrame);
    avcodec_free_context(&codecContext);
}
