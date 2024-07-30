#ifndef LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H
#define LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

#include <fstream>
#include <opencv2/opencv.hpp>

#include "Action.hpp"

class StreamVideoCvAction : public Action
{
public:
    using Action::Action;

protected:
    void init() override;

    bool loop() override;

    void cleanup() override;

    cv::VideoCapture cap;
    int frameIndex;
    AVCodecContext *codecContext;
    AVFrame *avFrame;
    AVPacket *avPacket;
};

#endif  // LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H
