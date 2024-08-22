#ifndef LIBDATACHANNEL_APP_VIDEOENCODER_H
#define LIBDATACHANNEL_APP_VIDEOENCODER_H

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>
}

class VideoEncoder
{
public:
    VideoEncoder();

    void encodeFrame(const uint8_t *data, int width, int height, int step);

    bool nextPacket();

    const std::byte *getPacketData();

    size_t getPacketSize();

    int64_t getStartTime() const;

    int64_t getElapsedTime() const;

    ~VideoEncoder();

private:
    int64_t startTime;
    int frameIndex;
    AVCodecContext *codecContext;
    AVFrame *avFrame;
    AVPacket *avPacket;
};

#endif  // LIBDATACHANNEL_APP_VIDEOENCODER_H
