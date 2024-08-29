#ifndef LIBDATACHANNEL_APP_VIDEOENCODER_H
#define LIBDATACHANNEL_APP_VIDEOENCODER_H

#include <map>
#include <string>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>
}

struct VideoEncoderConfig
{
    int64_t bit_rate = 1000000;
    int width = 640;
    int height = 480;
    AVRational time_base = {1, 30};
    int gop_size = 10;
    AVPixelFormat pix_fmt = AV_PIX_FMT_YUV420P;
    int codec_flags = AV_CODEC_FLAG_LOW_DELAY;
    int max_b_frames = 0;
    int refs = 1;

    std::map<std::string, std::string> options = {
        {"preset", "ultrafast"},
        {"tune", "zerolatency"}
    };

};

class VideoEncoder
{
public:
    VideoEncoder(const VideoEncoderConfig &config);

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
