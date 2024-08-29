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
    /**
     * @brief Creates a VideoEncoder and initializes the FFmpeg encoder.
     * @param config Configuration for the encoder.
     *
     * Sets up the codec context with parameters such as bit rate, resolution,
     * and pixel format. Allocates necessary resources for encoding.
     */
    VideoEncoder(const VideoEncoderConfig &config);

    /**
     * @brief Encodes a single video frame.
     * @param data Pointer to the RGB24 frame data.
     * @param width Width of the frame.
     * @param height Height of the frame.
     * @param step Step size for the frame data.
     *
     * Converts the input frame from RGB24 to YUV420P format and sends it to the encoder.
     */
    void encodeFrame(const uint8_t *data, int width, int height, int step);

    /**
     * @brief Retrieves the next encoded packet.
     * @return True if a packet is available, false otherwise.
     *
     * Attempts to receive an encoded packet from the codec context.
     */
    bool nextPacket();

    /**
     * @brief Gets the data of the current encoded packet.
     * @return Pointer to the packet data.
     */
    const std::byte *getPacketData();

    /**
     * @brief Gets the size of the current encoded packet.
     * @return Size of the packet in bytes.
     */

    size_t getPacketSize();

    /**
     * @brief Gets the start time of the encoder.
     * @return The start time in microseconds.
     */
    int64_t getStartTime() const;

    /**
     * @brief Gets the elapsed time since the encoder started.
     * @return The elapsed time in microseconds.
     */
    int64_t getElapsedTime() const;

    /**
     * @brief Releases allocated resources for the encoder.
     */
    ~VideoEncoder();

private:
    int64_t startTime;
    int frameIndex;
    AVCodecContext *codecContext;
    AVFrame *avFrame;
    AVPacket *avPacket;
};

#endif  // LIBDATACHANNEL_APP_VIDEOENCODER_H
