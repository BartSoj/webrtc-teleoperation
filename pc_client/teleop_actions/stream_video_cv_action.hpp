#ifndef LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H
#define LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H

#include <fstream>
#include <opencv2/opencv.hpp>

#include "action.hpp"

class StreamVideoCvAction : public Action
{
public:
    using Action::Action;

protected:
    void init() override;

    bool loop() override;

    void close() override;

    cv::VideoCapture cap;
};

#endif  // LIBDATACHANNEL_APP_STREAMVIDEOCVACTION_H
