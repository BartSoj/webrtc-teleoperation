#pragma once

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