#include "stream_video_cv_action.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>

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

    teleoperation->broadcastVideo(frame.data, frame.cols, frame.rows, frame.step);

    return true;
}

void StreamVideoCvAction::close() { cap.release(); }
