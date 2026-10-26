#include "video_source.h"

#include <cmath>

VideoSource::VideoSource(const std::string& path) {
    if (!cap_.open(path)) error_ = "could not open video file '" + path + "'";
}

VideoSource::VideoSource(int device_index) {
    if (!cap_.open(device_index)) error_ = "could not open camera device " + std::to_string(device_index);
}

bool VideoSource::read(cv::Mat& frame) { return cap_.read(frame) && !frame.empty(); }

bool VideoSource::seek(int frame_index) {
    return cap_.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frame_index));
}

double VideoSource::fps() const {
    double f = cap_.get(cv::CAP_PROP_FPS);
    if (f <= 0 || !std::isfinite(f)) f = 30.0;
    return f;
}

int VideoSource::frameCount() const {
    return static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
}
