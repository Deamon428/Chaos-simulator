// Abstraction over the video input so a recorded file and a live camera
// can later share the same pipeline.
#pragma once

#include <opencv2/videoio.hpp>
#include <string>

class VideoSource {
public:
    // Opens a file. Use index >= 0 to open a camera device instead.
    explicit VideoSource(const std::string& path);
    VideoSource(int device_index);

    bool isOpened() const { return cap_.isOpened(); }
    std::string error() const { return error_; }

    // Reads the next frame; returns false at end of stream.
    bool read(cv::Mat& frame);
    // Jumps to the given frame index (files only).
    bool seek(int frame_index);
    double fps() const;
    int frameCount() const;

private:
    cv::VideoCapture cap_;
    std::string error_;
};
