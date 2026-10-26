// Bob detection on the warped board image: color/brightness threshold,
// largest-blob centroid, and moving-average smoothing.
#pragma once

#include <opencv2/core.hpp>
#include <deque>

#include "config.h"

class Tracker {
public:
    explicit Tracker(const SetupConfig& cfg);

    // Processes one warped frame. Returns the smoothed position in board
    // coordinates; ok is false when the bob was not detected this frame.
    bool step(const cv::Mat& warped, cv::Point2d& out);

    // Raw (unsmoothed) detection, useful for debug overlays.
    bool detectRaw(const cv::Mat& warped, cv::Point2d& out) const;

    // The post-open detection mask, for --dump-mask debugging.
    void debugMask(const cv::Mat& warped, cv::Mat& mask) const;

    void reset();

private:
    bool detectMask(const cv::Mat& warped, cv::Mat& mask) const;

    SetupConfig cfg_;
    cv::Mat roi_;  // board-region mask (empty = unrestricted)
    std::deque<cv::Point2d> recent_;
    int since_last_detect_ = 0;
    cv::Point2d last_;
    bool have_last_ = false;
};
