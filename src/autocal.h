// Per-video auto-calibration: detect the 3 base magnets (small, low-saturation,
// static blobs on the dark board) across several sampled frames, then fit an
// affine transform from their image centroids to the canonical board layout.
#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "config.h"

struct AutoCalResult {
    bool ok = false;
    std::string error;
    cv::Mat homography;                    // image px -> board coords (3x3, CV_64F)
    std::vector<cv::Point2d> magnets_img;  // detected centroids, board-labeled order
    std::vector<cv::Point2d> static_blobs_img;  // all stable bright blobs (magnets,
                                                // tape, highlights) for rejection
    cv::Mat sample_frame;                  // frame used for the preview dump
};

// label_offset rotates the magnet labeling by N positions (for videos where
// the phone was held rotated relative to the board).
AutoCalResult autoCalibrate(const std::string& video_path, const SetupConfig& cfg,
                            int label_offset, int first_frame = 0);
