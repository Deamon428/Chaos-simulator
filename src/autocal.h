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

struct PhotoCalResult {
    bool ok = false;
    std::string error;
    cv::Mat homography;                    // photo px -> canonical board px (3x3)
    std::vector<cv::Point2d> magnets_image;  // detected magnets, photo pixels
    cv::Point2d hub_image;
    cv::Mat annotated;                     // photo with markers drawn
};

// Single-image variant: detects the hub + 3 magnets on a top-down photo of
// the board and returns the affine mapping photo -> canonical layout.
PhotoCalResult photoCalibrate(const cv::Mat& image, const SetupConfig& cfg,
                              int label_offset);
