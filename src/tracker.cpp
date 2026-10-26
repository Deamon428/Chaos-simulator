#include "tracker.h"

#include <opencv2/imgproc.hpp>
// OpenCV 5 moved contourArea/moments out of imgproc into geometry/2d.hpp.
#include <opencv2/geometry/2d.hpp>

Tracker::Tracker(const SetupConfig& cfg) : cfg_(cfg) {
    // Board-region mask: the magnet triangle grown about its centroid. The
    // warp corners lie outside the physical board and only contain junk.
    std::vector<cv::Point2d> tri = cfg.canonical_magnets.empty() ? cfg.magnets
                                                                 : cfg.canonical_magnets;
    if (tri.size() >= 3) {
        cv::Point2d c(0, 0);
        for (const auto& p : tri) c += p;
        c *= 1.0 / tri.size();
        std::vector<cv::Point> poly;
        for (const auto& p : tri)
            poly.push_back(c + (p - c) * cfg_.roi_scale);
        roi_ = cv::Mat::zeros(cfg_.board_size, cfg_.board_size, CV_8U);
        cv::fillPoly(roi_, std::vector<std::vector<cv::Point>>{poly}, 255);
    }
}

bool Tracker::detectMask(const cv::Mat& warped, cv::Mat& mask) const {
    if (cfg_.detection_mode == "bright") {
        cv::Mat gray;
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, cfg_.bright_threshold, 255, cv::THRESH_BINARY);
    } else {
        // Default: HSV color range.
        cv::Mat hsv;
        cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cfg_.hsv_lower, cfg_.hsv_upper, mask);
    }
    if (!roi_.empty()) cv::bitwise_and(mask, roi_, mask);
    return true;
}

bool Tracker::detectRaw(const cv::Mat& warped, cv::Point2d& out) const {
    cv::Mat mask;
    detectMask(warped, mask);

    // Remove speckle noise; the kernel must be wider than the string's
    // warped width or the blob's centroid lands mid-string.
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               {cfg_.open_kernel, cfg_.open_kernel}));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Collect all area-valid candidates; pick the largest, or the one nearest
    // the previous position (the white tape at the hub is also in the color
    // band, so continuity — not size — disambiguates once we have a track).
    std::vector<cv::Point2d> cands;
    double best_area = 0;
    cv::Point2d largest(0, 0);
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < cfg_.min_blob_area || area > cfg_.max_blob_area) continue;
        cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        cv::Point2d p(m.m10 / m.m00, m.m01 / m.m00);
        cands.push_back(p);
        if (area > best_area) {
            best_area = area;
            largest = p;
        }
    }
    if (cands.empty()) return false;

    // Drop candidates sitting on static bright objects (tape, magnets, wood
    // highlights) — the moving bob is never one of them. When everything is
    // near something static (bob crossing the tape) keep the raw candidates.
    if (!cfg_.static_blobs.empty()) {
        std::vector<cv::Point2d> filtered;
        for (const auto& p : cands) {
            double dmin = -1;
            for (const auto& s : cfg_.static_blobs) {
                double d = cv::norm(p - s);
                if (dmin < 0 || d < dmin) dmin = d;
            }
            if (dmin > cfg_.static_reject_px) filtered.push_back(p);
        }
        if (!filtered.empty()) cands = std::move(filtered);
    }

    if (!have_last_) {
        out = largest;
        return true;
    }
    double best_d = -1;
    cv::Point2d nearest;
    for (const auto& p : cands) {
        double d = cv::norm(p - last_);
        if (best_d < 0 || d < best_d) {
            best_d = d;
            nearest = p;
        }
    }
    out = best_d <= cfg_.track_gate_px ? nearest : largest;
    return true;
}

void Tracker::debugMask(const cv::Mat& warped, cv::Mat& mask) const {
    detectMask(warped, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               {cfg_.open_kernel, cfg_.open_kernel}));
}

bool Tracker::step(const cv::Mat& warped, cv::Point2d& out) {
    cv::Point2d raw;
    if (detectRaw(warped, raw)) {
        since_last_detect_ = 0;
        last_ = raw;
        have_last_ = true;
        recent_.push_back(raw);
        while (static_cast<int>(recent_.size()) > cfg_.smooth_window) recent_.pop_front();
    } else {
        ++since_last_detect_;
        // A long gap breaks the trail: stale smoothing samples would produce
        // phantom motion between the old and new positions.
        if (since_last_detect_ > cfg_.max_gap_frames) recent_.clear();
        return false;
    }

    cv::Point2d sum(0, 0);
    for (const auto& p : recent_) sum += p;
    out = sum * (1.0 / static_cast<double>(recent_.size()));
    return true;
}

void Tracker::reset() {
    recent_.clear();
    since_last_detect_ = 0;
    have_last_ = false;
}
