// Setup configuration shared by the tracker and the calibration tool.
// Stored as JSON; homography maps source-frame pixels to board coordinates.
#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

struct SetupConfig {
    int board_size = 800;              // warped board is board_size x board_size px

    // 3x3 homography (source pixel -> board coords). Empty when not calibrated.
    cv::Mat homography;

    std::vector<cv::Point2d> magnets;  // board coords, clicked during calibration
    std::vector<cv::Point2d> valleys;  // board coords of the equilibrium points;
                                       // with ATTRACTION these are the magnets themselves

    // Canonical magnet layout for --autocal (equilateral by default): the
    // detected magnets in each video are mapped onto these points.
    std::vector<cv::Point2d> canonical_magnets;

    // Barycentric coordinates (u, v) of the hub within the magnet triangle,
    // measured once on a reference video. Identifies WHICH detected magnet is
    // M0/M1/M2 consistently across videos (phone rotation does not matter).
    std::vector<double> layout_ref_bary;

    // Magnet auto-detection (--autocal): small low-saturation blobs on the
    // dark board, sampled across several frames and clustered.
    double magnet_v_lower = 110.0;
    double magnet_v_upper = 205.0;
    double magnet_s_upper = 80.0;
    double magnet_min_area = 8.0;
    double magnet_max_area = 150.0;
    double magnet_dark_ring_v = 80.0;  // mean V around a candidate must be below this
    double magnet_bright_core_v = 195.0;  // p90 V at the candidate above this = bob/tape, not magnet
    double tape_board_width = 95.0;    // hub-tape diameter in board px; anchors the
                                       // zoom estimate (tape physical size is fixed)
    int autocal_frames = 13;

    // Bob detection. Area bounds are in BOARD pixels (the warp zooms the
    // image, so a ~12 px image bob becomes ~40 px / ~1500 px^2 on the board).
    std::string detection_mode = "hsv";  // "hsv" or "bright"
    cv::Scalar hsv_lower{0, 60, 60};
    cv::Scalar hsv_upper{30, 255, 255};
    double bright_threshold = 180.0;
    double min_blob_area = 150.0;
    double max_blob_area = 3500.0;

    // Tracking robustness
    int smooth_window = 5;      // moving-average window over raw detections
    int max_gap_frames = 5;     // missed frames before the trail is considered broken
    double track_gate_px = 120.0;  // max jump from the previous position when gating
    int open_kernel = 7;        // morphology open size in BOARD px; must exceed the
                                // string width after warping (the zoom factor scales it)

    // Static bright objects (hub tape, magnets, wood highlights) detected by
    // autocal and mapped to board coords; the moving bob is never one of them.
    std::vector<cv::Point2d> static_blobs;
    double static_reject_px = 45.0;

    // Detection is restricted to the magnet triangle scaled about its centroid
    // by this factor — masks out the warp corners outside the physical board.
    double roi_scale = 1.45;

    // Attraction reach: with the magnets attracting the bob, a genuine run
    // ends essentially on a magnet. A rest point farther than this from every
    // magnet means the attraction never engaged (string too long / no drop).
    double attract_reach_px = 100.0;

    // Returns the equilibrium points for the autocal canonical layout — with
    // attraction, the canonical magnet positions themselves.
    std::vector<cv::Point2d> canonicalEquilibria() const;

    // Canonical magnet layout used when the config doesn't specify one.
    static std::vector<cv::Point2d> defaultCanonicalMagnets();

    // Settle detection
    double settle_speed_px_s = 15.0;  // speed under which the bob counts as quiet
    double settle_window_s = 2.5;     // quiet duration required to declare SETTLED
    double bounce_speed_px_s = 40.0;  // speed above which a settled bob counts as bouncing again
    int settle_avg_frames = 15;       // frames averaged for the final resting point

    // Loads config from JSON. Throws std::runtime_error on malformed input.
    static SetupConfig load(const std::string& path);
    // Same as load(), but missing file returns defaults instead of throwing.
    static SetupConfig loadOrThrow(const std::string& path) { return load(path); }

    // Writes the config back out (calibration tool).
    void save(const std::string& path) const;

    bool calibrated() const { return !homography.empty(); }
};
