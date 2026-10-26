#include "config.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "json.h"

namespace {

cv::Point2d pointFromArray(const minjson::Value& v) {
    if (!v.isArray() || v.size() != 2) throw std::runtime_error("config: expected [x, y] point");
    return {v.at(0).number, v.at(1).number};
}

cv::Scalar scalarFromArray(const minjson::Value& v) {
    if (!v.isArray() || v.size() != 3) throw std::runtime_error("config: expected [h, s, v] triple");
    return {v.at(0).number, v.at(1).number, v.at(2).number};
}

std::string num(double d) {
    std::ostringstream os;
    os.precision(6);
    os << std::fixed << d;
    return os.str();
}

}  // namespace

std::vector<cv::Point2d> SetupConfig::defaultCanonicalMagnets() {
    // Equilateral triangle on an 800x800 board: M0 top, M1 bottom-left,
    // M2 bottom-right (centroid 400,450, circumradius 260).
    return {{400, 190}, {175, 580}, {625, 580}};
}

std::vector<cv::Point2d> SetupConfig::canonicalEquilibria() const {
    // Attraction: the equilibrium points are the magnets themselves.
    return canonical_magnets.empty() ? defaultCanonicalMagnets() : canonical_magnets;
}

SetupConfig SetupConfig::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("config: cannot open '" + path + "'");
    std::stringstream buf;
    buf << in.rdbuf();

    minjson::Value root;
    try {
        root = minjson::parse(buf.str());
    } catch (const std::exception& e) {
        throw std::runtime_error("config: " + path + ": " + e.what());
    }
    if (!root.isObject()) throw std::runtime_error("config: root must be an object");

    SetupConfig cfg;
    if (root.has("board_size")) cfg.board_size = static_cast<int>(root.at("board_size").number);

    if (root.has("homography") && !root.at("homography").isNull()) {
        const minjson::Value& h = root.at("homography");
        if (!h.isArray() || h.size() != 9) throw std::runtime_error("config: homography must be 9 numbers");
        cfg.homography = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) cfg.homography.at<double>(i / 3, i % 3) = h.at(i).number;
    }

    if (root.has("magnets")) {
        const minjson::Value& m = root.at("magnets");
        for (size_t i = 0; i < m.size(); ++i) cfg.magnets.push_back(pointFromArray(m.at(i)));
    }
    if (root.has("valleys")) {
        const minjson::Value& v = root.at("valleys");
        for (size_t i = 0; i < v.size(); ++i) cfg.valleys.push_back(pointFromArray(v.at(i)));
    }
    if (root.has("layout_ref_bary")) {
        const minjson::Value& b = root.at("layout_ref_bary");
        for (size_t i = 0; i < b.size(); ++i) cfg.layout_ref_bary.push_back(b.at(i).number);
    }
    if (root.has("canonical_magnets")) {
        const minjson::Value& m = root.at("canonical_magnets");
        for (size_t i = 0; i < m.size(); ++i) cfg.canonical_magnets.push_back(pointFromArray(m.at(i)));
    }

    if (root.has("detection_mode")) cfg.detection_mode = root.at("detection_mode").string;
    if (root.has("hsv_lower")) cfg.hsv_lower = scalarFromArray(root.at("hsv_lower"));
    if (root.has("hsv_upper")) cfg.hsv_upper = scalarFromArray(root.at("hsv_upper"));
    if (root.has("bright_threshold")) cfg.bright_threshold = root.at("bright_threshold").number;
    if (root.has("min_blob_area")) cfg.min_blob_area = root.at("min_blob_area").number;
    if (root.has("max_blob_area")) cfg.max_blob_area = root.at("max_blob_area").number;
    if (root.has("smooth_window")) cfg.smooth_window = static_cast<int>(root.at("smooth_window").number);
    if (root.has("max_gap_frames")) cfg.max_gap_frames = static_cast<int>(root.at("max_gap_frames").number);
    if (root.has("track_gate_px")) cfg.track_gate_px = root.at("track_gate_px").number;
    if (root.has("open_kernel")) cfg.open_kernel = static_cast<int>(root.at("open_kernel").number);
    if (root.has("magnet_v_lower")) cfg.magnet_v_lower = root.at("magnet_v_lower").number;
    if (root.has("magnet_v_upper")) cfg.magnet_v_upper = root.at("magnet_v_upper").number;
    if (root.has("magnet_s_upper")) cfg.magnet_s_upper = root.at("magnet_s_upper").number;
    if (root.has("magnet_min_area")) cfg.magnet_min_area = root.at("magnet_min_area").number;
    if (root.has("magnet_max_area")) cfg.magnet_max_area = root.at("magnet_max_area").number;
    if (root.has("magnet_dark_ring_v")) cfg.magnet_dark_ring_v = root.at("magnet_dark_ring_v").number;
    if (root.has("magnet_bright_core_v")) cfg.magnet_bright_core_v = root.at("magnet_bright_core_v").number;
    if (root.has("tape_board_width")) cfg.tape_board_width = root.at("tape_board_width").number;
    if (root.has("autocal_frames")) cfg.autocal_frames = static_cast<int>(root.at("autocal_frames").number);
    if (root.has("attract_reach_px")) cfg.attract_reach_px = root.at("attract_reach_px").number;
    if (root.has("settle_speed_px_s")) cfg.settle_speed_px_s = root.at("settle_speed_px_s").number;
    if (root.has("settle_window_s")) cfg.settle_window_s = root.at("settle_window_s").number;
    if (root.has("bounce_speed_px_s")) cfg.bounce_speed_px_s = root.at("bounce_speed_px_s").number;
    if (root.has("settle_avg_frames")) cfg.settle_avg_frames = static_cast<int>(root.at("settle_avg_frames").number);

    return cfg;
}

void SetupConfig::save(const std::string& path) const {
    std::ostringstream os;
    os << "{\n";
    os << "  \"board_size\": " << board_size << ",\n";

    if (homography.empty()) {
        os << "  \"homography\": null,\n";
    } else {
        os << "  \"homography\": [";
        for (int i = 0; i < 9; ++i) {
            if (i) os << ", ";
            os << num(homography.at<double>(i / 3, i % 3));
        }
        os << "],\n";
    }

    auto writePoints = [&](const char* key, const std::vector<cv::Point2d>& pts, bool last) {
        os << "  \"" << key << "\": [";
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i) os << ", ";
            os << "[" << num(pts[i].x) << ", " << num(pts[i].y) << "]";
        }
        os << "]" << (last ? "\n" : ",\n");
    };
    writePoints("magnets", magnets, false);
    writePoints("valleys", valleys, false);
    writePoints("canonical_magnets",
                canonical_magnets.empty() ? defaultCanonicalMagnets() : canonical_magnets,
                false);

    os << "  \"layout_ref_bary\": [";
    {
        std::vector<double> bary = layout_ref_bary;
        if (bary.empty()) bary = {0.428, 0.360};
        for (size_t i = 0; i < bary.size(); ++i) {
            if (i) os << ", ";
            os << num(bary[i]);
        }
    }
    os << "],\n";

    os << "  \"detection_mode\": \"" << detection_mode << "\",\n";
    os << "  \"hsv_lower\": [" << num(hsv_lower[0]) << ", " << num(hsv_lower[1]) << ", " << num(hsv_lower[2]) << "],\n";
    os << "  \"hsv_upper\": [" << num(hsv_upper[0]) << ", " << num(hsv_upper[1]) << ", " << num(hsv_upper[2]) << "],\n";
    os << "  \"bright_threshold\": " << num(bright_threshold) << ",\n";
    os << "  \"min_blob_area\": " << num(min_blob_area) << ",\n";
    os << "  \"max_blob_area\": " << num(max_blob_area) << ",\n";
    os << "  \"smooth_window\": " << smooth_window << ",\n";
    os << "  \"max_gap_frames\": " << max_gap_frames << ",\n";
    os << "  \"track_gate_px\": " << num(track_gate_px) << ",\n";
    os << "  \"open_kernel\": " << open_kernel << ",\n";
    os << "  \"magnet_v_lower\": " << num(magnet_v_lower) << ",\n";
    os << "  \"magnet_v_upper\": " << num(magnet_v_upper) << ",\n";
    os << "  \"magnet_s_upper\": " << num(magnet_s_upper) << ",\n";
    os << "  \"magnet_min_area\": " << num(magnet_min_area) << ",\n";
    os << "  \"magnet_max_area\": " << num(magnet_max_area) << ",\n";
    os << "  \"magnet_dark_ring_v\": " << num(magnet_dark_ring_v) << ",\n";
    os << "  \"magnet_bright_core_v\": " << num(magnet_bright_core_v) << ",\n";
    os << "  \"tape_board_width\": " << num(tape_board_width) << ",\n";
    os << "  \"autocal_frames\": " << autocal_frames << ",\n";
    os << "  \"attract_reach_px\": " << num(attract_reach_px) << ",\n";
    os << "  \"settle_speed_px_s\": " << num(settle_speed_px_s) << ",\n";
    os << "  \"settle_window_s\": " << num(settle_window_s) << ",\n";
    os << "  \"bounce_speed_px_s\": " << num(bounce_speed_px_s) << ",\n";
    os << "  \"settle_avg_frames\": " << settle_avg_frames << "\n";
    os << "}\n";

    std::ofstream out(path);
    if (!out) throw std::runtime_error("config: cannot write '" + path + "'");
    out << os.str();
}
