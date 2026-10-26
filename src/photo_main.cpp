// pendulum-photo: detect the magnet layout on a top-down photo of the board.
//
//   pendulum-photo --image <file> [--config config/setup.json]
//                  [--out out.json] [--annotated out.png] [--label-offset N]
//
// Writes a JSON result (ok, error, hub, magnets in photo px, canonical
// magnets, homography photo->canonical) and an annotated preview image.

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "autocal.h"
#include "config.h"

int main(int argc, char** argv) {
    std::string image_path, config_path = "config/setup.json", out_path, annotated_path;
    int label_offset = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--image" && i + 1 < argc) image_path = argv[++i];
        else if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--out" && i + 1 < argc) out_path = argv[++i];
        else if (a == "--annotated" && i + 1 < argc) annotated_path = argv[++i];
        else if (a == "--label-offset" && i + 1 < argc) label_offset = std::atoi(argv[++i]);
        else { std::cerr << "unknown/incomplete arg: " << a << "\n"; return 2; }
    }
    if (image_path.empty()) {
        std::cerr << "usage: pendulum-photo --image <file> [--config c] "
                     "[--out json] [--annotated png] [--label-offset N]\n";
        return 2;
    }

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "error: could not read image '" << image_path << "'\n";
        return 1;
    }

    std::string err;
    SetupConfig cfg;
    try {
        cfg = SetupConfig::load(config_path);
    } catch (const std::exception& e) {
        err = e.what();
    }

    PhotoCalResult res;
    if (!err.empty()) {
        res.error = err;
    } else {
        res = photoCalibrate(img, cfg, label_offset);
    }

    std::ostringstream os;
    os << "{\n";
    auto num = [](double d) {
        std::ostringstream o; o.precision(6); o << std::fixed << d; return o.str();
    };
    if (!res.ok) {
        os << "  \"ok\": false,\n  \"error\": \"" << res.error << "\"\n";
        os << "}\n";
    } else {
        std::vector<cv::Point2d> canon = cfg.canonical_magnets.empty()
                                             ? SetupConfig::defaultCanonicalMagnets()
                                             : cfg.canonical_magnets;
        os << "  \"ok\": true,\n";
        os << "  \"board_size\": " << cfg.board_size << ",\n";
        os << "  \"hub_image\": [" << num(res.hub_image.x) << ", "
           << num(res.hub_image.y) << "],\n";
        os << "  \"magnets_image\": [";
        for (size_t i = 0; i < res.magnets_image.size(); ++i) {
            if (i) os << ", ";
            os << "[" << num(res.magnets_image[i].x) << ", "
               << num(res.magnets_image[i].y) << "]";
        }
        os << "],\n";
        os << "  \"magnets_canonical\": [";
        for (size_t i = 0; i < canon.size(); ++i) {
            if (i) os << ", ";
            os << "[" << num(canon[i].x) << ", " << num(canon[i].y) << "]";
        }
        os << "],\n  \"homography\": [";
        for (int i = 0; i < 9; ++i) {
            if (i) os << ", ";
            os << num(res.homography.at<double>(i / 3, i % 3));
        }
        os << "]\n}\n";
    }

    bool ok = res.ok;
    if (out_path.empty()) {
        std::cout << os.str();
    } else {
        std::ofstream f(out_path);
        f << os.str();
        if (!f) { std::cerr << "error: cannot write " << out_path << "\n"; return 1; }
    }
    if (res.ok && !annotated_path.empty()) cv::imwrite(annotated_path, res.annotated);
    return ok ? 0 : 1;
}
