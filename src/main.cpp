// pendulum-track: process one or more pendulum-drop videos.
//
//   pendulum-track --video <file> [--videos <dir>] [--config config/setup.json]
//                  [--out data/results.csv] [--traj-dir data/trajectories]
//                  [--debug] [--start-frame N]
//
// For each video: warps frames onto the calibrated board, tracks the bob,
// waits for the settle state machine, classifies the nearest valley, and
// appends a row to the results CSV plus a full trajectory CSV.

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "autocal.h"
#include "config.h"
#include "logger.h"
#include "state_machine.h"
#include "tracker.h"
#include "valleys.h"
#include "video_source.h"

namespace fs = std::filesystem;

namespace {

struct Args {
    std::vector<std::string> videos;
    std::string config = "config/setup.json";
    std::string out = "data/results.csv";
    std::string traj_dir = "data/trajectories";
    std::string dump_dir;        // when set, write warped frames + overlays here
    int dump_every = 30;         // dump every Nth frame
    bool dump_mask = false;      // tint the detection mask red on dumps
    bool autocal = false;        // per-video magnet-based calibration
    int label_offset = 0;        // rotate autocal magnet labeling
    bool debug = false;
    int start_frame = 0;
    std::string probe;           // "x,y,r": print HSV stats in a circle, then exit
};

struct RunOutcome {
    bool ok = false;
    bool settled = false;
    cv::Point2d start{-1, -1}, end{-1, -1};
    int magnet_id = -1;
    double dist = 0;
    double duration_s = 0;
    double detection_ratio = 0;
    double fps = 30.0;
    std::string note;
    std::vector<std::pair<int, cv::Point2d>> history;  // (frame no, smoothed point)
};

void drawOverlay(cv::Mat& img, const std::vector<std::pair<int, cv::Point2d>>& history,
                 const SetupConfig& cfg, const SettleDetector& settle, double speed);

std::vector<std::string> listVideos(const std::string& dir) {
    std::vector<std::string> out;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        std::string ext = e.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".mp4" || ext == ".mov" || ext == ".avi" || ext == ".mkv")
            out.push_back(e.path().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

RunOutcome processVideo(const std::string& video_path, const SetupConfig& cfg,
                        const Args& args) {
    RunOutcome res;
    VideoSource src(video_path);
    if (!src.isOpened()) {
        res.note = src.error();
        return res;
    }
    const double fps = src.fps();
    res.fps = fps;
    const double dt = 1.0 / fps;
    const std::string stem = fs::path(video_path).stem().string();

    if (args.start_frame > 0 && !src.seek(args.start_frame))
        std::cerr << "warning: could not seek to frame " << args.start_frame << "\n";

    Tracker tracker(cfg);
    SettleDetector settle(cfg);
    cv::Mat frame, warped;    int frame_no = args.start_frame, frames = 0, detected = 0;
    cv::Point2d prev;
    double speed = 0.0;
    bool have_prev = false;

    while (src.read(frame)) {
        cv::warpPerspective(frame, warped, cfg.homography, {cfg.board_size, cfg.board_size});
        cv::Point2d pos;
        bool ok = tracker.step(warped, pos);
        ++frames;
        if (ok) {
            ++detected;
            if (have_prev) {
                speed = cv::norm(pos - prev) / dt;
                settle.update(dt, speed);
            }
            prev = pos;
            have_prev = true;
            res.history.emplace_back(frame_no, pos);
        }
        res.duration_s = frames * dt;

        if (settle.state() == SettleDetector::SETTLED) {
            res.settled = true;
            if (args.debug) {
                drawOverlay(warped, res.history, cfg, settle, speed);
                cv::imshow("pendulum-track", warped);
                cv::waitKey(500);
            }
            break;
        }

        if (args.debug) {
            drawOverlay(warped, res.history, cfg, settle, speed);
            cv::imshow("pendulum-track", warped);
            if ((cv::waitKey(1) & 0xff) == 'q') {
                res.note = "aborted by user";
                break;
            }
        }
        if (!args.dump_dir.empty() && frame_no % args.dump_every == 0) {
            cv::Mat vis = warped.clone();
            if (args.dump_mask) {
                cv::Mat mask;
                tracker.debugMask(warped, mask);
                std::vector<cv::Mat> ch = {cv::Mat::zeros(mask.size(), CV_8U), mask * 0.4, mask};
                cv::Mat tint;
                cv::merge(ch, tint);
                cv::addWeighted(vis, 0.6, tint, 0.4, 0, vis);
            }
            drawOverlay(vis, res.history, cfg, settle, speed);
            std::ostringstream name;
            name << args.dump_dir << "/" << stem << "_f" << frame_no << ".png";
            cv::imwrite(name.str(), vis);
        }
        ++frame_no;
    }
    cv::destroyAllWindows();

    res.detection_ratio = frames > 0 ? static_cast<double>(detected) / frames : 0.0;
    if (res.history.empty()) {
        res.note = res.note.empty() ? "bob never detected" : res.note + "; bob never detected";
        return res;
    }
    res.ok = true;
    res.start = res.history.front().second;

    // Final resting point: mean of the last settle_avg_frames detections.
    size_t n = std::min<size_t>(cfg.settle_avg_frames, res.history.size());
    cv::Point2d sum(0, 0);
    for (auto it = res.history.end() - n; it != res.history.end(); ++it) sum += it->second;
    res.end = sum * (1.0 / n);

    // Clips often end right after the bob stops: accept a quiet tail as a
    // settle instead of discarding the run.
    if (!res.settled && settle.quietFor() > 1.5) {
        res.settled = true;
        res.note = "settled at video end";
    }
    if (!res.settled) res.note = "unsettled at end of video";
    return res;
}

void drawOverlay(cv::Mat& img, const std::vector<std::pair<int, cv::Point2d>>& history,
                 const SetupConfig& cfg, const SettleDetector& settle, double speed) {
    const cv::Scalar green(0, 255, 0), red(0, 0, 255), blue(255, 0, 0);
    for (size_t i = 1; i < history.size(); ++i)
        cv::line(img, history[i - 1].second, history[i].second, green, 1);
    // With attraction the equilibrium points ARE the magnets: one marker set.
    for (size_t i = 0; i < cfg.valleys.size(); ++i) {
        cv::circle(img, cfg.valleys[i], 8, red, 2);
        cv::putText(img, "M" + std::to_string(i), cfg.valleys[i] + cv::Point2d(10, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, blue, 2);
    }
    std::ostringstream os;
    os << (settle.state() == SettleDetector::SETTLED ? "SETTLED" : "SWINGING")
       << "  speed=" << static_cast<int>(speed) << " px/s"
       << "  quiet=" << settle.quietFor() << "s";
    cv::putText(img, os.str(), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--video" && i + 1 < argc) args.videos.push_back(argv[++i]);
        else if (a == "--videos" && i + 1 < argc) {
            try {
                auto more = listVideos(argv[++i]);
                args.videos.insert(args.videos.end(), more.begin(), more.end());
            } catch (const std::exception& e) {
                std::cerr << "error: " << e.what() << "\n";
                return 2;
            }
        } else if (a == "--config" && i + 1 < argc) args.config = argv[++i];
        else if (a == "--out" && i + 1 < argc) args.out = argv[++i];
        else if (a == "--traj-dir" && i + 1 < argc) args.traj_dir = argv[++i];
        else if (a == "--debug") args.debug = true;
        else if (a == "--autocal") args.autocal = true;
        else if (a == "--label-offset" && i + 1 < argc) args.label_offset = std::atoi(argv[++i]);
        else if (a == "--dump-dir" && i + 1 < argc) args.dump_dir = argv[++i];
        else if (a == "--dump-every" && i + 1 < argc) args.dump_every = std::atoi(argv[++i]);
        else if (a == "--dump-mask") args.dump_mask = true;
        else if (a == "--start-frame" && i + 1 < argc) args.start_frame = std::atoi(argv[++i]);
        else if (a == "--probe" && i + 1 < argc) args.probe = argv[++i];
        else { std::cerr << "unknown/incomplete arg: " << a << "\n"; return 2; }
    }
    if (args.videos.empty()) {
        std::cerr << "usage: pendulum-track --video <file> | --videos <dir> [--config c] "
                     "[--out csv] [--traj-dir dir] [--debug] [--start-frame n]\n";
        return 2;
    }

    SetupConfig cfg;
    try {
        cfg = SetupConfig::load(args.config);
    } catch (const std::exception& e) {
        std::cerr << "error: " + std::string(e.what()) << "\n";
        return 1;
    }

    // --probe x,y,r: HSV statistics inside a circle of the raw frame (image
    // coordinates) at --start-frame. Used to tune the color bands.
    if (!args.probe.empty()) {
        double px = 0, py = 0, pr = 0;
        if (std::sscanf(args.probe.c_str(), "%lf,%lf,%lf", &px, &py, &pr) != 3) {
            std::cerr << "error: --probe expects x,y,r\n";
            return 2;
        }
        VideoSource src(args.videos.front());
        if (!src.isOpened() || (args.start_frame > 0 && !src.seek(args.start_frame))) {
            std::cerr << "error: cannot open/seek video\n";
            return 1;
        }
        cv::Mat frame;
        if (!src.read(frame)) {
            std::cerr << "error: cannot read frame\n";
            return 1;
        }
        cv::Mat hsv, mask = cv::Mat::zeros(frame.size(), CV_8U);
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::circle(mask, {static_cast<int>(px), static_cast<int>(py)}, static_cast<int>(pr), 255, -1);
        std::vector<double> hs, ss, vs;
        for (int y = 0; y < hsv.rows; ++y)
            for (int x = 0; x < hsv.cols; ++x)
                if (mask.at<uchar>(y, x)) {
                    const cv::Vec3b& v = hsv.at<cv::Vec3b>(y, x);
                    hs.push_back(v[0]);
                    ss.push_back(v[1]);
                    vs.push_back(v[2]);
                }
        auto pct = [](std::vector<double>& v, double p) {
            std::sort(v.begin(), v.end());
            return v[static_cast<size_t>(p / 100.0 * (v.size() - 1))];
        };
        std::cout << "probe(" << px << "," << py << "," << pr << ") n=" << vs.size()
                  << "  H p5/p50/p95: " << pct(hs, 5) << "/" << pct(hs, 50) << "/" << pct(hs, 95)
                  << "  S: " << pct(ss, 5) << "/" << pct(ss, 50) << "/" << pct(ss, 95)
                  << "  V: " << pct(vs, 5) << "/" << pct(vs, 50) << "/" << pct(vs, 95) << "\n";
        return 0;
    }
    if (!cfg.calibrated() && !args.autocal) {
        std::cerr << "error: no homography in " << args.config
                  << " — run pendulum-calibrate or pass --autocal\n";
        return 1;
    }
    if (!args.autocal && cfg.valleys.empty()) {
        std::cerr << "error: no valleys in " << args.config << "\n";
        return 1;
    }

    if (!args.dump_dir.empty()) fs::create_directories(args.dump_dir);
    fs::create_directories(args.traj_dir);
    ResultsLogger logger(args.out);

    int failures = 0;
    for (const auto& video : args.videos) {
        std::cout << "== " << video << "\n";
        SetupConfig effective = cfg;
        if (args.autocal) {
            AutoCalResult cal = autoCalibrate(video, cfg, args.label_offset, args.start_frame);
            if (!cal.ok) {
                ++failures;
                std::cout << "   AUTOCAL FAILED: " << cal.error << "\n";
                continue;
            }
            effective.homography = cal.homography;
            effective.valleys = cfg.canonicalEquilibria();
            // Map static bright blobs (tape, magnets, highlights) to board
            // coords so the tracker can ignore them.
            if (!cal.static_blobs_img.empty()) {
                std::vector<cv::Point2f> img_pts, board_pts;
                for (const auto& p : cal.static_blobs_img) img_pts.push_back(p);
                cv::perspectiveTransform(img_pts, board_pts, cal.homography);
                for (const auto& p : board_pts) effective.static_blobs.push_back(p);
            }
            std::cout << "   autocal magnets (image px):";
            for (const auto& m : cal.magnets_img)
                std::cout << " (" << static_cast<int>(m.x) << "," << static_cast<int>(m.y) << ")";
            std::cout << "\n";
            if (!args.dump_dir.empty() && !cal.sample_frame.empty()) {
                // sample_frame is already in the provisional board frame.
                cv::Mat vis = cal.sample_frame.clone();
                std::vector<std::pair<int, cv::Point2d>> empty;
                SettleDetector dummy(effective);
                drawOverlay(vis, empty, effective, dummy, 0.0);
                cv::imwrite(args.dump_dir + "/cal_" + fs::path(video).stem().string() + ".png",
                            vis);
            }
        }

        RunOutcome res = processVideo(video, effective, args);
        if (!res.ok) {
            ++failures;
            std::cout << "   FAILED: " << res.note << "\n";
            continue;
        }
        res.magnet_id = classifyAttractor(effective.valleys, res.end, res.dist);
        bool valid = res.settled && res.detection_ratio >= 0.8;
        // Attraction setup: a genuine run ends essentially ON a magnet (the
        // bob hangs above it). A rest point far from every magnet means the
        // attraction never engaged — string too long (Z too high) or no drop.
        double magnet_dist = -1;
        for (const auto& m : effective.valleys) {
            double d = cv::norm(res.end - m);
            if (magnet_dist < 0 || d < magnet_dist) magnet_dist = d;
        }
        bool out_of_reach = magnet_dist < 0 || magnet_dist > effective.attract_reach_px;
        if (out_of_reach) {
            res.note += (res.note.empty() ? std::string() : std::string("; ")) +
                        "rest point far from all magnets (attraction not engaged or no drop)";
            valid = false;
        }
        if (!valid) ++failures;

        int run_id = logger.nextRunId();
        std::string stem = fs::path(video).stem().string();
        std::string traj_path =
            args.traj_dir + "/run_" + std::to_string(run_id) + "_" + stem + ".csv";

        std::ostringstream records;
        records << "frame,t_sec,x_board,y_board\n";
        for (const auto& [f, p] : res.history) {
            records << f << ',' << (f / res.fps) << ',' << p.x << ',' << p.y << '\n';
        }
        ResultsLogger::writeTrajectory(traj_path, video, records.str());

        logger.append({run_id,
                       fs::path(video).filename().string(),
                       res.start.x, res.start.y,
                       res.end.x, res.end.y,
                       res.magnet_id, res.dist,
                       res.duration_s, res.detection_ratio,
                       valid ? 1 : 0,
                       res.note});

        std::cout << "   start=(" << res.start.x << ", " << res.start.y << ")  end=("
                  << res.end.x << ", " << res.end.y << ")\n"
                  << "   magnet M" << res.magnet_id << "  dist=" << res.dist << " px"
                  << "  duration=" << res.duration_s << "s"
                  << "  detection=" << res.detection_ratio * 100 << "%"
                  << (valid ? "  [valid]" : "  [INVALID]") << "\n"
                  << "   trajectory: " << traj_path << "\n";
        if (!res.note.empty()) std::cout << "   note: " << res.note << "\n";
    }

    std::cout << "results written to " << args.out << "\n";
    return failures > 0 ? 1 : 0;
}
