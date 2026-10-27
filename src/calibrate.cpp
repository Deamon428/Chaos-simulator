// Interactive calibration: click the 3 board corners on a video frame,
// then click each magnet on the warped view. Writes config/setup.json.
//
//   pendulum-calibrate --video <file> [--config config/setup.json]
//                      [--magnets 3] [--board 800] [--frame 0]
//
// Keys: left-click = add point, u = undo, d = done.

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// OpenCV 5 moved getAffineTransform out of imgproc into geometry/2d.hpp.
#include <opencv2/geometry/2d.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"
#include "valleys.h"
#include "video_source.h"

namespace {

struct ClickState {
  std::vector<cv::Point2d> points;
  cv::Mat base; // image shown underneath the markers
  std::string hint;
};

void drawPoints(cv::Mat &img, const std::vector<cv::Point2d> &pts,
                const cv::Scalar &color) {
  for (size_t i = 0; i < pts.size(); ++i) {
    cv::circle(img, pts[i], 6, color, 2);
    cv::putText(img, std::to_string(i + 1), pts[i] + cv::Point2d(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  }
  if (pts.size() >= 2) {
    for (size_t i = 1; i < pts.size(); ++i)
      cv::line(img, pts[i - 1], pts[i], color, 1);
  }
}

void onMouse(int event, int x, int y, int, void *userdata) {
  if (event != cv::EVENT_LBUTTONDOWN)
    return;
  auto *st = static_cast<ClickState *>(userdata);
  st->points.push_back({static_cast<double>(x), static_cast<double>(y)});
}

// Prompts for `required` clicks. Returns false if the window was closed early.
bool collectClicks(const std::string &window, ClickState &st,
                   const std::string &doneHint, size_t required,
                   const cv::Scalar &color) {
  cv::setMouseCallback(window, onMouse, &st);
  while (true) {
    cv::Mat img = st.base.clone();
    drawPoints(img, st.points, color);
    std::string hint = st.points.size() >= required
                           ? doneHint
                           : "Click point " +
                                 std::to_string(st.points.size() + 1) +
                                 " (need " + std::to_string(required) + ")";
    cv::putText(img, hint, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                {0, 255, 255}, 2);
    cv::imshow(window, img);

    int key = cv::waitKey(30) & 0xff;
    if (key == 'd' && st.points.size() >= required)
      return true;
    if (key == 'u' && !st.points.empty())
      st.points.pop_back();
    if (key == 27 || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1)
      return false;
  }
}

// Canonical destination triangle inside the S x S board: top, bottom-left,
// bottom-right (order must match the clicked corners, which the user gives
// clockwise).
std::vector<cv::Point2f> canonicalTriangle(int s) {
  return {{s / 2.0f, 0.1f * s}, {0.1f * s, 0.9f * s}, {0.9f * s, 0.9f * s}};
}

} // namespace

int main(int argc, char **argv) {
  std::string video_path, config_path = "config/setup.json";
  int num_magnets = 3, board_size = 800, frame_index = 0;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--video" && i + 1 < argc)
      video_path = argv[++i];
    else if (a == "--config" && i + 1 < argc)
      config_path = argv[++i];
    else if (a == "--magnets" && i + 1 < argc)
      num_magnets = std::atoi(argv[++i]);
    else if (a == "--board" && i + 1 < argc)
      board_size = std::atoi(argv[++i]);
    else if (a == "--frame" && i + 1 < argc)
      frame_index = std::atoi(argv[++i]);
    else {
      std::cerr << "unknown/incomplete arg: " << a << "\n";
      return 2;
    }
  }
  if (video_path.empty()) {
    std::cerr << "usage: pendulum-calibrate --video <file> [--config c] "
                 "[--magnets n] "
                 "[--board px] [--frame i]\n";
    return 2;
  }

  // Keep previously tuned thresholds; calibration replaces geometry only.
  SetupConfig cfg;
  try {
    cfg = SetupConfig::load(config_path);
  } catch (const std::exception &) {
    cfg = SetupConfig{};
  }
  cfg.board_size = board_size;

  VideoSource src(video_path);
  if (!src.isOpened()) {
    std::cerr << "error: " << src.error() << "\n";
    return 1;
  }
  if (frame_index > 0 && !src.seek(frame_index)) {
    std::cerr << "warning: could not seek to frame " << frame_index << "\n";
  }
  cv::Mat frame;
  if (!src.read(frame)) {
    std::cerr << "error: could not read a frame from the video\n";
    return 1;
  }

  // --- Step 1: board corners ---
  const std::string corners_win = "calibrate: board corners";
  cv::namedWindow(corners_win, cv::WINDOW_NORMAL);
  ClickState corner_state;
  corner_state.base = frame.clone();
  if (!collectClicks(corners_win, corner_state, "Press d to confirm corners", 3,
                     {0, 255, 0})) {
    std::cerr << "calibration aborted\n";
    return 1;
  }

  std::vector<cv::Point2f> src_pts, dst_pts = canonicalTriangle(board_size);
  for (const auto &p : corner_state.points)
    src_pts.push_back(p);
  cv::Mat affine = cv::getAffineTransform(src_pts, dst_pts);
  cv::Mat homography = cv::Mat::eye(3, 3, CV_64F);
  affine.convertTo(homography(cv::Rect(0, 0, 3, 2)), CV_64F);
  cfg.homography = homography;

  // --- Step 2: magnets on the warped view ---
  cv::Mat warped;
  cv::warpPerspective(frame, warped, cfg.homography, {board_size, board_size});
  const std::string mag_win = "calibrate: magnets (warped)";
  cv::namedWindow(mag_win, cv::WINDOW_NORMAL);
  ClickState mag_state;
  mag_state.base = warped.clone();
  if (!collectClicks(mag_win, mag_state, "Press d to confirm magnets",
                     static_cast<size_t>(num_magnets), {0, 0, 255})) {
    std::cerr << "calibration aborted\n";
    return 1;
  }
  cfg.magnets.assign(mag_state.points.begin(), mag_state.points.end());
  cfg.valleys = computeEquilibria(cfg.magnets);
  if (cfg.valleys.empty()) {
    std::cerr << "error: need at least 2 magnets\n";
    return 1;
  }

  cfg.save(config_path);
  cv::destroyAllWindows();

  // Preview image for later reference (and headless verification).
  cv::Mat preview = warped.clone();
  drawPoints(preview, cfg.valleys, {255, 0, 0});
  for (size_t i = 0; i < cfg.valleys.size(); ++i)
    cv::putText(preview, "V" + std::to_string(i),
                cfg.valleys[i] + cv::Point2d(10, 10), cv::FONT_HERSHEY_SIMPLEX,
                0.6, {255, 0, 0}, 2);
  cv::imwrite("data/calibration_preview.png", preview);

  std::cout << "calibration saved to " << config_path << "\n";
  std::cout << "magnets (" << cfg.magnets.size() << "):";
  for (const auto &m : cfg.magnets)
    std::cout << " (" << m.x << ", " << m.y << ")";
  std::cout << "\nequilibria (magnets, " << cfg.valleys.size() << "):";
  for (const auto &v : cfg.valleys)
    std::cout << " (" << v.x << ", " << v.y << ")";
  std::cout << "\npreview written to data/calibration_preview.png\n";
  return 0;
}
