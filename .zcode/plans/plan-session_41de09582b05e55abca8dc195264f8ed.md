# Magnetic Chaos Pendulum — Software Plan

## Goal
A C++/OpenCV pipeline that takes a video of a pendulum drop (like physics-input.mp4), tracks the bob, detects when it settles, classifies which valley (equilibrium point) it stopped in, and logs the result. Phase 2 adds the basin-of-attraction map.

## Project structure (new CMake project in /Users/aayushgarg/CHAOS)

CHAOS/
├── CMakeLists.txt
├── config/
│   └── setup.json          # magnet positions, valley points, homography, thresholds
├── src/
│   ├── main.cpp            # CLI: --video <file> --config <file>
│   ├── video_source.{h,cpp}    # VideoCapture wrapper (file now, webcam-ready interface)
│   ├── calibrate.{h,cpp}       # 3-corner click → perspective warp
│   ├── tracker.{h,cpp}         # bob detection + tracking per frame
│   ├── state_machine.{h,cpp}   # SWINGING ↔ SETTLED logic
│   ├── valleys.{h,cpp}         # valley config + Euclidean classification
│   └── logger.{h,cpp}          # CSV output + trajectory dump
├── tools/ (phase 2)
│   └── basin_map.cpp       # renders basin map from results CSV
└── data/                   # results.csv, trajectories/*.csv

## Build stages

**Stage 1 — Calibration (interactive, once per setup)**
- Open a video frame; user clicks the 3 corners of the triangular board.
- Tool computes and saves the perspective-transform matrix to config/setup.json.
- Then user clicks each magnet's center; tool computes valley points as midpoints between adjacent magnets (3 magnets → 3 valleys) and saves them.
- Thresholds (settle speed, quiet duration, bob color/size) live in the same config, tunable without recompiling.

**Stage 2 — Bob detection & tracking**
- Convert each warped frame to HSV; detect bob by color (cv::inRange + largest contour + centroid). Recommended: high-saturation bob color (red/orange), or brightness threshold since the board is dark.
- Fallback: background subtraction (MOG2) if color tracking proves unreliable.
- Record (x, y, timestamp, detected/lost) every frame; smooth with a small moving average; interpolate gaps ≤ 5 frames.
- Output: per-run trajectory CSV (pixel coords + warped coordinates).

**Stage 3 — State machine**
- Velocity = displacement between consecutive warped points / dt.
- SWINGING until speed stays below settle_speed_px_s for settle_window_s (default 2–3 s), then SETTLED. Reset to SWINGING if speed spikes after settling.
- Flag degenerate runs (bob never detected, or lost > 20% of frames) as invalid instead of logging garbage.

**Stage 4 — Classification & logging**
- Once SETTLED: average position over last N frames = final point.
- Euclidean distance to each valley; nearest valley = winner.
- Append to data/results.csv: run_id, video_file, start_x, start_y, end_x, end_y, valley_id, distance, duration_s, valid.
- Live debug overlay window (warped feed + trail + valley markers), togglable with --debug.

**Stage 5 — Verification & polish**
- Test against the provided clip; tune HSV thresholds on real frames.
- Batch mode: --videos data/*.mp4.
- (Phase 2, after MVP works) basin_map.cpp: plot each run's start point colored by its valley.

## Dependencies
- OpenCV 4 (brew install opencv), CMake ≥ 3.16, C++17.

## Out of scope (for now)
- Physics simulation, live webcam mode (interface leaves room for it), automatic magnet detection.