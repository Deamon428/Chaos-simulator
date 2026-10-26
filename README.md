# Magnetic Chaos Pendulum — Tracking Software

C++/OpenCV pipeline that takes top-down videos of the magnetic pendulum,
tracks the bob, detects when it settles, classifies which magnet it was
captured by, and logs the result.

**Setup (attraction):** the base magnets ATTRACT the central pendulum magnet
(an earlier repulsion design was abandoned because the magnets were too weak
to show it). The equilibrium points are therefore the magnet positions
themselves: a finished run ends essentially on top of one of the magnets, and
the "basin of attraction" of each magnet is the region from which drops lead
to it.

**Magnet layout is the real, measured one** — the magnets are NOT evenly
spaced, so the canonical board frame (`canonical_magnets` in the config)
holds the actual measured positions from the reference video, and every
video's calibration maps its detected magnets onto that same layout. The
web simulator derives its board triangle from these positions too.

## Build

```bash
brew install opencv cmake     # once
cmake -S . -B build
cmake --build build
```

## Web UI (localhost, no terminal needed day-to-day)

```bash
python3 server.py             # zero dependencies; then open http://localhost:8000
```

Three tabs:
- **Simulator** — a live physics model of the attraction pendulum in the
  browser (canvas): drag the bob anywhere inside the string's reach and
  release; sliders for attraction strength, damping, and string length.
  "Compute basin map" simulates a grid of drop points and colors each by the
  magnet that captures it — the fractal basin boundary of the chaos.
- **Real runs & basin map** — plots every analyzed video's drop point colored
  by its capturing magnet (gray = invalid run), the results table, and the
  calibration preview images.
- **Videos** — click Analyze on any clip in `Videos/` (or "Analyze all"); the
  server runs the C++ tracker and streams the log back. Same engine and
  outputs as the terminal commands below.

The frontend is a single plain file you can freely restyle: `web/index.html`.
The server (`server.py`) is stdlib-only Python and only wraps the binaries.

Two binaries: `build/pendulum-track` (the pipeline) and `build/pendulum-calibrate`
(manual click calibration — usually not needed, see below).

## Usage

```bash
# One video (auto-calibrated):
./build/pendulum-track --video "Videos/your-video.mp4" --autocal --debug

# The whole folder:
./build/pendulum-track --videos Videos --autocal

# Skip an angled/chaotic intro (frames before the top-down view):
./build/pendulum-track --video f.mp4 --autocal --start-frame 750
```

Outputs:
- `data/results.csv` — one row per run: start/end points, nearest magnet
  (`magnet_id`), distance, duration, detection ratio, valid flag, diagnostics.
- `data/trajectories/run_N_*.csv` — full per-frame bob trajectory.
- with `--dump-dir D` — warped frames with the trail/magnets overlaid,
  plus a `cal_*` calibration preview. Inspect these to verify a run.

## How it works (per video)

1. **Auto-calibration** — no fixed camera: every video is calibrated
   independently. The 3 base magnets are detected as small low-saturation
   blobs on the dark board (sampled across many frames, clustered by position;
   a resting bob is excluded by its brighter core). The best-fitting triangle
   is matched against the canonical layout and a homography is fitted from the
   3 magnets (+ the hub tape found as the large white blob) to an 800×800
   board frame. The equilibrium points (`valleys` in the config) are the
   magnet positions themselves.
2. **Tracking** — each frame is warped to the board; the bob is detected by
   HSV color (white/gray band), largest blob in an area band, restricted to
   the board region, with nearest-to-previous-position gating and static-blob
   rejection (tape, wood highlights, and off-magnet static spots are
   catalogued and ignored; the magnets themselves stay trackable because the
   bob must be able to reach them).
3. **State machine** — SWINGING until the bob stays under
   `settle_speed_px_s` for `settle_window_s`; a hard bounce re-opens the
   state. A quiet tail at the end of a clip (≥1.5 s) counts as settled.
4. **Classification** — nearest magnet by Euclidean distance; row appended
   to `results.csv`. Runs are marked invalid when the bob was never seen,
   detected < 80 % of frames, or the rest point ended farther than
   `attract_reach_px` (100 px) from every magnet — with working attraction
   that means the bob was never captured (string too long / no real drop).

All thresholds live in `config/setup.json`.

## Findings from the current video set (recorded 2026-10-26, repulsion-era)

These clips predate the attraction change, and the software's verdict
under attraction physics matches what was already visible: in almost every
clip the bob hangs nearly still mid-board and is **never captured** —
consistent with the magnets being too weak at that string height (the reason
the design was flipped to attraction). One clip (`19.42.53`) shows a genuine
capture by magnet M0 (rest 68 px from it).

Also:
- The videos fall into **two rig configurations**: the magnets were moved
  between the `19.39–19.42` and the `19.53–19.54` recordings. Magnet IDs are
  comparable within a group, not across groups (the autocal fingerprint
  `layout_ref_bary` in the config was measured on the first group).
- `physics-input.mp4` has camera drift and an angled intro; its auto-calibration
  is unreliable — use `--start-frame 750` or calibrate manually with
  `pendulum-calibrate`.

## Manual calibration (optional)

```bash
./build/pendulum-calibrate --video <file>          # click 3 corners, then magnets
```

Writes `config/setup.json`; `pendulum-track` then runs without `--autocal`.

## Phase 2 (not yet built)

`tools/basin_map.cpp` — plot each run's start point colored by the magnet that
captured it, to render the basin-of-attraction map once enough valid runs exist.
