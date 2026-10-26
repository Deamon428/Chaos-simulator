# Pendulum Lab — Fixes + Prediction & Photo Features

## 1. Real runs & basin map tab (bug fixes + real interactivity)
- **Refresh feedback:** after reload, show "updated HH:MM:SS · N runs" next to the button so it visibly does something.
- **Row ↔ dot interaction:** clicking a table row highlights its drop dot on the canvas (ring) and vice versa.
- **Invalid filter:** checkbox to hide/show invalid runs (applies to dots and table).
- **Trajectory view:** selecting a run fetches its trajectory CSV and draws the bob's path on the canvas (time-gradient polyline, start = white dot, end = magnet-colored dot).
- **Staleness hardening:** cache-busting query params on all API fetches, a visible red banner if any JS error occurs, and data loaded on page load (not only on tab click).
- server.py: new endpoints `GET /api/trajectories` (list) and `GET /api/traj?name=` (serve one CSV, sanitized).

## 2. Simulator damping
- Default damping 1.0 → **0.25** (slider range 0–2), force default 12. The readout already shows live speed/distance; keep semantics (damping is a per-second exponential drag).

## 3. Videos tab: proper results UI
- After analysis, render **result cards** instead of the terminal log: video name, valid/invalid badge with reason, start → end, captured magnet (colored), distance, duration, detection %, and a **mini trajectory canvas** (path drawn from the trajectory CSV).
- Raw terminal log moves into a collapsed `<details>` block for debugging.

## 4. Fallback prediction for invalid recordings
- One shared JS simulation step function used by the simulator, basin map, and predictor (single source of truth).
- **Auto-tuning ("optimally"):** grid-search attraction strength × damping scored against the *valid* runs (predicted magnet == observed magnet); best parameters are used for predictions and displayed.
- Runs tab button **"Simulate predictions for invalid runs"**: for each invalid run that has a measured start point, simulate the path (dashed, drawn on the canvas) and show the predicted magnet in a new table column.
- **Append to results.csv** (your choice): new endpoint `POST /api/append_prediction` writes the prediction as an extra row — video name gets `[SIMULATED]`, the note records the tuned parameters; real rows untouched. Results.csv schema otherwise unchanged.

## 5. Photo analysis (upload → layout → simulated path + expected stop)
- **C++:** new executable `pendulum-photo` (reuses autocal's detection helpers): loads an image, finds the hub + 3 magnets (single-frame version of the video detection: color band, area, dark-ring, bright-core, equilateral+hub-centered triple scoring), computes the affine photo → canonical layout, writes JSON, and saves an annotated preview.
- **server.py:** `POST /api/photo` (raw image upload) → saves to `data/uploads/`, runs the tool, returns detected positions + annotated image.
- **Frontend:** new **Photo tab** — upload a top-down photo of the board → detected magnets drawn on your photo → **click the drop point** on the photo → the predicted path is simulated in canonical space and drawn back onto the photo, with the expected stopping magnet highlighted and stats shown.

## Verification
Rebuild C++, restart server, then browser-test everything: runs-tab interactions and trajectory view, prediction on the currently invalid runs, videos-tab result cards, and the photo tab using a frame extracted from one of your videos as the test photo. Commit at the end.

**Notes:** results.csv schema stays unchanged (predictions are appended rows marked `[SIMULATED]`). Predictions assume the canonical (group-1) magnet layout — if you move the magnets, re-record and the autocal refresh handles it.