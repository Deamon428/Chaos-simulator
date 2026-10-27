# Project Progress — Magnetic Chaos Pendulum (updated 2026-10-27)

## Status: WORKING END-TO-END; design changed to ATTRACTION

**DESIGN CHANGE (2026-10-27):** the base magnets now ATTRACT the pendulum
magnet (repulsion was abandoned — magnets too weak to show it). Equilibria are
the magnet positions themselves (valleys.cpp computeEquilibria); a run ends
ON a magnet; a rest point > `attract_reach_px` (100) from every magnet is
invalid ("attraction not engaged or no drop"). CSV column renamed
valley_id → magnet_id. The old clips (repulsion-era) were re-run: only
19.42.53 shows a genuine capture (M0, 68 px); the rest never got captured —
consistent with the weak-magnet problem that motivated the change.

`build/pendulum-track --videos Videos --autocal` runs the full pipeline:
per-video auto-calibration → warp → bob tracking → settle detection → valley
classification → `data/results.csv` + trajectory CSVs + PNG dumps.
Latest batch log: `data/batch_log.txt`; results in `data/results.csv`;
calibration previews + tracked-frame dumps in `data/dumps/` (visually verified).

## Architecture (all in src/, C++17 + OpenCV 5 from Homebrew)

- `autocal.cpp` — per-video calibration. Magnet candidates = low-saturation
  blobs (V 110–185, S≤80, area in provisional board space), sampled over 21
  frames clustered by position (support ≥ 0.2). Resting-bob exclusion: bright
  core p90 > 195. Triple selection: most-equilateral, hub-centered, supported.
  Labeling: hub barycentric fingerprint vs `layout_ref_bary` in config
  (measured on video 1 = 19.39.48). Homography: affine magnets → canonical
  equilateral triangle (400,190),(175,580),(625,580); valleys = edge midpoints.
  Warp anchor = median per-frame largest white blob (hub tape), eccentricity
  < 1.8, fully in-frame.
- `tracker.cpp` — board-space HSV mask (V≥130, S≤55) + ROI (canonical triangle
  ×1.45) + open 11×11 (kills the string, which is thick after the 3.5× warp!)
  + area band [150,3500] + static-blob rejection (autocal catalogues tape,
  magnets, wood highlights) + nearest-to-previous gating (gate 120 px).
- `state_machine.cpp` — SWINGING→SETTLED after 2.5 s quiet (<15 px/s);
  bounce (>40 px/s) reopens; quiet tail ≥1.5 s at video end = settled.
- `valleys.cpp` — computeEquilibria returns the magnets (attraction); to flip
  back to repulsion, return the angular-sorted edge midpoints instead.
- `main.cpp` — CLI, batch mode, dumps, --probe (HSV stats tuner),
  validity rules: never-detected, <80 % detection, rest farther than
  attract_reach_px (100) from every magnet → invalid + diagnostic note.
- `config.{h,cpp}` + `json.h` — dependency-free JSON config.
- `calibrate.cpp` — manual click calibration (works, not needed normally).

## Key hard-won facts (do not re-derive)

- Handheld phone → every video needs its own calibration; framing differs a
  lot between recordings.
- The warp magnifies ~3.5×: image-space intuitions (blob sizes, string width)
  do NOT transfer to board space. String is 8–12 px wide on the board → open
  kernel must be ≥11 or the tracker follows the string's midpoint.
- The bob is a white ball (core V≈209, area ~130 image px²); magnets are dimmer
  (V≤171) and smaller (20–45 px²). With attraction the bob ENDS on a magnet —
  the merged bob+magnet blob is the settle point; magnets are excluded from
  the static-reject list so they stay reachable.
- Videos fall into TWO rig configurations (magnets moved between 19.42 and
  19.53 sessions): trios are consistent within each group. `layout_ref_bary`
  belongs to group 1.
- The old clips show the bob hanging still mid-board → attraction/repulsion
  never engaged at that string height (the reason for the design change).

## Canonical layout is now the REAL measured one (2026-10-27)

`config/setup.json canonical_magnets = [[77,433],[439,281],[413,688]]` — the
reference video's detected trio in its provisional frame (hub at 400,450).
The real magnets are slightly UNEVEN (sides 393/414/428 px) and the hub is
offset; the old equilateral assumption put the markers near-but-not-on the
silver circles (user-visible bug). The cal preview is now warped by the FINAL
homography so markers land exactly on the circles. The web sim derives its
board triangle from this layout (vertex i = m_j + m_k − m_i, fit-scaled);
fixes also included: wall normals were inverted (bob glued to edges), basin
grid sims had undefined vx/vy (all cells "no capture"), and the old hard
force cap dominated the dynamics (now softened k/(d²+4000)).

## Web UI (added 2026-10-27)

`server.py` (stdlib-only Python) + `web/index.html`: localhost UI on
http://localhost:8000. Endpoints: /api/videos, POST /api/analyze (wraps the
tracker binary), /api/results, /api/images, /img. Frontend tabs: JS physics
simulator (draggable bob, sliders, theoretical basin map computed in-browser),
real-runs basin map + results table + calibration gallery, and one-click video
analysis. Phase-2 basin map is effectively covered by the web UI (real points
on the Runs tab; theoretical map in the simulator).

## Session 2026-10-27 (later): fixes + prediction/photo features

- **User-reported bugs fixed:** runs-tab refresh now visibly updates (stamp +
  counts) with row↔dot click linking, invalid filter, and trajectory drawing on
  selection; damping default reduced to 0.25 (force 12); Videos tab renders
  result cards (badge, stats, mini trajectory canvas) with the raw log
  collapsed; server TRACKER constant restored (analyze was 500ing) and
  Cache-Control: no-store everywhere.
- **New C++ tool `pendulum-photo`:** single-image magnet+hub detection with
  triple scoring against the rig's hub-distance FINGERPRINT (sorted normalized
  hub-dists — floor junk can't match it). OpenCV 5 gotchas: 2-arg
  invertAffineTransform, 2x3 affine for it, 3x3 composition order.
- **Prediction engine (web):** shared physicsStep for sim/basin/predictor;
  grid auto-tune vs valid runs; invalid runs simulated from their measured
  start points; "Save predictions to CSV" appends [SIMULATED] rows.
- **Photo tab (web):** upload top-down photo → /api/photo → markers on the
  photo → click drop point → predicted path + expected stop drawn on the photo.
  LIMITATION: all 3 magnets must be visible (a bob resting on one occludes it —
  verified fingerprint scoring still fails when 2 true magnets + 1 lucky junk
  blob is all there is).
- results.csv currently contains test duplicates + [SIMULATED] rows from
  verification; delete it and re-run the batch for a clean slate.

## Runs-tab rewrite for judges (2026-10-27, latest)

The "Real runs & basin map" tab was rewritten as a guided story for a
non-technical audience (per user: science-fair judges): intro explaining the
drop experiment in plain words, color legend, headline stat cards (real drops
/ measured / wins per magnet / simulated fills — [SIMULATED] rows are EXCLUDED
from the measured numbers), 4-step "how the software measures a drop" panel,
plain-language failure notes (plainNote() maps technical notes), predicted
zones background (canonical-space grid sim with tuned params, cached, toggle),
click dot/row → trajectory + caption. Technical table + cal gallery remain
behind a collapsed "Technical details" section. The tab-switching claim/
release cycle of the in-app browser required re-claiming the user tab between
test calls (iab tabs revert to user tabs at turn end).

## Next steps

1. Re-record with ATTRACTION: string short enough that the bob reaches the
   magnets, camera fixed per session, many drops from varied start points.
   Verify each session's calibration via the cal_*.png dumps (or the web UI).
2. Optional: motion-prediction tracking for fast blurred swings (the <80 %-
   detection invalids), per-group layout references.

## Run commands

```
cmake --build build
./build/pendulum-track --videos Videos --autocal --dump-dir data/dumps
AUTOCAL_DEBUG=1 ./build/pendulum-track --video f.mp4 --autocal   # debug cal
./build/pendulum-track --video f.mp4 --start-frame 450 --probe 268,388,10  # HSV tuner
```
