#!/usr/bin/env python3
"""Local web UI for the magnetic pendulum tracker.

Zero dependencies (Python stdlib only). Serves the frontend from web/ and
wraps the compiled C++ tracker:

    python3 server.py            # then open http://localhost:8000

Endpoints:
    GET  /                     -> web/index.html
    GET  /api/videos           -> list of clip names in Videos/
    POST /api/analyze          -> body {"video": "<name>" | "*"}; runs the tracker
    GET  /api/results          -> data/results.csv as JSON rows
    GET  /api/images           -> PNG overlays available in data/dumps/
    GET  /img?name=<name>      -> one PNG from data/dumps/ (name is sanitized)
"""

import csv
import json
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
VIDEOS = ROOT / "Videos"
DATA = ROOT / "data"
RESULTS = DATA / "results.csv"
DUMPS = DATA / "dumps"
TRAJ = DATA / "trajectories"
UPLOADS = DATA / "uploads"
PHOTO_TOOL = ROOT / "build" / "pendulum-photo"
TRACKER = ROOT / "build" / "pendulum-track"
VIDEO_EXTS = {".mp4", ".mov", ".avi", ".mkv"}
ANALYZE_TIMEOUT_S = 900


def list_videos():
    if not VIDEOS.exists():
        return []
    return sorted(p.name for p in VIDEOS.iterdir() if p.suffix.lower() in VIDEO_EXTS)


def results_rows():
    if not RESULTS.exists():
        return []
    with RESULTS.open(newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("run_id", "start_x", "start_y", "end_x", "end_y", "magnet_id",
                  "dist_px", "duration_s", "detection_ratio", "valid"):
            try:
                r[k] = float(r[k]) if "." in r[k] else int(r[k])
            except (ValueError, KeyError):
                pass
    return rows


def list_images():
    if not DUMPS.exists():
        return []
    return sorted(p.name for p in DUMPS.iterdir() if p.suffix.lower() == ".png")


def list_trajectories():
    if not TRAJ.exists():
        return []
    return sorted(p.name for p in TRAJ.iterdir() if p.suffix.lower() == ".csv")


def next_run_id():
    if not RESULTS.exists():
        return 1
    top = 0
    with RESULTS.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                top = max(top, int(row["run_id"]))
            except (ValueError, KeyError):
                pass
    return top + 1


def append_prediction(payload):
    """Append a simulated-prediction row to results.csv. Real rows untouched."""
    UPLOADS.mkdir(exist_ok=True)
    vid = str(payload.get("video", "unknown")) + " [SIMULATED]"
    note = "simulated prediction (" + str(payload.get("params", "default params")) + ")"
    row = [next_run_id(), vid,
           payload.get("start_x", -1), payload.get("start_y", -1),
           payload.get("end_x", -1), payload.get("end_y", -1),
           payload.get("magnet_id", -1), payload.get("dist_px", -1),
           payload.get("duration_s", 0), 1.0, 1, note]
    new_file = not RESULTS.exists()
    with RESULTS.open("a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["run_id", "video", "start_x", "start_y", "end_x", "end_y",
                        "magnet_id", "dist_px", "duration_s", "detection_ratio",
                        "valid", "note"])
        w.writerow(row)
    return row[0]


def run_photo(image_bytes, ext):
    if not PHOTO_TOOL.exists():
        return False, {"error": "pendulum-photo not built — run: cmake --build build"}
    UPLOADS.mkdir(exist_ok=True)
    DUMPS.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    upload = UPLOADS / f"upload_{ts}{ext}"
    upload.write_bytes(image_bytes)
    out_json = UPLOADS / f"photo_{ts}.json"
    annotated = DUMPS / f"photo_{ts}.png"
    proc = subprocess.run(
        [str(PHOTO_TOOL), "--image", str(upload), "--out", str(out_json),
         "--annotated", str(annotated)],
        capture_output=True, text=True, timeout=120, cwd=str(ROOT))
    if not out_json.exists():
        return False, {"error": "photo tool failed", "log": (proc.stdout + proc.stderr).strip()}
    data = json.loads(out_json.read_text())
    if data.get("ok"):
        data["annotated"] = annotated.name
        data["upload"] = upload.name
    return data.get("ok", False), data


def run_tracker(video_name):
    if not TRACKER.exists():
        return 1, "tracker not built — run: cmake --build build"
    if video_name == "*":
        cmd = [str(TRACKER), "--videos", str(VIDEOS), "--autocal",
               "--dump-dir", str(DUMPS)]
        label = "ALL videos"
    else:
        if video_name not in list_videos():
            return 1, f"unknown video: {video_name}"
        cmd = [str(TRACKER), "--video", str(VIDEOS / video_name), "--autocal",
               "--dump-dir", str(DUMPS)]
        label = video_name
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=ANALYZE_TIMEOUT_S, cwd=str(ROOT))
        log = (proc.stdout + proc.stderr).strip()
        return proc.returncode, f"$ pendulum-track {label}\n{log}"
    except subprocess.TimeoutExpired:
        return 1, f"timed out after {ANALYZE_TIMEOUT_S}s"


class QuietServer(ThreadingHTTPServer):
    # A browser tab reloading/closing mid-response raises BrokenPipeError in
    # the handler thread; that is normal and must not spam the terminal.
    def handle_error(self, request, client_address):
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client went away mid-response

    def do_GET(self):
        route = urlparse(self.path)
        if route.path in ("/", "/index.html"):
            page = (WEB / "index.html").read_bytes()
            self._send(200, page, "text/html; charset=utf-8")
        elif route.path == "/api/config":
            cfg_path = ROOT / "config" / "setup.json"
            self._send(200, json.loads(cfg_path.read_text()))
        elif route.path == "/api/videos":
            self._send(200, {"videos": list_videos()})
        elif route.path == "/api/results":
            self._send(200, {"rows": results_rows()})
        elif route.path == "/api/trajectories":
            self._send(200, {"files": list_trajectories()})
        elif route.path == "/api/traj":
            name = Path(parse_qs(route.query).get("name", [""])[0]).name
            path = TRAJ / name
            if path.suffix.lower() == ".csv" and path.exists():
                self._send(200, path.read_bytes(), "text/csv; charset=utf-8")
            else:
                self._send(404, {"error": "no such trajectory"})
        elif route.path == "/api/images":
            self._send(200, {"images": list_images()})
        elif route.path == "/img":
            name = Path(parse_qs(route.query).get("name", [""])[0]).name
            path = DUMPS / name
            if path.suffix.lower() == ".png" and path.exists():
                self._send(200, path.read_bytes(), "image/png")
            else:
                self._send(404, {"error": "no such image"})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        route = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        if route == "/api/analyze":
            try:
                video = json.loads(body or b"{}").get("video", "")
            except json.JSONDecodeError:
                video = ""
            code, log = run_tracker(video)
            self._send(200, {"ok": code == 0, "log": log,
                             "results": results_rows(),
                             "images": list_images()})
        elif route == "/api/photo":
            ctype = self.headers.get("Content-Type", "image/jpeg")
            ext = {"image/jpeg": ".jpg", "image/jpg": ".jpg",
                   "image/png": ".png", "image/webp": ".webp"}.get(ctype, ".jpg")
            if not body:
                self._send(400, {"error": "empty upload"})
                return
            ok, data = run_photo(body, ext)
            self._send(200, {**data, "ok": ok})
        elif route == "/api/append_prediction":
            try:
                payload = json.loads(body or b"{}")
            except json.JSONDecodeError:
                self._send(400, {"error": "bad json"})
                return
            run_id = append_prediction(payload)
            self._send(200, {"ok": True, "run_id": run_id,
                             "results": results_rows()})
        else:
            self._send(404, {"error": "not found"})

    def log_message(self, fmt, *args):  # quieter console
        if "/api/analyze" not in (args[0] if args else ""):
            return


if __name__ == "__main__":
    try:
        server = QuietServer(("127.0.0.1", 8000), Handler)
    except OSError as e:
        if e.errno in (48, 98):  # address already in use
            print("port 8000 is busy — another server.py is probably running.\n"
                  "stop it with:  kill $(lsof -ti :8000)\nthen start this one again.")
            raise SystemExit(1)
        raise
    print("Magnetic Pendulum Lab -> http://localhost:8000  (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
