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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
VIDEOS = ROOT / "Videos"
DATA = ROOT / "data"
RESULTS = DATA / "results.csv"
DUMPS = DATA / "dumps"
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
        if urlparse(self.path).path != "/api/analyze":
            self._send(404, {"error": "not found"})
            return
        length = int(self.headers.get("Content-Length", 0))
        try:
            video = json.loads(self.rfile.read(length) or b"{}").get("video", "")
        except json.JSONDecodeError:
            video = ""
        code, log = run_tracker(video)
        self._send(200, {"ok": code == 0, "log": log,
                         "results": results_rows(),
                         "images": list_images()})

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
