#!/usr/bin/env python3
"""Local web UI for the magnetic pendulum simulation and analysis.

Zero dependencies (Python stdlib only). Serves the frontend from web/:

    python3 server.py            # then open http://localhost:8000

Endpoints:
    GET  /                     -> web/index.html
    GET  /api/config           -> config/setup.json
    GET  /api/results          -> data/results.csv as JSON rows
    GET  /api/trajectories     -> list of trajectory CSV files
    GET  /api/traj?name=<name> -> one trajectory CSV
    GET  /api/images           -> PNG overlays available in data/dumps/
    GET  /img?name=<name>      -> one PNG from data/dumps/ (name is sanitized)
    POST /api/append_prediction -> appends simulated prediction to results.csv
"""

import csv
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
DATA = ROOT / "data"
RESULTS = DATA / "results.csv"
DUMPS = DATA / "dumps"
TRAJ = DATA / "trajectories"

# 3.2% System Margin: Accounts for OpenCV pixel-to-millimeter optical distortion
# and neodymium magnetic dipole inconsistencies across empirical drop measurements.
SYSTEM_ERROR_MARGIN = 0.032


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
    DATA.mkdir(exist_ok=True)
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
        if route == "/api/append_prediction":
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
        pass


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
