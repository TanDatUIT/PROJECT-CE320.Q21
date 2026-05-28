from __future__ import annotations

import argparse
import base64
import csv
import json
import threading
import time
import traceback
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen

from pc_fuzzy_strong import PCFuzzyStrong

ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT.parent / "data"
CAPTURE_DIR = DATA_DIR / "captures"
SNAP_DIR = DATA_DIR / "debug_snaps"
CSV_PATH = DATA_DIR / "pc_fuzzy_log.csv"


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest: dict[str, object] | None = None
        self.lines: deque[str] = deque(maxlen=120)
        self.frame_count = 0
        self.camera_url = ""
        self.running = False
        self.error = ""
        self.camera_config: dict[str, object] = {
            "brightness": 0,
            "contrast": 1,
            "saturation": 0,
            "quality": 14,
        }

    def add_line(self, line: str) -> None:
        with self.lock:
            self.lines.append(line)

    def set_source(self, camera_url: str) -> None:
        with self.lock:
            self.camera_url = camera_url

    def set_status(self, running: bool, error: str = "") -> None:
        with self.lock:
            self.running = running
            self.error = error

    def set_camera_config(self, config: dict[str, object]) -> None:
        with self.lock:
            self.camera_config.update(config)

    def set_latest(self, latest: dict[str, object]) -> None:
        with self.lock:
            self.frame_count += 1
            latest["index"] = self.frame_count
            self.latest = latest

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "latest": self.latest,
                "lines": list(self.lines),
                "frame_count": self.frame_count,
                "camera_url": self.camera_url,
                "running": self.running,
                "error": self.error,
                "camera_config": dict(self.camera_config),
            }


STATE = State()


def ensure_dirs() -> None:
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    if not CSV_PATH.exists():
        with CSV_PATH.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "pc_time",
                    "filename",
                    "gesture",
                    "confidence",
                    "fingers",
                    "mode",
                    "area_ratio",
                    "bbox",
                    "frame_ms",
                    "reason",
                ]
            )


def append_csv(row: dict[str, object]) -> None:
    result = row.get("result") or {}
    with CSV_PATH.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                row.get("pc_time"),
                row.get("filename"),
                result.get("gesture"),
                result.get("confidence"),
                result.get("fingers"),
                result.get("mode"),
                result.get("area_ratio"),
                json.dumps(result.get("bbox"), ensure_ascii=True),
                result.get("frame_ms"),
                result.get("reason"),
            ]
        )


def fetch_jpeg(camera_url: str, timeout_s: float) -> bytes:
    parsed = urlparse(camera_url)
    if "<" in camera_url or ">" in camera_url:
        raise ValueError("camera_url_placeholder: thay <ESP_IP> bang IP that, vi du http://192.168.1.25/capture")
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("camera_url_invalid: dung dang http://<ESP_IP>/capture")
    if parsed.path.rstrip("/") != "/capture":
        raise ValueError("camera_url_path_invalid: dung endpoint /capture, khong phai /stream")

    req = Request(camera_url, headers={"User-Agent": "pc-fuzzy-server/1.0", "Cache-Control": "no-cache"})
    with urlopen(req, timeout=timeout_s) as resp:
        content_type = resp.headers.get("Content-Type", "")
        data = resp.read()
    if "image/jpeg" not in content_type and not data.startswith(b"\xff\xd8"):
        raise ValueError(f"not_jpeg_content_type:{content_type}")
    return data


def camera_control_url(camera_url: str, config: dict[str, int]) -> str:
    parsed = urlparse(camera_url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("camera_url_invalid: dung dang http://<ESP_IP>/capture")
    return parsed._replace(path="/control", query=urlencode(config), fragment="").geturl()


def clamp_camera_config(payload: dict[str, object], current: dict[str, object]) -> dict[str, int]:
    def clamp_int(name: str, default: int, lo: int, hi: int) -> int:
        try:
            value = int(payload.get(name, current.get(name, default)))
        except (TypeError, ValueError):
            value = default
        return max(lo, min(hi, value))

    return {
        "brightness": clamp_int("brightness", 0, -2, 2),
        "contrast": clamp_int("contrast", 1, -2, 2),
        "saturation": clamp_int("saturation", 0, -2, 2),
        "quality": clamp_int("quality", 14, 10, 30),
    }


def set_camera_config(camera_url: str, config: dict[str, int], timeout_s: float = 3.0) -> dict[str, object]:
    url = camera_control_url(camera_url, config)
    req = Request(url, headers={"User-Agent": "pc-fuzzy-server/1.0", "Cache-Control": "no-cache"})
    with urlopen(req, timeout=timeout_s) as resp:
        data = resp.read()
    try:
        payload = json.loads(data.decode("utf-8") or "{}")
    except json.JSONDecodeError:
        payload = {"ok": False, "raw": data.decode("utf-8", errors="replace")}
    payload.update(config)
    return payload


def save_debug_snapshot(snapshot: dict[str, object]) -> dict[str, object]:
    latest = snapshot.get("latest")
    if not isinstance(latest, dict):
        raise ValueError("no latest frame to snap")
    image_b64 = latest.get("image_b64")
    if not isinstance(image_b64, str) or not image_b64:
        raise ValueError("latest frame has no image")

    ensure_dirs()
    result = latest.get("result") if isinstance(latest.get("result"), dict) else {}
    gesture = str(result.get("gesture") or "none")
    fingers = result.get("fingers", "x")
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    index = snapshot.get("frame_count", 0)
    stem = f"snap_{timestamp}_{int(index):06d}_{gesture}_f{fingers}"
    jpg_path = SNAP_DIR / f"{stem}.jpg"
    json_path = SNAP_DIR / f"{stem}.json"

    jpg_path.write_bytes(base64.b64decode(image_b64))
    debug_data = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "jpg": str(jpg_path),
        "frame_count": snapshot.get("frame_count"),
        "camera_url": snapshot.get("camera_url"),
        "camera_config": snapshot.get("camera_config"),
        "pc_time": latest.get("pc_time"),
        "filename": latest.get("filename"),
        "result": result,
    }
    json_path.write_text(json.dumps(debug_data, ensure_ascii=False, indent=2), encoding="utf-8")
    return {
        "jpg": str(jpg_path),
        "json": str(json_path),
        "gesture": gesture,
        "fingers": fingers,
        "reason": result.get("reason"),
    }


def worker(camera_url: str, interval_s: float, timeout_s: float, save_every: int) -> None:
    ensure_dirs()
    STATE.set_source(camera_url)
    analyzer = PCFuzzyStrong(prefer_mediapipe=True)
    STATE.add_line(f"[PC] Camera URL: {camera_url}")
    STATE.add_line(
        "[PC] Analyzer: "
        + ("mediapipe" if analyzer.hands is not None else "opencv_fallback")
        + (f" ({analyzer.backend_note})" if analyzer.backend_note else "")
    )
    if "<" in camera_url or ">" in camera_url:
        STATE.add_line("[PC] Replace <ESP_IP> bang IP that ESP32 in ra tren Serial Monitor")
    STATE.add_line("[PC] PC FUZZY worker started")

    while True:
        try:
            image_bytes = fetch_jpeg(camera_url, timeout_s)
            result, annotated = analyzer.analyze_jpeg(image_bytes)
            result_dict = result.to_dict()
            STATE.set_status(True)

            ts = time.strftime("%Y%m%d_%H%M%S")
            should_save = save_every > 0 and (STATE.snapshot()["frame_count"] % save_every == 0)
            filename = ""
            if should_save:
                filename = f"pcfuzzy_{ts}_{result.gesture}_{int(result.confidence * 100):02d}.jpg"
                (CAPTURE_DIR / filename).write_bytes(annotated or image_bytes)

            latest = {
                "pc_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "filename": filename,
                "image_b64": base64.b64encode(annotated or image_bytes).decode("ascii"),
                "result": result_dict,
            }
            STATE.set_latest(latest)
            append_csv(latest)
            STATE.add_line(
                f"[PC] {result.gesture} conf={result.confidence:.2f} "
                f"mode={result.mode} ms={result.frame_ms:.1f} reason={result.reason}"
            )
        except (URLError, TimeoutError, ValueError, OSError) as exc:
            error = f"{type(exc).__name__}: {exc}"
            STATE.set_status(False, error)
            STATE.add_line(f"[PC] camera error: {error}")
            time.sleep(1.0)
        except Exception as exc:  # Keep server alive for debugging.
            error = f"{type(exc).__name__}: {exc}"
            STATE.set_status(False, error)
            STATE.add_line(f"[PC] worker error: {error}")
            STATE.add_line(traceback.format_exc().splitlines()[-1])
            time.sleep(1.0)

        time.sleep(interval_s)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_html()
        elif parsed.path == "/api/state":
            self.send_json(STATE.snapshot())
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/camera_config":
            self.handle_camera_config()
            return
        if parsed.path == "/api/snapshot":
            self.handle_snapshot()
            return
        self.send_error(404)

    def handle_camera_config(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
            if not isinstance(payload, dict):
                raise ValueError("payload must be object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.send_json({"ok": False, "error": f"invalid camera config: {exc}"}, status=400)
            return

        snapshot = STATE.snapshot()
        camera_url = str(snapshot.get("camera_url") or "")
        config = clamp_camera_config(payload, snapshot.get("camera_config") or {})
        try:
            result = set_camera_config(camera_url, config)
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            STATE.add_line(f"[PC] camera config set failed: {error}")
            self.send_json({"ok": False, "error": error}, status=502)
            return

        applied = {
            "brightness": result.get("brightness", config["brightness"]),
            "contrast": result.get("contrast", config["contrast"]),
            "saturation": result.get("saturation", config["saturation"]),
            "quality": result.get("quality", config["quality"]),
        }
        STATE.set_camera_config(applied)
        STATE.add_line(
            "[PC] camera config "
            f"brightness={applied['brightness']} contrast={applied['contrast']} "
            f"saturation={applied['saturation']} quality={applied['quality']}"
        )
        self.send_json({"ok": True, "camera_config": STATE.snapshot()["camera_config"], "esp": result})

    def handle_snapshot(self) -> None:
        snapshot = STATE.snapshot()
        try:
            saved = save_debug_snapshot(snapshot)
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            STATE.add_line(f"[PC] snap failed: {error}")
            self.send_json({"ok": False, "error": error}, status=409)
            return

        STATE.add_line(
            f"[PC] snap saved gesture={saved.get('gesture')} fingers={saved.get('fingers')} "
            f"jpg={saved.get('jpg')}"
        )
        self.send_json({"ok": True, "saved": saved})

    def send_json(self, payload: dict[str, object], status: int = 200) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_html(self) -> None:
        body = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PC FUZZY Strong</title>
<style>
body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#111;padding:20px}
.wrap{max-width:1120px;margin:0 auto}
.grid{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(320px,.85fr);gap:16px;align-items:start}
.panel{background:#fff;border:1px solid #d7dde4;border-radius:8px;padding:14px;min-width:0}
.frameBox{width:100%;aspect-ratio:4/3;border-radius:6px;border:1px solid #d7dde4;background:#111;overflow:hidden}
img{display:block;width:100%;height:100%;object-fit:contain;background:#111}
.big{font-size:40px;font-weight:700;line-height:1;margin:8px 0}
.muted{color:#586270}
.controls{display:grid;grid-template-columns:116px minmax(0,1fr) 36px;gap:8px;align-items:center;margin:12px 0 4px}
.controls input{width:100%}
pre{height:260px;overflow:auto;background:#0d1117;color:#d1d5db;padding:12px;border-radius:6px;font-size:12px}
table{width:100%;border-collapse:collapse;table-layout:fixed}td{border-bottom:1px solid #edf0f2;padding:7px 4px;vertical-align:top;overflow-wrap:anywhere;word-break:break-word}td:first-child{width:116px;color:#2d333a}
#status{overflow-wrap:anywhere}
#reason{font-size:14px;line-height:1.35}
@media(max-width:850px){.grid{grid-template-columns:1fr}.big{font-size:32px}.frameBox{aspect-ratio:4/3}}
</style>
</head>
<body>
<div class="wrap">
<h2>PC FUZZY Strong</h2>
<div class="grid">
  <div class="panel">
    <div class="frameBox"><img id="frame" alt="latest frame"></div>
  </div>
  <div class="panel">
    <div class="muted" id="status">waiting</div>
    <div class="big" id="gesture">none</div>
    <div class="controls">
      <label for="brightness">brightness</label>
      <input id="brightness" type="range" min="-2" max="2" step="1" value="0">
      <span id="brightnessValue">0</span>
    </div>
    <table>
      <tr><td>confidence</td><td id="confidence">-</td></tr>
      <tr><td>fingers</td><td id="fingers">-</td></tr>
      <tr><td>mode</td><td id="mode">-</td></tr>
      <tr><td>area</td><td id="area">-</td></tr>
      <tr><td>frame ms</td><td id="frame_ms">-</td></tr>
      <tr><td>reason</td><td id="reason">-</td></tr>
    </table>
  </div>
</div>
<div class="panel" style="margin-top:16px">
<pre id="log"></pre>
</div>
</div>
<script>
async function refresh(){
  const s = await fetch('/api/state').then(r => r.json());
  const l = s.latest;
  document.getElementById('status').textContent =
    (s.running ? 'running' : 'error') + ' | ' + s.camera_url + (s.error ? ' | ' + s.error : '');
  if(l){
    const r = l.result || {};
    document.getElementById('frame').src = 'data:image/jpeg;base64,' + l.image_b64;
    document.getElementById('gesture').textContent = r.gesture || 'none';
    document.getElementById('confidence').textContent = r.confidence ?? '-';
    document.getElementById('fingers').textContent = r.fingers ?? '-';
    document.getElementById('mode').textContent = r.mode || '-';
    document.getElementById('area').textContent = r.area_ratio ?? '-';
    document.getElementById('frame_ms').textContent = r.frame_ms ?? '-';
    document.getElementById('reason').textContent = r.reason || '-';
  }
  const cfg = s.camera_config || {};
  if(document.activeElement !== document.getElementById('brightness')){
    document.getElementById('brightness').value = cfg.brightness ?? 0;
    document.getElementById('brightnessValue').textContent = cfg.brightness ?? 0;
  }
  document.getElementById('log').textContent = (s.lines || []).slice().reverse().join('\\n');
}
let brightnessTimer = null;
document.getElementById('brightness').addEventListener('input', (ev) => {
  const value = Number(ev.target.value);
  document.getElementById('brightnessValue').textContent = value;
  clearTimeout(brightnessTimer);
  brightnessTimer = setTimeout(async () => {
    const res = await fetch('/api/camera_config', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({brightness: value})
    }).then(r => r.json()).catch(err => ({ok:false,error:String(err)}));
    if(!res.ok){
      document.getElementById('status').textContent = 'brightness error | ' + (res.error || 'unknown');
    }
  }, 180);
});
setInterval(refresh, 700);
refresh();
</script>
</body>
</html>"""
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> None:
    parser = argparse.ArgumentParser(description="PC FUZZY strong server for ESP32-CAM snapshots")
    parser.add_argument("--camera-url", required=True, help="ESP32 URL, usually http://<ESP_IP>/capture")
    parser.add_argument("--web-port", type=int, default=5000)
    parser.add_argument("--interval", type=float, default=0.35, help="Seconds between PC polls")
    parser.add_argument("--timeout", type=float, default=2.5, help="Camera fetch timeout in seconds")
    parser.add_argument("--save-every", type=int, default=10, help="Save annotated image every N frames, 0 disables")
    args = parser.parse_args()

    thread = threading.Thread(
        target=worker,
        args=(args.camera_url, args.interval, args.timeout, args.save_every),
        daemon=True,
    )
    thread.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.web_port), Handler)
    print(f"[PC] Web: http://127.0.0.1:{args.web_port}", flush=True)
    print(f"[PC] Camera: {args.camera_url}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
