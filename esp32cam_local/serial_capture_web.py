#!/usr/bin/env python3
"""
Read ESP32-CAM local fuzzy logs over Serial, save capture JPGs on the PC,
and serve a small local web dashboard.

Usage on Windows:
  python serial_capture_web.py --port COM5 --baudrate 115200 --web-port 5000
"""

from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import re
import threading
import time
import traceback
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Missing dependency: pip install pyserial") from exc


ROOT = Path(__file__).resolve().parent
CAPTURE_DIR = ROOT / "pc_captures"
CSV_PATH = ROOT / "pc_capture_log.csv"

BEGIN_RE = re.compile(r"CAPTURE_JPG_BASE64_BEGIN,time_ms=(\d+),len=(\d+)")
CAPTURE_RE = re.compile(
    r"CAPTURE_LOG,time_ms=(?P<time_ms>\d+),path=(?P<path>[^,]+),"
    r"edge_density=(?P<edge_density>[-0-9.]+),"
    r"mean_strength=(?P<mean_strength>[-0-9.]+),"
    r"hand=(?P<hand>[01]),fingers=(?P<fingers>\d+),"
    r"(?:gesture=(?P<gesture>[^,]+),)?"
    r"bbox=(?P<min_x>-?\d+),(?P<min_y>-?\d+),(?P<max_x>-?\d+),(?P<max_y>-?\d+),"
    r"frame_time_ms=(?P<frame_time_ms>\d+),frame=(?P<frame>\d+)"
)
CONFIG_RE = re.compile(r"CONFIG_STATUS,(?P<body>.+)")

DEFAULT_CONFIG = {
    "quality": 18,
    "brightness": 0,
    "contrast": 2,
    "edge_dark": 0.26,
    "edge_bright": 0.20,
    "hand_threshold": 0.012,
    "capture_ms": 0,
}


class SerialBridge:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.ser: serial.Serial | None = None

    def attach(self, ser: serial.Serial | None) -> None:
        with self.lock:
            self.ser = ser

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\n").encode("utf-8")
        with self.lock:
            if self.ser is None or not self.ser.is_open:
                return False
            self.ser.write(data)
            self.ser.flush()
            return True


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.lines: deque[str] = deque(maxlen=120)
        self.latest_capture: dict[str, object] | None = None
        self.capture_count = 0
        self.port = ""
        self.baudrate = 0
        self.serial_open = False
        self.serial_error = ""
        self.last_serial_at = 0.0
        self.base64_mode = "idle"
        self.base64_chars = 0
        self.expected_jpg_len: int | None = None
        self.config = dict(DEFAULT_CONFIG)

    def add_line(self, line: str) -> None:
        with self.lock:
            self.lines.append(line)

    def configure_serial(self, port: str, baudrate: int) -> None:
        with self.lock:
            self.port = port
            self.baudrate = baudrate

    def set_serial_status(self, is_open: bool, error: str = "") -> None:
        with self.lock:
            self.serial_open = is_open
            self.serial_error = error

    def note_serial_line(self) -> None:
        with self.lock:
            self.last_serial_at = time.time()

    def set_base64_status(self, mode: str, chars: int = 0, expected_len: int | None = None) -> None:
        with self.lock:
            self.base64_mode = mode
            self.base64_chars = chars
            self.expected_jpg_len = expected_len

    def set_capture(self, capture: dict[str, object]) -> None:
        with self.lock:
            self.capture_count += 1
            capture["index"] = self.capture_count
            self.latest_capture = capture

    def set_config(self, config: dict[str, int | float]) -> None:
        with self.lock:
            self.config = dict(config)

    def set_existing_capture(self, capture: dict[str, object], capture_count: int) -> None:
        with self.lock:
            self.capture_count = capture_count
            capture["index"] = capture_count
            self.latest_capture = capture

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "latest_capture": self.latest_capture,
                "lines": list(self.lines),
                "capture_count": self.capture_count,
                "port": self.port,
                "baudrate": self.baudrate,
                "serial_open": self.serial_open,
                "serial_error": self.serial_error,
                "last_serial_age_s": round(time.time() - self.last_serial_at, 1) if self.last_serial_at else None,
                "base64_mode": self.base64_mode,
                "base64_chars": self.base64_chars,
                "expected_jpg_len": self.expected_jpg_len,
                "config": dict(self.config),
            }


STATE = State()
SERIAL_BRIDGE = SerialBridge()


def ensure_csv() -> None:
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    if CSV_PATH.exists():
        return
    with CSV_PATH.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "pc_time",
                "esp_time_ms",
                "filename",
                "edge_density",
                "mean_strength",
                "hand",
                "fingers",
                "gesture",
                "min_x",
                "min_y",
                "max_x",
                "max_y",
                "frame_time_ms",
                "frame",
            ]
        )


def append_csv(capture: dict[str, object]) -> None:
    with CSV_PATH.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        metrics = capture.get("metrics") or {}
        writer.writerow(
            [
                capture.get("pc_time"),
                capture.get("time_ms"),
                capture.get("filename"),
                metrics.get("edge_density"),
                metrics.get("mean_strength"),
                metrics.get("hand"),
                metrics.get("fingers"),
                metrics.get("gesture"),
                metrics.get("min_x"),
                metrics.get("min_y"),
                metrics.get("max_x"),
                metrics.get("max_y"),
                metrics.get("frame_time_ms"),
                metrics.get("frame"),
            ]
        )


def load_existing_latest_capture() -> None:
    if not CSV_PATH.exists():
        return

    rows: list[dict[str, str]] = []
    with CSV_PATH.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        return

    latest = rows[-1]
    filename = latest.get("filename") or ""
    if not filename or not (CAPTURE_DIR / filename).exists():
        return

    def as_int(name: str, default: int = 0) -> int:
        value = latest.get(name)
        try:
            return int(value) if value not in (None, "") else default
        except ValueError:
            return default

    def as_float(name: str, default: float = 0.0) -> float:
        value = latest.get(name)
        try:
            return float(value) if value not in (None, "") else default
        except ValueError:
            return default

    capture = {
        "pc_time": latest.get("pc_time") or "",
        "time_ms": as_int("esp_time_ms", int(time.time() * 1000)),
        "filename": filename,
        "path": str(CAPTURE_DIR / filename),
        "metrics": {
            "edge_density": as_float("edge_density"),
            "mean_strength": as_float("mean_strength"),
            "hand": as_int("hand"),
            "fingers": as_int("fingers"),
            "gesture": latest.get("gesture") or gesture_from_code(as_int("fingers")),
            "min_x": as_int("min_x", -1),
            "min_y": as_int("min_y", -1),
            "max_x": as_int("max_x", -1),
            "max_y": as_int("max_y", -1),
            "frame_time_ms": as_int("frame_time_ms"),
            "frame": as_int("frame"),
        },
    }
    STATE.set_existing_capture(capture, len(rows))
    STATE.add_line(f"[PC] Loaded latest saved capture: {filename}")


def parse_capture_log(line: str) -> dict[str, object] | None:
    match = CAPTURE_RE.match(line)
    if not match:
        return None
    data = match.groupdict()
    return {
        "time_ms": int(data["time_ms"]),
        "sd_path": data["path"],
        "edge_density": float(data["edge_density"]),
        "mean_strength": float(data["mean_strength"]),
        "hand": int(data["hand"]),
        "fingers": int(data["fingers"]),
        "gesture": data.get("gesture") or gesture_from_code(int(data["fingers"])),
        "min_x": int(data["min_x"]),
        "min_y": int(data["min_y"]),
        "max_x": int(data["max_x"]),
        "max_y": int(data["max_y"]),
        "frame_time_ms": int(data["frame_time_ms"]),
        "frame": int(data["frame"]),
    }


def parse_config_status(line: str) -> dict[str, int | float] | None:
    match = CONFIG_RE.match(line)
    if not match:
        return None
    config: dict[str, int | float] = {}
    for part in match.group("body").split(","):
        key, sep, value = part.partition("=")
        if not sep:
            continue
        key = key.strip()
        value = value.strip()
        try:
            if key in {"quality", "brightness", "contrast", "capture_ms"}:
                config[key] = int(float(value))
            elif key in {"edge_dark", "edge_bright", "hand_threshold"}:
                config[key] = float(value)
        except ValueError:
            continue
    return config or None


def gesture_from_code(code: int) -> str:
    if code == 5:
        return "open"
    if code == 1:
        return "fist"
    return "none"


def clamp_float(value: object, default: float, lo: float, hi: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    return max(lo, min(hi, number))


def clamp_int(value: object, default: int, lo: int, hi: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = default
    return max(lo, min(hi, number))


def serial_worker(port: str, baudrate: int) -> None:
    ensure_csv()
    load_existing_latest_capture()
    STATE.configure_serial(port, baudrate)
    pending_time_ms: int | None = None
    pending_len: int | None = None
    pending_chunks: list[str] = []
    pending_metrics: dict[str, object] | None = None
    pending_kind = "idle"

    while True:
        try:
            STATE.add_line(f"[PC] Opening serial {port} @ {baudrate}...")
            print(f"[PC] Opening serial {port} @ {baudrate}...", flush=True)
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baudrate
            ser.timeout = 1
            # Một số mạch USB-Serial/ESP32-CAM bị DTR/RTS giữ reset hoặc bootloader.
            ser.dtr = False
            ser.rts = False
            ser.open()
            with ser:
                ser.dtr = False
                ser.rts = False
                SERIAL_BRIDGE.attach(ser)
                STATE.set_serial_status(True)
                STATE.add_line(f"[PC] Serial opened: {port}")
                STATE.add_line("[PC] Press ESP32-CAM RST/EN once if no Serial lines appear")
                SERIAL_BRIDGE.send_line("GET_CONFIG")
                print(f"[PC] Serial opened: {port}", flush=True)
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    STATE.note_serial_line()

                    if pending_kind != "idle" and line != "CAPTURE_JPG_BASE64_END":
                        pending_chunks.append(line)
                        STATE.set_base64_status(pending_kind, sum(len(chunk) for chunk in pending_chunks), pending_len)
                        continue

                    print(line, flush=True)
                    STATE.add_line(line)

                    capture_log = parse_capture_log(line)
                    if capture_log:
                        pending_metrics = capture_log
                        continue

                    config = parse_config_status(line)
                    if config:
                        STATE.set_config(config)
                        continue

                    begin = BEGIN_RE.match(line)
                    if begin:
                        pending_time_ms = int(begin.group(1))
                        pending_len = int(begin.group(2))
                        pending_chunks = []
                        pending_kind = "capture"
                        STATE.set_base64_status(pending_kind, 0, pending_len)
                        continue

                    if line == "CAPTURE_JPG_BASE64_END" and pending_kind != "idle":
                        b64 = "".join(pending_chunks)
                        try:
                            image_bytes = base64.b64decode(b64, validate=True)
                        except Exception as exc:
                            STATE.add_line(f"[PC] Base64 decode failed: {exc}")
                            pending_kind = "idle"
                            STATE.set_base64_status("idle")
                            continue

                        if pending_len is not None and len(image_bytes) != pending_len:
                            STATE.add_line(f"[PC] Length mismatch: expected {pending_len}, got {len(image_bytes)}")

                        esp_time = pending_time_ms or int(time.time() * 1000)
                        filename = f"capture_{esp_time}.jpg"
                        path = CAPTURE_DIR / filename
                        path.write_bytes(image_bytes)

                        capture = {
                            "pc_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "time_ms": esp_time,
                            "filename": filename,
                            "path": str(path),
                            "metrics": pending_metrics,
                        }
                        STATE.set_capture(capture)
                        append_csv(capture)
                        STATE.add_line(f"[PC] Saved {path}")
                        print(f"[PC] Saved {path}", flush=True)

                        pending_time_ms = None
                        pending_len = None
                        pending_chunks = []
                        pending_metrics = None
                        pending_kind = "idle"
                        STATE.set_base64_status("idle")
                        continue

        except Exception as exc:  # Keep the web UI alive and show serial failures there.
            SERIAL_BRIDGE.attach(None)
            error = f"{type(exc).__name__}: {exc}"
            STATE.set_serial_status(False, error)
            STATE.add_line(f"[PC] Serial error: {error}")
            print(f"[PC] Serial error: {error}", flush=True)
            print(traceback.format_exc(), flush=True)
            time.sleep(2)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_html()
        elif parsed.path == "/api/state":
            self.send_json(STATE.snapshot())
        elif parsed.path.startswith("/captures/"):
            self.send_capture(parsed.path.removeprefix("/captures/"))
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            self.handle_config_post()
            return
        if parsed.path == "/api/snapshot":
            self.handle_snapshot_post()
            return
        self.send_error(404)

    def handle_snapshot_post(self) -> None:
        if not SERIAL_BRIDGE.send_line("SNAP"):
            self.send_json({"ok": False, "error": "serial not open"}, status=503)
            return
        STATE.add_line("[PC] Sent SNAP")
        self.send_json({"ok": True, "sent": "SNAP"})

    def handle_config_post(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self.send_json({"ok": False, "error": "invalid json"}, status=400)
            return

        quality = clamp_int(payload.get("quality"), int(DEFAULT_CONFIG["quality"]), 10, 40)
        brightness = clamp_int(payload.get("brightness"), int(DEFAULT_CONFIG["brightness"]), -2, 2)
        contrast = clamp_int(payload.get("contrast"), int(DEFAULT_CONFIG["contrast"]), -2, 2)
        edge_dark = clamp_float(payload.get("edge_dark"), float(DEFAULT_CONFIG["edge_dark"]), 0.05, 0.80)
        edge_bright = clamp_float(payload.get("edge_bright"), float(DEFAULT_CONFIG["edge_bright"]), 0.05, 0.80)
        hand_threshold = clamp_float(payload.get("hand_threshold"), float(DEFAULT_CONFIG["hand_threshold"]), 0.005, 0.200)
        capture_ms = clamp_int(payload.get("capture_ms"), int(DEFAULT_CONFIG["capture_ms"]), 0, 60000)

        command = (
            f"SET quality={quality} brightness={brightness} contrast={contrast} "
            f"edge_dark={edge_dark:.3f} edge_bright={edge_bright:.3f} "
            f"hand_threshold={hand_threshold:.3f} capture_ms={capture_ms}"
        )
        if not SERIAL_BRIDGE.send_line(command):
            self.send_json({"ok": False, "error": "serial not open"}, status=503)
            return

        STATE.add_line(f"[PC] Sent config: {command}")
        STATE.set_config(
            {
                "quality": quality,
                "brightness": brightness,
                "contrast": contrast,
                "edge_dark": round(edge_dark, 3),
                "edge_bright": round(edge_bright, 3),
                "hand_threshold": round(hand_threshold, 3),
                "capture_ms": capture_ms,
            }
        )
        self.send_json({"ok": True, "sent": command, "config": STATE.snapshot()["config"]})

    def send_html(self) -> None:
        body = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Local Fuzzy</title>
<style>
body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#111;padding:20px}
.wrap{max-width:1100px;margin:0 auto}
.panel{background:white;border-radius:8px;padding:16px;margin-bottom:16px;box-shadow:0 6px 18px rgba(0,0,0,.08)}
.grid{display:grid;grid-template-columns:2fr 1fr;gap:16px;align-items:start}
.statusbar{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-top:12px}
img{width:100%;background:#111;border-radius:6px;display:block}
.placeholder{min-height:260px;background:#101418;color:#d8e4ec;border-radius:6px;display:flex;align-items:center;justify-content:center;text-align:center;padding:16px}
.metrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
.metric{background:#f1f4f7;border-radius:6px;padding:10px}
.label{font-size:12px;color:#667;text-transform:uppercase;margin-bottom:4px}
.value{font-size:22px;font-weight:700}
.small{font-size:14px;font-weight:600;overflow-wrap:anywhere}
.controls{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center}
.controls input[type=range]{width:100%}
.controls button{padding:10px 14px;border:0;border-radius:6px;background:#17324d;color:#fff;font-weight:700;cursor:pointer}
pre{white-space:pre-wrap;max-height:360px;overflow:auto;background:#101418;color:#d8e4ec;border-radius:6px;padding:12px;font-size:12px}
@media(max-width:800px){.grid,.statusbar{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="wrap">
<div class="panel">
<h1>ESP32-CAM Local Fuzzy</h1>
<div class="statusbar">
<div class="metric"><div class="label">Serial</div><div class="value small" id="serial">-</div></div>
<div class="metric"><div class="label">Last Line</div><div class="value small" id="lastline">-</div></div>
<div class="metric"><div class="label">Base64</div><div class="value small" id="base64">-</div></div>
<div class="metric"><div class="label">Captures</div><div class="value small" id="captures">0</div></div>
</div>
</div>
<div class="panel">
<div class="grid">
<div>
<div id="capture-placeholder" class="placeholder">Waiting for capture...<br>Serial/API status is shown above.</div>
<img id="capture" alt="Latest capture" style="display:none">
<p id="status">Waiting for capture...</p>
</div>
<div class="metrics">
<div class="metric"><div class="label">Hand</div><div class="value" id="hand">-</div></div>
<div class="metric"><div class="label">Gesture</div><div class="value" id="fingers">-</div></div>
<div class="metric"><div class="label">Edge</div><div class="value" id="edge">-</div></div>
<div class="metric"><div class="label">Mean</div><div class="value" id="mean">-</div></div>
<div class="metric"><div class="label">BBox</div><div class="value" id="bbox">-</div></div>
<div class="metric"><div class="label">Frame</div><div class="value" id="frame">-</div></div>
<div class="metric"><div class="label">Mode</div><div class="value small" id="mode">-</div></div>
<div class="metric"><div class="label">Quality</div><div class="value" id="qualityValue">18</div></div>
<div class="metric"><div class="label">Brightness</div><div class="value" id="brightnessValue">0</div></div>
<div class="metric"><div class="label">Contrast</div><div class="value" id="contrastValue">2</div></div>
<div class="metric"><div class="label">Edge Dark</div><div class="value" id="edgeDarkValue">0.260</div></div>
<div class="metric"><div class="label">Edge Bright</div><div class="value" id="edgeBrightValue">0.200</div></div>
<div class="metric"><div class="label">Hand Threshold</div><div class="value" id="handThresholdValue">0.012</div></div>
</div>
</div>
</div>
<div class="panel">
<h2>Camera Controls</h2>
<div class="controls">
<div><div class="label">JPEG Quality (10-40, thấp hơn = nét hơn, nặng hơn)</div><input id="quality" type="range" min="10" max="40" step="1" value="18"></div>
<button onclick="applyConfig()">Apply</button>
</div>
<div class="controls">
<div><div class="label">Capture Interval ms (0=off)</div><input id="capture_ms" type="range" min="0" max="60000" step="1000" value="0"></div>
<button onclick="requestSnapshot()">Snapshot</button>
</div>
<div class="small" id="captureMsText">0 ms</div>
<div class="controls">
<div><div class="label">Brightness (-2 đến 2)</div><input id="brightness" type="range" min="-2" max="2" step="1" value="0"></div>
<div class="small" id="brightnessText">0</div>
</div>
<div class="controls">
<div><div class="label">Contrast (-2 đến 2)</div><input id="contrast" type="range" min="-2" max="2" step="1" value="2"></div>
<div class="small" id="contrastText">2</div>
</div>
<div class="controls">
<div><div class="label">Edge Dark (cao hơn = ít nhiễu hơn trong tối)</div><input id="edge_dark" type="range" min="0.05" max="0.80" step="0.01" value="0.26"></div>
<div class="small" id="edgeDarkText">0.26</div>
</div>
<div class="controls">
<div><div class="label">Edge Bright (cao hơn = bớt bắt nền sáng)</div><input id="edge_bright" type="range" min="0.05" max="0.80" step="0.01" value="0.20"></div>
<div class="small" id="edgeBrightText">0.20</div>
</div>
<div class="controls">
<div><div class="label">Hand Threshold (cao hơn = khó nhận tay hơn)</div><input id="hand_threshold" type="range" min="0.005" max="0.200" step="0.005" value="0.012"></div>
<div class="small" id="handThresholdText">0.012</div>
</div>
<p id="configStatus">Ready</p>
</div>
<div class="panel"><h2>Serial Log</h2><pre id="log"></pre></div>
</div>
<script>
var configDirty = false;

function syncSliderText(){
  document.getElementById('qualityValue').textContent = document.getElementById('quality').value;
  document.getElementById('brightnessValue').textContent = document.getElementById('brightness').value;
  document.getElementById('contrastValue').textContent = document.getElementById('contrast').value;
  document.getElementById('edgeDarkValue').textContent = Number(document.getElementById('edge_dark').value).toFixed(3);
  document.getElementById('edgeBrightValue').textContent = Number(document.getElementById('edge_bright').value).toFixed(3);
  document.getElementById('handThresholdValue').textContent = Number(document.getElementById('hand_threshold').value).toFixed(3);
  document.getElementById('brightnessText').textContent = document.getElementById('brightness').value;
  document.getElementById('contrastText').textContent = document.getElementById('contrast').value;
  document.getElementById('edgeDarkText').textContent = Number(document.getElementById('edge_dark').value).toFixed(3);
  document.getElementById('edgeBrightText').textContent = Number(document.getElementById('edge_bright').value).toFixed(3);
  document.getElementById('handThresholdText').textContent = Number(document.getElementById('hand_threshold').value).toFixed(3);
  document.getElementById('captureMsText').textContent = document.getElementById('capture_ms').value + ' ms';
}

function markConfigDirty(){
  configDirty = true;
  syncSliderText();
}

async function applyConfig(){
  var payload = {
    quality: Number(document.getElementById('quality').value),
    brightness: Number(document.getElementById('brightness').value),
    contrast: Number(document.getElementById('contrast').value),
    edge_dark: Number(document.getElementById('edge_dark').value),
    edge_bright: Number(document.getElementById('edge_bright').value),
    hand_threshold: Number(document.getElementById('hand_threshold').value),
    capture_ms: Number(document.getElementById('capture_ms').value)
  };
  document.getElementById('configStatus').textContent = 'Sending...';
  try {
    var r = await fetch('/api/config', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload)
    });
    var data = await r.json();
    if(!r.ok || !data.ok){
      throw new Error(data.error || ('HTTP ' + r.status));
    }
    configDirty = false;
    document.getElementById('configStatus').textContent = 'Applied: ' + data.sent;
  } catch (err) {
    document.getElementById('configStatus').textContent = 'Apply failed: ' + err;
  }
}

async function requestSnapshot(){
  document.getElementById('configStatus').textContent = 'Requesting snapshot...';
  try {
    var r = await fetch('/api/snapshot', {method: 'POST'});
    var data = await r.json();
    if(!r.ok || !data.ok){
      throw new Error(data.error || ('HTTP ' + r.status));
    }
    document.getElementById('configStatus').textContent = 'Snapshot requested; wait for CAPTURE_JPG_BASE64_END.';
  } catch (err) {
    document.getElementById('configStatus').textContent = 'Snapshot failed: ' + err;
  }
}

async function refresh(){
  try {
    var r = await fetch('/api/state?ts=' + Date.now(), {cache: 'no-store'});
    if (!r.ok) {
      throw new Error('HTTP ' + r.status);
    }
    var s = await r.json();
    var c = s.latest_capture;
    var cfg = s.config || {};
    var serialText = s.serial_open ? ('OPEN ' + s.port + ' @ ' + s.baudrate) : ('ERROR/CLOSED ' + (s.port || ''));
    document.getElementById('serial').textContent = s.serial_error ? (serialText + ': ' + s.serial_error) : serialText;
    document.getElementById('lastline').textContent = s.last_serial_age_s === null ? 'no data yet' : (s.last_serial_age_s + 's ago');
    document.getElementById('base64').textContent = s.base64_mode !== 'idle' ? (s.base64_mode + ' | ' + s.base64_chars + ' chars / jpg ' + (s.expected_jpg_len || '?') + ' bytes') : 'idle';
    document.getElementById('captures').textContent = s.capture_count || 0;
    document.getElementById('mode').textContent = s.base64_mode || 'idle';
    if(!configDirty){
      document.getElementById('quality').value = cfg.quality != null ? cfg.quality : 18;
      document.getElementById('brightness').value = cfg.brightness != null ? cfg.brightness : -1;
      document.getElementById('contrast').value = cfg.contrast != null ? cfg.contrast : 2;
      document.getElementById('edge_dark').value = cfg.edge_dark != null ? cfg.edge_dark : 0.26;
      document.getElementById('edge_bright').value = cfg.edge_bright != null ? cfg.edge_bright : 0.20;
      document.getElementById('hand_threshold').value = cfg.hand_threshold != null ? cfg.hand_threshold : 0.012;
      document.getElementById('capture_ms').value = cfg.capture_ms != null ? cfg.capture_ms : 0;
    }
    syncSliderText();
    if(c){
      document.getElementById('capture-placeholder').style.display = 'none';
      document.getElementById('capture').style.display = 'block';
      document.getElementById('capture').src = '/captures/' + c.filename + '?t=' + c.time_ms;
      document.getElementById('status').textContent = c.pc_time + ' | ' + c.filename;
      var m = c.metrics || {};
      document.getElementById('hand').textContent = m.hand === 1 ? 'YES' : 'NO';
      document.getElementById('fingers').textContent = m.gesture || (m.fingers === 5 ? 'open' : (m.fingers === 1 ? 'fist' : 'none'));
      document.getElementById('edge').textContent = Number(m.edge_density || 0).toFixed(4);
      document.getElementById('mean').textContent = Number(m.mean_strength || 0).toFixed(4);
      document.getElementById('bbox').textContent = [m.min_x,m.min_y,m.max_x,m.max_y].join(', ');
      document.getElementById('frame').textContent = m.frame != null ? m.frame : '-';
    } else {
      document.getElementById('capture-placeholder').style.display = 'flex';
      document.getElementById('capture').style.display = 'none';
      document.getElementById('status').textContent = 'Waiting for capture... Check Serial Log and status cards.';
    }
    document.getElementById('log').textContent = (s.lines || []).join('\\n');
  } catch (err) {
    document.getElementById('status').textContent = 'Dashboard API error: ' + err;
    document.getElementById('log').textContent = 'Cannot read /api/state. Restart Python script and hard-refresh browser.';
    }
}
document.getElementById('quality').addEventListener('input', markConfigDirty);
document.getElementById('brightness').addEventListener('input', markConfigDirty);
document.getElementById('contrast').addEventListener('input', markConfigDirty);
document.getElementById('edge_dark').addEventListener('input', markConfigDirty);
document.getElementById('edge_bright').addEventListener('input', markConfigDirty);
document.getElementById('hand_threshold').addEventListener('input', markConfigDirty);
document.getElementById('capture_ms').addEventListener('input', markConfigDirty);
syncSliderText();
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>"""
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj: object, status: int = 200) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_capture(self, filename: str) -> None:
        safe_name = Path(filename).name
        path = CAPTURE_DIR / safe_name
        if not path.exists():
            self.send_error(404)
            return
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

def main() -> None:
    parser = argparse.ArgumentParser(description="ESP32-CAM local fuzzy Serial capture web viewer")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--web-port", type=int, default=5000)
    args = parser.parse_args()

    worker = threading.Thread(target=serial_worker, args=(args.port, args.baudrate), daemon=True)
    worker.start()

    server = ThreadingHTTPServer(("127.0.0.1", args.web_port), Handler)
    print(f"[PC] Web: http://127.0.0.1:{args.web_port}")
    print(f"[PC] Saving captures to: {CAPTURE_DIR}")
    server.serve_forever()


if __name__ == "__main__":
    main()
