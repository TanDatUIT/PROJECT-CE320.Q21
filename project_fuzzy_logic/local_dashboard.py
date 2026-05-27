from __future__ import annotations

import argparse
import base64
import csv
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Deque

import serial
from flask import Flask, jsonify, Response, request, send_from_directory


CSV_HEADER = [
    "time_ms",
    "edge_density",
    "mean_strength",
    "hand_detected",
    "finger_count",
    "min_x",
    "min_y",
    "max_x",
    "max_y",
    "frame_time_ms",
]

ROOT = Path(__file__).resolve().parent
CAPTURE_DIR = ROOT / "pc_captures"
BEGIN_RE = re.compile(r"CAPTURE_JPG_BASE64_BEGIN,time_ms=(\d+),len=(\d+)")


@dataclass
class LogEntry:
    time_ms: int
    edge_density: float
    mean_strength: float
    hand_detected: int
    finger_count: int
    min_x: int
    min_y: int
    max_x: int
    max_y: int
    frame_time_ms: int


class SerialLogBridge:
    def __init__(self, port: str, baudrate: int, csv_path: Path, history_size: int = 300) -> None:
        self.port = port
        self.baudrate = baudrate
        self.csv_path = csv_path
        self.history: Deque[LogEntry] = deque(maxlen=history_size)
        self.latest_status = "dang_khoi_tao"
        self.last_raw_line = ""
        self.latest_capture: dict[str, object] | None = None
        self.capture_count = 0
        self.base64_mode = "idle"
        self.base64_chars = 0
        self.expected_jpg_len: int | None = None
        self.capture_status = "idle"
        self.last_snapshot_request_at = 0.0
        self.config: dict[str, int | float] = {
            "quality": 18,
            "brightness": -1,
            "contrast": 2,
            "edge_dark": 0.32,
            "edge_bright": 0.24,
            "hand_threshold": 0.035,
            "snapshot_ms": 0,
        }
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._ensure_csv_header()
        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)

    def _ensure_csv_header(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.csv_path.exists():
            with self.csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(CSV_HEADER)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict:
        with self._lock:
            latest = asdict(self.history[-1]) if self.history else None
            history = [asdict(item) for item in self.history]
            return {
                "status": self.latest_status,
                "latest": latest,
                "history": history,
                "last_raw_line": self.last_raw_line,
                "csv_path": str(self.csv_path),
                "latest_capture": self.latest_capture,
                "capture_count": self.capture_count,
                "base64_mode": self.base64_mode,
                "base64_chars": self.base64_chars,
                "expected_jpg_len": self.expected_jpg_len,
                "capture_status": self.capture_status,
                "snapshot_wait_s": (
                    round(time.time() - self.last_snapshot_request_at, 1)
                    if self.last_snapshot_request_at and self.capture_status == "requested_by_web"
                    else None
                ),
                "config": dict(self.config),
            }

    def send_line(self, line: str) -> bool:
        payload = (line.rstrip("\r\n") + "\n").encode("utf-8")
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            self._serial.write(payload)
            self._serial.flush()
            return True

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                    self._attach_serial(ser)
                    self._set_status(f"da_ket_noi_{self.port}")
                    self.send_line("GET_CONFIG")
                    pending_capture_time: int | None = None
                    pending_expected_len: int | None = None
                    pending_chunks: list[str] = []
                    while not self._stop_event.is_set():
                        raw = ser.readline().decode("utf-8", errors="ignore").strip()
                        if not raw:
                            continue
                        if pending_capture_time is not None:
                            if raw == "CAPTURE_JPG_BASE64_END":
                                self._set_last_raw_line(raw)
                                self._finish_capture(pending_capture_time, pending_chunks)
                                pending_capture_time = None
                                pending_expected_len = None
                                pending_chunks = []
                                self._set_base64_status("idle")
                            else:
                                pending_chunks.append(raw)
                                self._set_base64_status(
                                    "capture",
                                    sum(len(chunk) for chunk in pending_chunks),
                                    pending_expected_len,
                                )
                            continue

                        self._set_last_raw_line(raw)
                        begin_match = BEGIN_RE.match(raw)
                        if begin_match:
                            pending_capture_time = int(begin_match.group(1))
                            pending_expected_len = int(begin_match.group(2))
                            pending_chunks = []
                            self._set_base64_status("capture", 0, pending_expected_len)
                            continue

                        if raw.startswith("LOGCSV,"):
                            entry = self._parse_logcsv(raw)
                            if entry is not None:
                                self._append_entry(entry)
                        elif raw.startswith("CAPTURE_STATUS,"):
                            self._set_capture_status(raw.removeprefix("CAPTURE_STATUS,"))
                        elif raw.startswith("CONFIG_STATUS,"):
                            self._parse_config_status(raw)
                        elif raw.startswith("LOGCSV_HEADER,"):
                            self._set_status("dang_nhan_header")
                        else:
                            self._set_status("dang_nhan_serial")
            except serial.SerialException as exc:
                self._attach_serial(None)
                self._set_status(f"loi_serial:{exc}")
                time.sleep(2.0)
            except Exception as exc:
                self._attach_serial(None)
                self._set_status(f"loi_dashboard:{type(exc).__name__}:{exc}")
                time.sleep(2.0)
            finally:
                self._attach_serial(None)

    def _append_entry(self, entry: LogEntry) -> None:
        with self._lock:
            self.history.append(entry)
        with self.csv_path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                entry.time_ms,
                entry.edge_density,
                entry.mean_strength,
                entry.hand_detected,
                entry.finger_count,
                entry.min_x,
                entry.min_y,
                entry.max_x,
                entry.max_y,
                entry.frame_time_ms,
            ])

    def _parse_logcsv(self, raw: str) -> LogEntry | None:
        parts = raw.split(",")
        if len(parts) != 11:
            self._set_status("logcsv_khong_hop_le")
            return None

        try:
            return LogEntry(
                time_ms=int(parts[1]),
                edge_density=float(parts[2]),
                mean_strength=float(parts[3]),
                hand_detected=int(parts[4]),
                finger_count=int(parts[5]),
                min_x=int(parts[6]),
                min_y=int(parts[7]),
                max_x=int(parts[8]),
                max_y=int(parts[9]),
                frame_time_ms=int(parts[10]),
            )
        except ValueError:
            self._set_status("logcsv_parse_loi")
            return None

    def _set_status(self, status: str) -> None:
        with self._lock:
            self.latest_status = status

    def _set_last_raw_line(self, line: str) -> None:
        with self._lock:
            self.last_raw_line = line

    def _attach_serial(self, ser: serial.Serial | None) -> None:
        with self._serial_lock:
            self._serial = ser

    def _set_base64_status(self, mode: str, chars: int = 0, expected_len: int | None = None) -> None:
        with self._lock:
            self.base64_mode = mode
            self.base64_chars = chars
            self.expected_jpg_len = expected_len

    def _set_capture_status(self, status: str) -> None:
        with self._lock:
            self.capture_status = status

    def _finish_capture(self, esp_time_ms: int, chunks: list[str]) -> None:
        try:
            image_bytes = base64.b64decode("".join(chunks), validate=True)
        except Exception as exc:
            self._set_status(f"capture_base64_loi:{exc}")
            return

        filename = f"capture_{esp_time_ms}.jpg"
        path = CAPTURE_DIR / filename
        path.write_bytes(image_bytes)
        capture = {
            "filename": filename,
            "time_ms": esp_time_ms,
            "bytes": len(image_bytes),
            "pc_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with self._lock:
            self.capture_count += 1
            capture["index"] = self.capture_count
            self.latest_capture = capture

    def _parse_config_status(self, raw: str) -> None:
        config: dict[str, int | float] = {}
        for item in raw.removeprefix("CONFIG_STATUS,").split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            try:
                if "." in value:
                    config[key] = float(value)
                else:
                    config[key] = int(value)
            except ValueError:
                continue

        if config:
            with self._lock:
                self.config.update(config)
            self._set_status("config_da_cap_nhat")


def create_app(bridge: SerialLogBridge) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index() -> Response:
        html = """
<!doctype html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Fuzzy Logic Local Dashboard</title>
  <style>
    :root {
      --bg: #f2efe8;
      --panel: #fffaf0;
      --ink: #1c2526;
      --accent: #d06b2f;
      --line: #d7c9b6;
      --good: #2f7d4a;
      --warn: #a14b1a;
    }
    body {
      margin: 0;
      font-family: Georgia, "Times New Roman", serif;
      background: radial-gradient(circle at top left, #fff6de 0%, var(--bg) 45%, #e9e2d6 100%);
      color: var(--ink);
    }
    .wrap {
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero {
      background: linear-gradient(135deg, rgba(208,107,47,0.18), rgba(255,250,240,0.92));
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 24px;
      box-shadow: 0 12px 32px rgba(69, 46, 26, 0.08);
    }
    h1 {
      margin: 0 0 8px;
      font-size: 38px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 16px;
      margin-top: 20px;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 16px;
    }
    .panel {
      margin-top: 20px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 16px;
    }
    .preview {
      min-height: 220px;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #101820;
      color: #d8e4e8;
      border-radius: 12px;
      overflow: hidden;
    }
    .preview img {
      width: 100%;
      max-height: 420px;
      object-fit: contain;
      image-rendering: pixelated;
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 14px;
      align-items: end;
    }
    input[type="range"] {
      width: 100%;
    }
    button {
      border: 0;
      border-radius: 10px;
      padding: 12px 16px;
      background: var(--ink);
      color: white;
      font-weight: 700;
      cursor: pointer;
    }
    .label {
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: #6b645b;
      margin-bottom: 8px;
    }
    .value {
      font-size: 30px;
      font-weight: 700;
    }
    .good { color: var(--good); }
    .warn { color: var(--warn); }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 20px;
      background: var(--panel);
      border-radius: 16px;
      overflow: hidden;
    }
    th, td {
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      text-align: left;
      font-size: 14px;
    }
    th {
      background: rgba(208,107,47,0.12);
    }
    .meta {
      margin-top: 14px;
      font-size: 14px;
      color: #5e554c;
    }
    @media (max-width: 640px) {
      h1 { font-size: 30px; }
      .value { font-size: 24px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>Fuzzy Logic Local Dashboard</h1>
      <div id="meta" class="meta">Dang tai du lieu...</div>
      <div class="grid">
        <div class="card"><div class="label">Trang thai</div><div class="value" id="status">-</div></div>
        <div class="card"><div class="label">Edge Density</div><div class="value" id="edge_density">-</div></div>
        <div class="card"><div class="label">Mean Strength</div><div class="value" id="mean_strength">-</div></div>
        <div class="card"><div class="label">Hand</div><div class="value" id="hand_detected">-</div></div>
        <div class="card"><div class="label">Gesture</div><div class="value" id="finger_count">-</div></div>
        <div class="card"><div class="label">Frame Time</div><div class="value" id="frame_time_ms">-</div></div>
      </div>
    </section>
    <section class="panel">
      <div class="label">Anh debug</div>
      <div class="preview">
        <div id="capture-placeholder">Dang doi snapshot...</div>
        <img id="capture" alt="Latest ESP32-CAM capture" style="display:none">
      </div>
      <div class="meta" id="capture_meta">Captures: 0 | Base64: idle</div>
    </section>
    <section class="panel">
      <div class="label">Tune nhanh</div>
      <div class="controls">
        <div><div>Quality: <span id="qualityText">18</span></div><input id="quality" type="range" min="10" max="40" step="1" value="18"></div>
        <div><div>Brightness: <span id="brightnessText">-1</span></div><input id="brightness" type="range" min="-2" max="2" step="1" value="-1"></div>
        <div><div>Contrast: <span id="contrastText">2</span></div><input id="contrast" type="range" min="-2" max="2" step="1" value="2"></div>
        <div><div>Edge Dark: <span id="edgeDarkText">0.32</span></div><input id="edge_dark" type="range" min="0.18" max="0.55" step="0.01" value="0.32"></div>
        <div><div>Edge Bright: <span id="edgeBrightText">0.24</span></div><input id="edge_bright" type="range" min="0.16" max="0.50" step="0.01" value="0.24"></div>
        <div><div>Hand Threshold: <span id="handThresholdText">0.035</span></div><input id="hand_threshold" type="range" min="0.010" max="0.120" step="0.005" value="0.035"></div>
        <div><div>Snapshot ms: <span id="snapshotMsText">0</span></div><input id="snapshot_ms" type="range" min="0" max="30000" step="1000" value="0"></div>
        <button id="applyConfig" type="button">Apply</button>
        <button id="snapshotNow" type="button">Snapshot now</button>
      </div>
      <div class="meta" id="config_status">-</div>
    </section>
    <table>
      <thead>
        <tr>
          <th>time_ms</th>
          <th>edge_density</th>
          <th>mean_strength</th>
          <th>hand</th>
          <th>gesture</th>
          <th>bbox</th>
          <th>frame_time_ms</th>
        </tr>
      </thead>
      <tbody id="history_rows"></tbody>
    </table>
  </div>
  <script>
    function fmt(value, digits = 3) {
      if (value === null || value === undefined) return "-";
      if (typeof value === "number") return value.toFixed(digits);
      return String(value);
    }
    function gestureName(code) {
      if (code === 5) return "open";
      if (code === 1) return "fist";
      return "none";
    }
    function syncSliderText() {
      document.getElementById('qualityText').textContent = document.getElementById('quality').value;
      document.getElementById('brightnessText').textContent = document.getElementById('brightness').value;
      document.getElementById('contrastText').textContent = document.getElementById('contrast').value;
      document.getElementById('edgeDarkText').textContent = Number(document.getElementById('edge_dark').value).toFixed(2);
      document.getElementById('edgeBrightText').textContent = Number(document.getElementById('edge_bright').value).toFixed(2);
      document.getElementById('handThresholdText').textContent = Number(document.getElementById('hand_threshold').value).toFixed(3);
      document.getElementById('snapshotMsText').textContent = document.getElementById('snapshot_ms').value;
    }
    function setConfigInputs(cfg) {
      if (!cfg) return;
      if (document.activeElement && document.activeElement.type === 'range') return;
      document.getElementById('quality').value = cfg.quality ?? 18;
      document.getElementById('brightness').value = cfg.brightness ?? -1;
      document.getElementById('contrast').value = cfg.contrast ?? 2;
      document.getElementById('edge_dark').value = cfg.edge_dark ?? 0.32;
      document.getElementById('edge_bright').value = cfg.edge_bright ?? 0.24;
      document.getElementById('hand_threshold').value = cfg.hand_threshold ?? 0.035;
      document.getElementById('snapshot_ms').value = cfg.snapshot_ms ?? 0;
      syncSliderText();
    }

    async function refresh() {
      const res = await fetch('/api/state');
      const data = await res.json();
      document.getElementById('status').textContent = data.status;
      document.getElementById('meta').textContent = 'CSV local: ' + data.csv_path + ' | raw line: ' + data.last_raw_line;

      if (data.latest) {
        document.getElementById('edge_density').textContent = fmt(data.latest.edge_density);
        document.getElementById('mean_strength').textContent = fmt(data.latest.mean_strength);
        document.getElementById('hand_detected').textContent = data.latest.hand_detected ? 'yes' : 'no';
        document.getElementById('finger_count').textContent = gestureName(data.latest.finger_count);
        document.getElementById('frame_time_ms').textContent = data.latest.frame_time_ms + ' ms';
      }
      setConfigInputs(data.config);

      const base64Text = data.base64_mode !== 'idle'
        ? `${data.base64_mode} | ${data.base64_chars} chars / jpg ${data.expected_jpg_len || '?'} bytes`
        : 'idle';
      const waitText = data.snapshot_wait_s != null ? ` | Wait: ${data.snapshot_wait_s}s` : '';
      document.getElementById('capture_meta').textContent = `Captures: ${data.capture_count || 0} | Base64: ${base64Text} | Status: ${data.capture_status || 'idle'}${waitText}`;
      if (data.latest_capture) {
        document.getElementById('capture-placeholder').style.display = 'none';
        document.getElementById('capture').style.display = 'block';
        document.getElementById('capture').src = '/captures/' + data.latest_capture.filename + '?t=' + data.latest_capture.time_ms;
      }

      const rows = data.history.slice(-20).reverse().map((item) => {
        const bbox = `${item.min_x},${item.min_y},${item.max_x},${item.max_y}`;
        return `<tr>
          <td>${item.time_ms}</td>
          <td>${fmt(item.edge_density)}</td>
          <td>${fmt(item.mean_strength)}</td>
          <td>${item.hand_detected ? 'yes' : 'no'}</td>
          <td>${gestureName(item.finger_count)}</td>
          <td>${bbox}</td>
          <td>${item.frame_time_ms}</td>
        </tr>`;
      }).join('');
      document.getElementById('history_rows').innerHTML = rows;
    }

    async function applyConfig() {
      const payload = {
        quality: Number(document.getElementById('quality').value),
        brightness: Number(document.getElementById('brightness').value),
        contrast: Number(document.getElementById('contrast').value),
        edge_dark: Number(document.getElementById('edge_dark').value),
        edge_bright: Number(document.getElementById('edge_bright').value),
        hand_threshold: Number(document.getElementById('hand_threshold').value),
        snapshot_ms: Number(document.getElementById('snapshot_ms').value)
      };
      const res = await fetch('/api/config', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      document.getElementById('config_status').textContent = data.sent ? data.command : 'Serial chua san sang';
    }

    async function snapshotNow() {
      const res = await fetch('/api/snapshot', {method: 'POST'});
      const data = await res.json();
      document.getElementById('config_status').textContent = data.sent ? 'SNAP sent' : 'Serial chua san sang';
    }

    ['quality','brightness','contrast','edge_dark','edge_bright','hand_threshold','snapshot_ms'].forEach((id) => {
      document.getElementById(id).addEventListener('input', syncSliderText);
    });
    document.getElementById('applyConfig').addEventListener('click', applyConfig);
    document.getElementById('snapshotNow').addEventListener('click', snapshotNow);
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""
        return Response(html, mimetype="text/html")

    @app.get("/api/state")
    def api_state() -> Response:
        return jsonify(bridge.snapshot())

    @app.route("/api/config", methods=["GET", "POST"])
    def api_config() -> Response:
        if request.method == "GET":
            return jsonify(bridge.snapshot()["config"])

        payload = request.get_json(silent=True) or {}

        def clamp_float(name: str, default: float, lo: float, hi: float) -> float:
            try:
                value = float(payload.get(name, default))
            except (TypeError, ValueError):
                value = default
            return max(lo, min(hi, value))

        def clamp_int(name: str, default: int, lo: int, hi: int) -> int:
            try:
                value = int(payload.get(name, default))
            except (TypeError, ValueError):
                value = default
            return max(lo, min(hi, value))

        current = bridge.snapshot()["config"]
        config = {
            "quality": clamp_int("quality", int(current.get("quality", 18)), 10, 40),
            "brightness": clamp_int("brightness", int(current.get("brightness", -1)), -2, 2),
            "contrast": clamp_int("contrast", int(current.get("contrast", 2)), -2, 2),
            "edge_dark": clamp_float("edge_dark", float(current.get("edge_dark", 0.32)), 0.18, 0.55),
            "edge_bright": clamp_float("edge_bright", float(current.get("edge_bright", 0.24)), 0.16, 0.50),
            "hand_threshold": clamp_float(
                "hand_threshold", float(current.get("hand_threshold", 0.035)), 0.010, 0.120
            ),
            "snapshot_ms": clamp_int("snapshot_ms", int(current.get("snapshot_ms", 0)), 0, 60000),
        }
        command = (
            "SET "
            f"quality={config['quality']} "
            f"brightness={config['brightness']} "
            f"contrast={config['contrast']} "
            f"edge_dark={config['edge_dark']:.3f} "
            f"edge_bright={config['edge_bright']:.3f} "
            f"hand_threshold={config['hand_threshold']:.3f} "
            f"snapshot_ms={config['snapshot_ms']}"
        )
        sent = bridge.send_line(command)
        if sent:
            with bridge._lock:
                bridge.config.update(config)
        return jsonify({"sent": sent, "command": command, "config": config})

    @app.post("/api/snapshot")
    def api_snapshot() -> Response:
        sent = bridge.send_line("SNAP")
        if sent:
            with bridge._lock:
                bridge.capture_status = "requested_by_web"
                bridge.last_snapshot_request_at = time.time()
        return jsonify({"sent": sent, "command": "SNAP"})

    @app.get("/captures/<path:filename>")
    def capture_file(filename: str) -> Response:
        return send_from_directory(CAPTURE_DIR, filename)

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dashboard local cho ESP32-CAM fuzzy log")
    parser.add_argument("--port", required=True, help="Cong serial, vi du COM5 hoac /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Toc do serial")
    parser.add_argument("--host", default="127.0.0.1", help="Host web local")
    parser.add_argument("--web-port", type=int, default=5000, help="Cong web local")
    parser.add_argument(
        "--csv-path",
        default="pc_fuzzy_log.csv",
        help="File CSV luu log tren may tinh",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = SerialLogBridge(args.port, args.baudrate, Path(args.csv_path))
    bridge.start()
    app = create_app(bridge)
    try:
        app.run(host=args.host, port=args.web_port, debug=False)
    finally:
        bridge.stop()


if __name__ == "__main__":
  main()
