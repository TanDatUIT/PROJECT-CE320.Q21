"""
Dashboard PC v3 — voi background calibration + skin HSV.

Worker pull /capture moi ~500ms tu ESP32-CAM. Pipeline:
    JPEG -> RGB -> skin HSV mask (+ background diff neu da calibrate)
    -> connected components -> chon blob hand
    -> fuzzy 3-class (open/pointing/fist) + finger_count 0..5

User flow:
    1) Mo dashboard, nhan "Calibrate Background"
    2) Rut tay khoi khung 3 giay
    3) Sau khi calibrate xong -> dua tay vao, dashboard nhan dien

Su dung:
    /mnt/d/HOCTAP/.venv/bin/python pc_stream_dashboard.py
    Mo http://127.0.0.1:5001
"""

from __future__ import annotations

import argparse
import csv
import io
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import requests
from flask import Flask, jsonify, render_template_string, request, send_file

from pc_background import BackgroundReference
from pc_fuzzy import analyze_jpeg
from pc_roi import decode_jpeg_rgb, draw_debug_overlay, build_skin_mask, TUNABLE as ROI_TUNABLE
from pc_finger_count import draw_finger_overlay, TUNABLE as FINGER_TUNABLE


# ============ Shared state ============

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_raw_jpeg: bytes = b""
        self.last_overlay_jpeg: bytes = b""
        self.last_verdict: dict = {
            "gesture": "—",
            "confidence": 0.0,
            "finger_count": 0,
            "area_ratio": 0.0,
            "skin_area_ratio": 0.0,
            "solidity": 0.0,
            "aspect": 0.0,
            "touches_edge": False,
            "roi_found": False,
            "used_background": False,
            "reason": "starting...",
            "bbox": [-1, -1, -1, -1],
            "esp_capture_ms": 0,
            "pc_recv_ts": 0.0,
            "round_trip_ms": 0,
        }
        self.error: str = ""
        self.frame_counter = 0
        self.bg = BackgroundReference()
        # 'idle' | 'calibrating' | 'calibrated' | 'error'
        self.bg_status: str = "idle"
        self.bg_progress: tuple[int, int] = (0, 0)
        self.bg_calibrated_age_s: float = -1.0


STATE = SharedState()
ESP_BASE_URL = ""   # set trong main()


# ============ Worker thread: pull /capture loop ============

def pull_worker(
    esp_url: str,
    period_s: float,
    csv_path: Path | None,
    debug_dir: Path | None,
    debug_every: int,
):
    capture_url = esp_url.rstrip("/") + "/capture"
    print(f"[worker] pulling {capture_url} every {period_s*1000:.0f} ms")

    csv_file = None
    csv_writer = None
    if csv_path is not None:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not csv_path.exists()
        csv_file = csv_path.open("a", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        if is_new:
            csv_writer.writerow([
                "pc_ts", "esp_capture_ms", "round_trip_ms",
                "gesture", "confidence", "finger_count",
                "solidity", "area_ratio", "skin_area_ratio", "aspect", "touches_edge",
                "used_background",
                "bbox_x0", "bbox_y0", "bbox_x1", "bbox_y1", "reason",
                # ROI/finger extras
                "defects_used", "defects_total", "hull_area", "contour_area",
                "skin_s_min", "skin_v_min", "skin_h_max", "bg_diff_thr",
                "min_defect_depth_ratio", "max_defect_angle",
                "ae_level", "aec_value", "agc_gain",
                "debug_basename",
            ])

    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)

    sess = requests.Session()

    while True:
        t0 = time.time()
        try:
            r = sess.get(capture_url, timeout=3.0)
            r.raise_for_status()
            jpg_bytes = r.content
            esp_ms = int(r.headers.get("X-Capture-Ms", "0"))
        except Exception as exc:
            with STATE.lock:
                STATE.error = f"pull_failed: {exc}"
                STATE.bg_status = STATE.bg_status   # giu nguyen
            time.sleep(period_s)
            continue

        # === Calibration: dang thu thap frame BG ===
        bg_collecting = STATE.bg.is_collecting()
        if bg_collecting:
            rgb = decode_jpeg_rgb(jpg_bytes)
            if rgb is not None:
                gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                done = STATE.bg.feed_frame(gray)
                # Tao overlay don gian: hien tien do
                collected, target = STATE.bg.progress() if not done else (
                    STATE.bg._target_count, STATE.bg._target_count
                )
                preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                msg = "DONE!" if done else f"Calibrating BG {collected}/{target} — KHONG GIO TAY"
                cv2.putText(
                    preview, msg, (5, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 2, cv2.LINE_AA,
                )
                ok, overlay_jpg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                overlay_bytes = overlay_jpg.tobytes() if ok else b""

                with STATE.lock:
                    STATE.last_raw_jpeg = jpg_bytes
                    STATE.last_overlay_jpeg = overlay_bytes
                    STATE.bg_progress = (collected, target)
                    STATE.bg_status = "calibrated" if done else "calibrating"
                    STATE.error = ""
                    STATE.frame_counter += 1
                    STATE.last_verdict["reason"] = (
                        f"BG_CALIBRATE {collected}/{target}" if not done else "BG_READY"
                    )

            elapsed = time.time() - t0
            if elapsed < period_s:
                time.sleep(period_s - elapsed)
            continue

        # === Phan tich binh thuong ===
        fuzzy_res, roi_res, finger_res = analyze_jpeg(jpg_bytes, bg=STATE.bg)

        # Tao overlay
        overlay = draw_debug_overlay(roi_res)
        if roi_res.found:
            overlay = draw_finger_overlay(overlay, roi_res.mask, finger_res)
            color = (0, 255, 0) if fuzzy_res.gesture != "none" else (0, 0, 255)
            cv2.putText(
                overlay,
                f"{fuzzy_res.gesture.upper()} ({fuzzy_res.confidence:.2f})",
                (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
            )

        ok, overlay_jpg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 80])
        overlay_bytes = overlay_jpg.tobytes() if ok else b""

        round_trip = int((time.time() - t0) * 1000)
        verdict = fuzzy_res.to_dict()
        verdict.update({
            "esp_capture_ms": esp_ms,
            "pc_recv_ts": time.time(),
            "round_trip_ms": round_trip,
            "skin_area_ratio": roi_res.skin_area_ratio,
            "used_background": roi_res.used_background,
        })

        bg_age = STATE.bg.calibrated_age_s()
        bg_status = "calibrated" if STATE.bg.is_ready() else "idle"

        with STATE.lock:
            STATE.last_raw_jpeg = jpg_bytes
            STATE.last_overlay_jpeg = overlay_bytes
            STATE.last_verdict = verdict
            STATE.error = ""
            STATE.frame_counter += 1
            STATE.bg_status = bg_status
            STATE.bg_calibrated_age_s = bg_age

        # Debug image dump (every N frames)
        debug_basename = ""
        if debug_dir is not None and debug_every > 0 and (STATE.frame_counter % debug_every == 0):
            ts_tag = f"{int(time.time()*1000) % 100000000:08d}"
            debug_basename = f"f{STATE.frame_counter:06d}_{ts_tag}_{fuzzy_res.gesture}"
            try:
                (debug_dir / f"{debug_basename}_raw.jpg").write_bytes(jpg_bytes)
                if overlay_bytes:
                    (debug_dir / f"{debug_basename}_overlay.jpg").write_bytes(overlay_bytes)
                # Skin mask + final mask (truc tiep tu pipeline)
                rgb_dbg = decode_jpeg_rgb(jpg_bytes)
                if rgb_dbg is not None:
                    skin_dbg = build_skin_mask(rgb_dbg)
                    cv2.imwrite(str(debug_dir / f"{debug_basename}_skin.png"), skin_dbg)
                if roi_res.found:
                    cv2.imwrite(str(debug_dir / f"{debug_basename}_mask.png"), roi_res.mask)
            except Exception as exc:
                with STATE.lock:
                    STATE.error = f"debug_save_failed: {exc}"

        if csv_writer is not None:
            csv_writer.writerow([
                f"{time.time():.3f}", esp_ms, round_trip,
                fuzzy_res.gesture, fuzzy_res.confidence, fuzzy_res.finger_count,
                fuzzy_res.solidity, fuzzy_res.area_ratio,
                roi_res.skin_area_ratio, fuzzy_res.aspect,
                int(fuzzy_res.touches_edge), int(roi_res.used_background),
                fuzzy_res.bbox[0], fuzzy_res.bbox[1],
                fuzzy_res.bbox[2], fuzzy_res.bbox[3],
                fuzzy_res.reason,
                finger_res.defects_used, finger_res.defects_total,
                finger_res.hull_area, finger_res.contour_area,
                ROI_TUNABLE["skin_s_min"], ROI_TUNABLE["skin_v_min"],
                ROI_TUNABLE["skin_h_max"], ROI_TUNABLE["bg_diff_thr"],
                FINGER_TUNABLE["min_defect_depth_ratio"], FINGER_TUNABLE["max_defect_angle"],
                "—", "—", "—",   # cam params hien chua track tu PC
                debug_basename,
            ])
            csv_file.flush()

        elapsed = time.time() - t0
        if elapsed < period_s:
            time.sleep(period_s - elapsed)


# ============ Flask app ============

app = Flask(__name__)


INDEX_HTML = """
<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8">
<title>ESP32-CAM v3 — PC Fuzzy Dashboard</title>
<style>
  body { font-family: -apple-system, "Segoe UI", sans-serif; background:#1b1d22; color:#eaeaea; margin:0; padding:18px; }
  h1 { font-size:20px; margin:0 0 12px; }
  .toolbar { background:#262931; border-radius:10px; padding:10px 14px; margin-bottom:12px; max-width:1300px; display:flex; gap:12px; align-items:center; flex-wrap:wrap; }
  button { background:#3a7afe; color:#fff; border:0; padding:9px 16px; border-radius:8px; cursor:pointer; font-weight:600; }
  button:hover { background:#558dff; }
  button.reset { background:#555; }
  .pill { padding:6px 12px; border-radius:14px; font-size:12px; font-weight:600; }
  .pill.idle { background:#444; color:#bbb; }
  .pill.calibrating { background:#f0b441; color:#221; }
  .pill.calibrated { background:#3aa55a; color:#fff; }
  .pill.error { background:#cc3a3a; color:#fff; }
  .grid { display:grid; grid-template-columns: 1fr 1fr; gap:14px; max-width:1300px; }
  .card { background:#262931; border-radius:10px; padding:12px; }
  img { width:100%; border-radius:8px; background:#000; display:block; }
  .verdict { margin-top:12px; background:#1f2229; border-radius:10px; padding:12px; max-width:1300px; }
  .gesture-big { font-size:34px; font-weight:700; }
  .open { color:#48d597; } .fist { color:#ff7474; } .pointing { color:#ffd166; } .none { color:#888; }
  table { width:100%; border-collapse:collapse; font-size:13px; margin-top:8px; }
  td { padding:5px 8px; border-bottom:1px solid #333; }
  td:first-child { color:#8b95a7; width:170px; }
  .err { color:#ff7474; margin-top:6px; font-size:12px; }
  .src-label { font-size:12px; color:#8b95a7; margin-bottom:4px; }
  .slider-row { display:flex; align-items:center; gap:10px; margin:4px 0; }
  .slider-row label { min-width:230px; font-size:13px; color:#aeb6c2; }
  .slider-row input[type=range] { flex:1; max-width:420px; }
  .slider-row span { min-width:44px; font-family:monospace; color:#fff; }
</style>
</head>
<body>
  <h1>ESP32-CAM v3 — PC Fuzzy Dashboard</h1>

  <div class="toolbar">
    <button id="btn_calib">📷 Calibrate Background</button>
    <button id="btn_reset" class="reset">↺ Reset BG</button>
    <span>BG status:</span>
    <span id="bg_pill" class="pill idle">idle</span>
    <span id="bg_extra" style="color:#888; font-size:12px;"></span>
  </div>

  <div class="toolbar" style="flex-direction:column; align-items:stretch;">
    <div style="font-weight:600; color:#9bb;">Camera tuning (gui truc tiep toi ESP)</div>
    <div class="slider-row">
      <label>AE level (-2..+2, am=toi):</label>
      <input type="range" id="s_ae" min="-2" max="2" step="1" value="0">
      <span id="v_ae">0</span>
    </div>
    <div class="slider-row">
      <label>Brightness (-2..+2):</label>
      <input type="range" id="s_br" min="-2" max="2" step="1" value="1">
      <span id="v_br">1</span>
    </div>
    <div class="slider-row">
      <label>Contrast (-2..+2):</label>
      <input type="range" id="s_co" min="-2" max="2" step="1" value="2">
      <span id="v_co">2</span>
    </div>
    <div class="slider-row">
      <label>AEC value (0..1200, lon=sang):</label>
      <input type="range" id="s_aecv" min="0" max="1200" step="20" value="400">
      <span id="v_aecv">400</span>
    </div>
    <div class="slider-row">
      <label>AGC gain (0..30):</label>
      <input type="range" id="s_agc" min="0" max="30" step="1" value="0">
      <span id="v_agc">0</span>
    </div>
    <div class="slider-row" style="gap:14px;">
      <label><input type="checkbox" id="s_aec_auto" checked> Auto Exposure (AEC)</label>
      <label><input type="checkbox" id="s_agc_auto" checked> Auto Gain (AGC)</label>
    </div>
  </div>

  <div class="toolbar" style="flex-direction:column; align-items:stretch;">
    <div style="font-weight:600; color:#9bb;">ROI / Skin tuning (giam S_min neu tay bi chay sang)</div>
    <div class="slider-row">
      <label>Skin S min (0..80):</label>
      <input type="range" id="r_smin" min="0" max="80" step="1" value="15">
      <span id="vr_smin">15</span>
    </div>
    <div class="slider-row">
      <label>Skin V min (0..120):</label>
      <input type="range" id="r_vmin" min="0" max="120" step="2" value="40">
      <span id="vr_vmin">40</span>
    </div>
    <div class="slider-row">
      <label>Skin V max (200..255):</label>
      <input type="range" id="r_vmax" min="200" max="255" step="1" value="255">
      <span id="vr_vmax">255</span>
    </div>
    <div class="slider-row">
      <label>Skin H max (10..40):</label>
      <input type="range" id="r_hmax" min="10" max="40" step="1" value="25">
      <span id="vr_hmax">25</span>
    </div>
    <div class="slider-row">
      <label>BG diff threshold (5..60):</label>
      <input type="range" id="r_bg" min="5" max="60" step="1" value="22">
      <span id="vr_bg">22</span>
    </div>
    <div class="slider-row">
      <label>Min area ratio (0.005..0.10):</label>
      <input type="range" id="r_area" min="0.005" max="0.10" step="0.005" value="0.025">
      <span id="vr_area">0.025</span>
    </div>
  </div>

  <div class="toolbar" style="flex-direction:column; align-items:stretch;">
    <div style="font-weight:600; color:#9bb;">Finger tuning (giam de dem nhieu ngon hon)</div>
    <div class="slider-row">
      <label>Min defect depth ratio (0.03..0.15):</label>
      <input type="range" id="f_depth" min="0.03" max="0.15" step="0.005" value="0.07">
      <span id="vf_depth">0.07</span>
    </div>
    <div class="slider-row">
      <label>Max defect angle (60..130°):</label>
      <input type="range" id="f_angle" min="60" max="130" step="1" value="100">
      <span id="vf_angle">100</span>
    </div>
    <div class="slider-row">
      <label>Solidity FIST high (>=0.85..0.98):</label>
      <input type="range" id="f_sh" min="0.85" max="0.98" step="0.01" value="0.92">
      <span id="vf_sh">0.92</span>
    </div>
    <div class="slider-row">
      <label>Solidity FIST med (>=0.70..0.92):</label>
      <input type="range" id="f_sm" min="0.70" max="0.92" step="0.01" value="0.85">
      <span id="vf_sm">0.85</span>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="src-label">RAW /capture (PC pulled)</div>
      <img id="raw" src="/raw.jpg">
    </div>
    <div class="card">
      <div class="src-label">PC ROI + Fingers + Fuzzy</div>
      <img id="overlay" src="/overlay.jpg">
    </div>
  </div>

  <div class="verdict">
    <div>Gesture: <span id="gesture" class="gesture-big none">—</span>
         <span id="conf" style="font-size:18px;color:#8b95a7;"></span></div>
    <table>
      <tr><td>finger_count</td><td id="fingers">—</td></tr>
      <tr><td>solidity</td><td id="solidity">—</td></tr>
      <tr><td>area_ratio</td><td id="area">—</td></tr>
      <tr><td>skin_area_ratio</td><td id="skin">—</td></tr>
      <tr><td>aspect (h/w)</td><td id="aspect">—</td></tr>
      <tr><td>touches_edge</td><td id="edge">—</td></tr>
      <tr><td>used_background</td><td id="usedbg">—</td></tr>
      <tr><td>roi_found</td><td id="roi">—</td></tr>
      <tr><td>bbox</td><td id="bbox">—</td></tr>
      <tr><td>esp_capture_ms</td><td id="ems">—</td></tr>
      <tr><td>round_trip_ms</td><td id="rtt">—</td></tr>
      <tr><td>reason</td><td id="reason" style="font-family:monospace;font-size:11px;">—</td></tr>
    </table>
    <div id="err" class="err"></div>
  </div>

<script>
document.getElementById('btn_calib').onclick = async () => {
  if (!confirm('Rut tay (va vat di dong) khoi khung. Bam OK de bat dau thu thap ~10 frame.')) return;
  try {
    await fetch('/calibrate', {method:'POST'});
  } catch(e) {}
};
document.getElementById('btn_reset').onclick = async () => {
  try { await fetch('/reset_bg', {method:'POST'}); } catch(e) {}
};

// Camera sliders
async function sendCam(varName, val) {
  try { await fetch(`/cam_control?var=${varName}&val=${val}`, {method:'POST'}); } catch(e) {}
}
function bindSlider(id, varName, displayId) {
  const el = document.getElementById(id);
  el.addEventListener('input', () => {
    document.getElementById(displayId).textContent = el.value;
  });
  el.addEventListener('change', () => sendCam(varName, el.value));
}
bindSlider('s_ae',   'ae_level',  'v_ae');
bindSlider('s_br',   'brightness','v_br');
bindSlider('s_co',   'contrast',  'v_co');
bindSlider('s_aecv', 'aec_value', 'v_aecv');
bindSlider('s_agc',  'agc_gain',  'v_agc');
document.getElementById('s_aec_auto').addEventListener('change', e => sendCam('aec', e.target.checked ? 1 : 0));
document.getElementById('s_agc_auto').addEventListener('change', e => sendCam('agc', e.target.checked ? 1 : 0));

// Finger tune sliders
async function sendFinger(key, val) {
  try { await fetch(`/finger_tune?key=${key}&val=${val}`, {method:'POST'}); } catch(e) {}
}
function bindFinger(id, key, displayId) {
  const el = document.getElementById(id);
  el.addEventListener('input', () => { document.getElementById(displayId).textContent = el.value; });
  el.addEventListener('change', () => sendFinger(key, el.value));
}
bindFinger('f_depth', 'min_defect_depth_ratio', 'vf_depth');
bindFinger('f_angle', 'max_defect_angle',       'vf_angle');
bindFinger('f_sh',    'solidity_fist_high',     'vf_sh');
bindFinger('f_sm',    'solidity_fist_med',      'vf_sm');

// ROI tune sliders
async function sendRoi(key, val) {
  try { await fetch(`/roi_tune?key=${key}&val=${val}`, {method:'POST'}); } catch(e) {}
}
function bindRoi(id, key, displayId) {
  const el = document.getElementById(id);
  el.addEventListener('input', () => { document.getElementById(displayId).textContent = el.value; });
  el.addEventListener('change', () => sendRoi(key, el.value));
}
bindRoi('r_smin', 'skin_s_min',    'vr_smin');
bindRoi('r_vmin', 'skin_v_min',    'vr_vmin');
bindRoi('r_vmax', 'skin_v_max',    'vr_vmax');
bindRoi('r_hmax', 'skin_h_max',    'vr_hmax');
bindRoi('r_bg',   'bg_diff_thr',   'vr_bg');
bindRoi('r_area', 'min_area_ratio','vr_area');

async function tick() {
  try {
    const r = await fetch('/state');
    const s = await r.json();
    const v = s.verdict;
    const g = document.getElementById('gesture');
    g.textContent = (v.gesture || '—').toUpperCase();
    g.className = 'gesture-big ' + (v.gesture || 'none');
    document.getElementById('conf').textContent = '  conf=' + Number(v.confidence||0).toFixed(2);
    document.getElementById('fingers').textContent = v.finger_count;
    document.getElementById('solidity').textContent = Number(v.solidity||0).toFixed(3);
    document.getElementById('area').textContent = Number(v.area_ratio||0).toFixed(3);
    document.getElementById('skin').textContent = Number(v.skin_area_ratio||0).toFixed(3);
    document.getElementById('aspect').textContent = Number(v.aspect||0).toFixed(3);
    document.getElementById('edge').textContent = v.touches_edge ? 'YES' : 'no';
    document.getElementById('usedbg').textContent = v.used_background ? 'YES' : 'no';
    document.getElementById('roi').textContent = v.roi_found ? 'YES' : 'no';
    document.getElementById('bbox').textContent = '[' + (v.bbox||[]).join(', ') + ']';
    document.getElementById('ems').textContent = v.esp_capture_ms;
    document.getElementById('rtt').textContent = v.round_trip_ms + ' ms';
    document.getElementById('reason').textContent = v.reason;
    document.getElementById('err').textContent = s.error || '';
    const pill = document.getElementById('bg_pill');
    pill.textContent = s.bg_status;
    pill.className = 'pill ' + s.bg_status;
    let extra = '';
    if (s.bg_status === 'calibrating') extra = ` (${s.bg_progress[0]}/${s.bg_progress[1]})`;
    else if (s.bg_status === 'calibrated' && s.bg_age_s >= 0) extra = ` (age ${s.bg_age_s.toFixed(0)}s)`;
    document.getElementById('bg_extra').textContent = extra;
    const ts = Date.now();
    document.getElementById('raw').src = '/raw.jpg?t=' + ts;
    document.getElementById('overlay').src = '/overlay.jpg?t=' + ts;
  } catch(e) {}
}
setInterval(tick, 500);
tick();
</script>
</body></html>
"""


@app.route("/")
def index():
    return render_template_string(INDEX_HTML)


@app.route("/raw.jpg")
def raw_jpg():
    with STATE.lock:
        data = STATE.last_raw_jpeg
    if not data:
        return ("", 204)
    return send_file(io.BytesIO(data), mimetype="image/jpeg")


@app.route("/overlay.jpg")
def overlay_jpg():
    with STATE.lock:
        data = STATE.last_overlay_jpeg
    if not data:
        return ("", 204)
    return send_file(io.BytesIO(data), mimetype="image/jpeg")


@app.route("/state")
def state_json():
    with STATE.lock:
        bg_status = STATE.bg_status
        progress = STATE.bg_progress
        age = STATE.bg_calibrated_age_s
        return jsonify({
            "verdict": STATE.last_verdict,
            "error": STATE.error,
            "frame_counter": STATE.frame_counter,
            "bg_status": bg_status,
            "bg_progress": list(progress),
            "bg_age_s": age,
        })


@app.route("/calibrate", methods=["POST"])
def trigger_calibrate():
    STATE.bg.start_calibration(target_count=10)
    with STATE.lock:
        STATE.bg_status = "calibrating"
        STATE.bg_progress = (0, 10)
    return jsonify({"ok": True, "msg": "Calibration started (10 frames)."})


@app.route("/roi_tune", methods=["POST"])
def roi_tune():
    key = request.args.get("key", "")
    val = request.args.get("val", "")
    if key not in ROI_TUNABLE:
        return jsonify({"ok": False, "error": f"unknown key {key}"}), 400
    try:
        ROI_TUNABLE[key] = float(val)
    except ValueError:
        return jsonify({"ok": False, "error": "val not float"}), 400
    return jsonify({"ok": True, "tunable": ROI_TUNABLE})


@app.route("/finger_tune", methods=["POST"])
def finger_tune():
    key = request.args.get("key", "")
    val = request.args.get("val", "")
    if key not in FINGER_TUNABLE:
        return jsonify({"ok": False, "error": f"unknown key {key}"}), 400
    try:
        FINGER_TUNABLE[key] = float(val)
    except ValueError:
        return jsonify({"ok": False, "error": "val not float"}), 400
    return jsonify({"ok": True, "tunable": FINGER_TUNABLE})


@app.route("/cam_control", methods=["POST"])
def cam_control():
    """Proxy toi /control cua ESP32-CAM."""
    var = request.args.get("var", "")
    val = request.args.get("val", "")
    if not var or val == "":
        return jsonify({"ok": False, "error": "need var, val"}), 400
    url = f"{ESP_BASE_URL.rstrip('/')}/control?var={var}&val={val}"
    try:
        r = requests.get(url, timeout=2.0)
        return jsonify({"ok": True, "esp_status": r.status_code, "body": r.text})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 502


@app.route("/reset_bg", methods=["POST"])
def trigger_reset_bg():
    STATE.bg.reset()
    with STATE.lock:
        STATE.bg_status = "idle"
        STATE.bg_progress = (0, 0)
        STATE.bg_calibrated_age_s = -1.0
    return jsonify({"ok": True, "msg": "Background reset."})


# ============ Main ============

def main():
    # === SUA IP ESP32-CAM O DAY (xem Serial Monitor sau khi flash) ===
    ESP_IP = "10.53.54.182"
    # ================================================================
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp", default=f"http://{ESP_IP}",
                    help=f"Base URL cua ESP32-CAM, default http://{ESP_IP}")
    ap.add_argument("--period", type=float, default=0.5,
                    help="Khoang thoi gian pull /capture (giay), default 0.5")
    ap.add_argument("--port", type=int, default=5001,
                    help="Port web dashboard PC, default 5001")
    ap.add_argument("--csv", default="pc_v3_log.csv",
                    help="File CSV ghi log (de '' de tat)")
    ap.add_argument("--debug-dir", default="debug_frames",
                    help="Thu muc luu raw/overlay/skin moi N frame (de '' de tat)")
    ap.add_argument("--debug-every", type=int, default=5,
                    help="Luu debug moi N frame (mac dinh 5)")
    args = ap.parse_args()

    csv_path = Path(args.csv) if args.csv else None
    debug_dir = Path(args.debug_dir) if args.debug_dir else None
    global ESP_BASE_URL
    ESP_BASE_URL = args.esp

    worker = threading.Thread(
        target=pull_worker,
        args=(args.esp, args.period, csv_path, debug_dir, args.debug_every),
        daemon=True,
    )
    worker.start()

    print(f"Dashboard chay tai http://127.0.0.1:{args.port}")
    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
