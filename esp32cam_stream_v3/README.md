# esp32cam_stream_v3

Kien truc moi: **ESP32-CAM = dumb streamer**, **PC = fuzzy + ROI + finger counter**.

Phien ban truoc (`esp32cam_local_v2/`, `esp32cam_stream/`) duoc **giu nguyen lam tham khao** — khong xoa.

## Khac biet voi v2

| | v2 (Serial, 10s/cap) | v3 (Wi-Fi stream) |
|---|---|---|
| Vat ly | USB Serial 115200 | Wi-Fi HTTP |
| Tan suat | 1 frame/10s | 2 fps (configurable) |
| ESP lam gi | Capture + LED flash + base64 dump | Capture + HTTP serve (khong fuzzy) |
| PC lam gi | Decode + fuzzy 2-class (open/fist) | ROI + finger count + fuzzy 3-class |
| Frame size | QQVGA 160x120 | QVGA 320x240 |
| Class | open / fist / none | open / pointing / fist / none + finger_count 0-5 |
| ROI selection | Khong (toan anh) | Connected components, chon blob gan tam |
| Finger detection | Column projection peaks | OpenCV convexity defects |

## Yeu cau

PC dependencies (da cai vao `/mnt/d/HOCTAP/.venv/`):
```
opencv-python  4.13
scipy          1.17
flask          3.1
pyserial       3.5     (chua dung — phong cho fallback)
numpy, Pillow, requests
```

ESP32-CAM AI-Thinker. Sketch dung Arduino IDE / PlatformIO, ESP32 core 3.x.

## Cach chay

### Buoc 1: Flash sketch v3

Mo `esp32cam_stream_v3.ino` bang Arduino IDE:
- Board: **AI Thinker ESP32-CAM**
- Partition: **Huge APP (3MB No OTA)** (do code + driver camera nang)
- Sua `WIFI_SSID` / `WIFI_PASS` neu khac
- Flash, mo Serial Monitor 115200 -> lay IP (vd `192.168.1.50`)

Truy cap `http://<IP>/` de check stream.

### Buoc 2: Chay dashboard PC

```bash
cd /mnt/d/HOCTAP/PROJECT-CE320.Q21/esp32cam_stream_v3

/mnt/d/HOCTAP/.venv/bin/python pc_stream_dashboard.py \
    --esp http://192.168.1.50 \
    --period 0.5 \
    --port 5001
```

Mo `http://127.0.0.1:5001` — thay 2 panel:
- **Trai**: anh raw vua pull tu ESP
- **Phai**: overlay PC (mask xanh + bbox vang + hull magenta + fingertips vang + label gesture)

Verdict hien o duoi: gesture, confidence, finger_count, solidity, area_ratio, reason.

CSV log ghi vao `pc_v3_log.csv` (cot day du tat ca metric).

## Cac module Python

| File | Vai tro |
|---|---|
| `pc_roi.py` | Otsu + morphology + `scipy.ndimage.label` -> chon blob "main hand" theo `score = area_ratio / (1 + 2.5*dist_to_center_norm)`. Loc nhung blob < 3% frame. |
| `pc_finger_count.py` | `cv2.convexHull` + `cv2.convexityDefects` -> dem ngon. Loc defect theo depth (>=12000 fixed-point) va angle (<=95°). Khong defect + solidity cao -> fist; thap -> pointing. |
| `pc_fuzzy.py` | Membership functions (trap/triangle) tren `finger_count`, `solidity`, `area_ratio`, `aspect`. 7 Mamdani rules (R1-R7). Defuzzify max-score. |
| `pc_stream_dashboard.py` | Flask + worker thread pull /capture moi 500ms. Hien web UI + CSV log. |

## Fuzzy rules (tom tat)

```
R1: many_fingers & loose_solidity & area_ok          -> OPEN
R2: few_fingers (1-2) & area_ok                      -> POINTING
R3: zero_fingers & compact & area_ok                 -> FIST
R4: area >= 0.18 & n >= 3                            -> bias OPEN (tay gan camera)
R5: aspect cuc (xoay ngang) & area >= 0.15 & n >= 2  -> bias OPEN
R6: touches_edge & n >= 2                            -> bias OPEN (vuot khung)
R7: solidity >= 0.92 & n >= 1                        -> override FIST (contradict guard)
```

Output `gesture = argmax(open_score, pointing_score, fist_score)`, conf = `best/total`.
Neu best < 0.30 -> `none`.

## Tuning checklist

Neu accuracy thap khi test that:

1. **Anh qua toi / qua sang** -> chinh `set_ae_level()` trong sketch (range -2..2). Hien tai dat 2 (sang).
2. **Fingers dem sai (0 thay vi nhieu)** -> giam `MIN_DEFECT_DEPTH` trong `pc_finger_count.py` (vd 8000).
3. **Fingers dem du** -> tang `MIN_DEFECT_DEPTH` len 16000-20000.
4. **ROI nham background** -> tang `MIN_AREA_RATIO` trong `pc_roi.py` (vd 0.05) hoac doi cong thuc score.
5. **Tay it tach khoi nen** -> can phong toi/sang co contrast. Otsu chi work khi co 2 cum gray ro ret.

## Endpoints ESP32

- `GET /` — index don gian (link toi stream)
- `GET /stream` — MJPEG cho preview
- `GET /capture` — 1 JPEG tuoi moi request (PC dung cai nay)
- `GET /info` — JSON: `frame_size`, `frame_counter`, `last_capture_ms`, `uptime_ms`, `free_heap`

## Test nhanh khong can ESP

```bash
cd /mnt/d/HOCTAP/PROJECT-CE320.Q21/esp32cam_stream_v3
/mnt/d/HOCTAP/.venv/bin/python -c "
import cv2, numpy as np, pc_fuzzy
img = np.zeros((240,240), dtype=np.uint8)
cv2.rectangle(img,(90,110),(150,220),255,-1)   # palm
for cx in (100,120,140):
    cv2.rectangle(img,(cx-5,60),(cx+5,115),255,-1)  # 3 fingers
ok, jpg = cv2.imencode('.jpg', img)
res, roi, fr = pc_fuzzy.analyze_jpeg(jpg.tobytes())
print(res)
"
```

Expected: `gesture='open' fingers=3 confidence~1.0`.
