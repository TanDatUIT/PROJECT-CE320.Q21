# ESP32-CAM PC FUZZY

Project tach ESP32-CAM va PC FUZZY:

```text
ESP32-CAM -> chup JPEG / stream qua Wi-Fi
PC Python -> MediaPipe landmark + fuzzy logic + OpenCV fallback
Web local -> xem anh, ket qua, log, chinh camera, snap debug
```

Muc tieu: ESP32-CAM chi phu trach camera, PC xu ly nhan dien manh hon.

## 1. Trang thai moi nhat

Cap nhat ngay 2026-05-28:

- PC server dung `MediaPipe Tasks Hand Landmarker` lam backend chinh.
- Ket qua bai toan hien tai la `none`, `fist`, va `open` kem so ngon `fingers=1..5`.
- `fist` duoc tinh rieng: nam tay thi `gesture=fist`, `fingers=0`.
- Khi MediaPipe khong thay tay, server thu `opencv_fallback` de bat `fist/open` theo contour.
- Khi chi co 1-4 ngon va bi cat mat long ban tay, server dung `opencv_partial_fingers`.
- Web co phim `S` de chup `Snap Debug`.
- Snap debug moi luu raw frame sach, anh annotated va JSON metadata.

## 2. Gesture hien tai

Ket qua chinh tren web gom `gesture`, `fingers`, `mode`, `reason`.

```text
none          khong thay tay hop le
fist          nam tay, fingers=0
open f=1..5   co ngon tay dang gio; so ngon nam trong truong fingers
```

Luu y: voi 1-4 ngon bi cat mat long ban tay, server co the hien `mode=opencv_partial_fingers`. Day la nhanh du phong, chinh xac kem MediaPipe nhung giup tranh bi `none`.

## 3. Nap ESP32-CAM

Mo sketch:

```text
D:\HOCTAP\PROJECT-CE320.Q21\esp32cam_pc_fuzzy\esp32cam_pc_fuzzy\esp32cam_pc_fuzzy.ino
```

Sua Wi-Fi trong dau file:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
```

Nap bang Arduino IDE cho board `AI Thinker ESP32-CAM`.

Sau khi boot, Serial Monitor `115200` se in:

```text
PC capture URL: http://<ESP_IP>/capture
Browser stream: http://<ESP_IP>:81/stream
READY
```

Endpoint ESP32:

```text
http://<ESP_IP>/capture
http://<ESP_IP>:81/stream
http://<ESP_IP>/control?brightness=-2..2&contrast=-2..2&saturation=-2..2&quality=10..30
```

Khi chi sua PC server hoac README, khong can nap lai ESP32.

## 4. Chay PC server

PowerShell, chay lan dau:

```powershell
cd "D:\HOCTAP\PROJECT-CE320.Q21\esp32cam_pc_fuzzy\pc_server"
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-mediapipe.txt
```

PowerShell, chay hang ngay:

```powershell
cd "D:\HOCTAP\PROJECT-CE320.Q21\esp32cam_pc_fuzzy\pc_server"
.\.venv\Scripts\Activate.ps1
python .\pc_server.py --camera-url "http://<ESP_IP>/capture" --web-port 5000 --timeout 6
```

Vi du voi IP gan day:

```powershell
python .\pc_server.py --camera-url "http://192.168.1.166/capture" --web-port 5000 --timeout 6
```

Mo web:

```text
http://127.0.0.1:5000
```

`<ESP_IP>` phai thay bang IP that ESP32-CAM in tren Serial Monitor. Khong go nguyen dau `< >`.

## 5. Web dieu khien

- Khung anh ben trai la anh PC da xu ly va ve overlay.
- Khung vang `hand ROI` la vung nen dua tay vao.
- Cac thanh `brightness`, `contrast`, `saturation`, `quality` goi ESP `/control` de chinh camera runtime.
- Nut `Snap Debug` hoac phim `S` de chup debug nhanh.
- Log duoi cung hien ly do nhan dien va loi camera neu co.

File debug nam trong:

```text
D:\HOCTAP\PROJECT-CE320.Q21\esp32cam_pc_fuzzy\data\debug_snaps
```

Moi lan snap se co:

```text
snap_...jpg             raw frame sach, dung de replay/debug
snap_..._annotated.jpg  anh co overlay neu co
snap_...json            gesture, fingers, bbox, reason, camera_config
```

## 6. Backend nhan dien

Backend uu tien theo thu tu:

```text
MediaPipe Tasks Hand Landmarker
-> fuzzy score tung ngon
-> OpenCV full fallback cho fist/open contour
-> OpenCV partial fingers cho 1-4 ngon bi cat long ban tay
```

Model MediaPipe:

```text
pc_server\models\hand_landmarker.task
```

Fuzzy MediaPipe tinh score cho:

```text
thumb, index, middle, ring, pinky
```

Neu tat ca score duoi nguong mo rong, ket qua la:

```text
gesture=fist
fingers=0
reason=mp_all_folded
```

Neu model/API MediaPipe thieu, server tu quay ve `opencv_fallback`.

## 7. Checklist test nhanh

Sau khi restart PC server:

1. Mo `http://127.0.0.1:5000`.
2. Dua tay vao khung `hand ROI`.
3. Test `fist`: nam tay gon, ket qua mong doi `gesture=fist`, `fingers=0`.
4. Test 1 ngon: dua 1 ngon ro, ket qua mong doi `gesture=open`, `fingers=1`.
5. Test 2-4 ngon: dua ngon ro va tach nhau, ket qua mong doi `fingers=2..4`.
6. Test 5 ngon: xoe ban tay ro, ket qua mong doi `fingers=5`.
7. Neu sai, bam `S` de chup debug va xem file `.json` trong `data\debug_snaps`.

## 8. Kich ban demo de giam sai

- Dat ban tay trong khung `hand ROI`.
- Nen dua camera xuong ban hoac tuong de mat/ao ra ngoai ROI.
- De ban tay cach camera khoang 25-45 cm.
- Khi test `fist`, nam tay gon, giu long ban tay/nam tay trong ROI.
- Khi test 1-4 ngon, dua cac ngon vao ROI; neu khong thay long ban tay thi ket qua co the la `opencv_partial_fingers`.
- Neu anh qua sang, giam `brightness`, giam den chieu thang vao camera, uu tien co bong/duong bien tay ro.
- Neu `/capture` timeout, kiem tra ESP32 va laptop cung Wi-Fi, dung dung IP moi tren Serial Monitor.

## 9. Loi thuong gap

Neu go nham PowerShell trong WSL se gap loi kieu `D:\... No such file or directory`. Khi do mo PowerShell that va chay lai lenh Windows.

Neu URL con dang:

```text
http://<ESP_IP>/capture
```

thi server se khong ket noi duoc. Phai thay bang IP that, vi du:

```text
http://192.168.1.166/capture
```
