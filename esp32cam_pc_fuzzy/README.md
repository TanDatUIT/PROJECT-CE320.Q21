# ESP32-CAM PC FUZZY

Project moi tach rieng cho huong:

```text
ESP32-CAM -> stream/snapshot JPEG qua Wi-Fi
PC Python -> xu ly anh bang OpenCV/MediaPipe -> fuzzy none/fist/open
Web local -> hien thi anh, landmark/result, log
```

Muc tieu la de ESP32-CAM chi phu trach camera, con PC FUZZY xu ly nang hon.

## 1. Nap ESP32-CAM

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

Sau khi chay, Serial Monitor `115200` se in:

```text
PC capture URL: http://<ESP_IP>/capture
Browser stream: http://<ESP_IP>:81/stream
```

Sketch co endpoint camera control:

```text
http://<ESP_IP>/control?brightness=-2..2
```

## 2. Chay PC server

PowerShell:

```powershell
cd "D:\HOCTAP\PROJECT-CE320.Q21\esp32cam_pc_fuzzy\pc_server"
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-mediapipe.txt
python .\pc_server.py --camera-url "http://<ESP_IP>/capture" --web-port 5000
```

`<ESP_IP>` phai thay bang IP that cua ESP32-CAM, khong go nguyen dau `< >`.
Vi du:

```powershell
python .\pc_server.py --camera-url "http://192.168.1.25/capture" --web-port 5000
```

Mo web:

```text
http://127.0.0.1:5000
```

Neu `mediapipe` cai khong duoc tren Python hien tai, server van chay bang OpenCV fallback, nhung nen dung Python 3.10-3.12 de co MediaPipe tot hon.

## 3. Luong chay

- ESP32 endpoint `/capture`: tra ve 1 anh JPEG moi nhat cho PC doc dinh ky.
- ESP32 endpoint `:81/stream`: xem live tren browser. Stream tach port de khong khoa `/capture`.
- Web PC co thanh `brightness` de goi ESP `/control` va chinh sang camera runtime.
- PC server luu anh vao `data/captures/`.
- `pc_fuzzy_strong.py` uu tien MediaPipe hand landmarks; neu thieu MediaPipe thi fallback OpenCV contour.

## 4. Ghi chu thuc nghiem

- Nen uu tien nen don, camera co dinh, anh khong qua nguoc sang.
- Khi dung MediaPipe, co nguoi trong nen van co the phat hien tay nguoi do neu tay lo ro. Cach giam sai la gioi han ROI hoac dat vung tay truoc camera.
- Khi can chay nhanh hon, tang `--interval`, vi du `--interval 0.5`.

## 5. Kich ban demo de giam sai

- Dat ban tay trong khung `hand ROI` mau vang tren web PC.
- De mat va ao ra ngoai khung ROI neu co the; neu khong, dua camera xuong ngang ban/tuong.
- Nen de ban tay cach camera khoang 25-45 cm, xoe ro nam ngon, khong de ngon cai bi che.
- Anh qua sang thi giam den chieu thang vao camera; uu tien anh co bong tay ro hon la anh sang trang phang.
- Neu dung Python 3.13, server co the chi chay `opencv_fallback`. Khi do ket qua nen xem la `none/fist/open`; so `fingers=5` la quy uoc cho open palm, khong phai dem landmark that.
