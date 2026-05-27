# Project Fuzzy Logic

Thu muc `project_fuzzy_logic` la thu muc lam viec chinh cho de tai:

- ESP32-CAM
- Fuzzy Logic
- Edge Detection
- Hand Sign

## Tep chinh

- `project_fuzzy_logic.ino`: sketch Arduino chay tren AI Thinker ESP32-CAM
- `NOTE.md`: ghi chu de tai, input/output, 5 buoc logic mo, huong toi uu
- `local_dashboard.py`: doc serial realtime va mo dashboard web local
- `requirements.txt`: thu vien Python can cai

## Chuc nang hien tai

Sketch hien tai thuc hien pipeline nhe cho ESP32-CAM:

1. Chup frame o `QQVGA`
2. Doc truc tiep `GRAYSCALE` de giam tai so voi `RGB565`
3. Tinh `delta` trong cua so `3x3`
4. Suy dien logic mo voi 3 tap `Low / Medium / High`
5. Dung nguong bien thich nghi theo `mean_strength`
6. Tao chi so bien, mat do bien, khung bao va phan loai gesture don gian `none/fist/open`

Ket qua duoc in ra `Serial Monitor` o toc do `115200`.
Mac dinh sketch khong ghi microSD de giu frame time on dinh hon.
Sketch phat ra dong serial dang `LOGCSV,...` theo chu ky telemetry de Python doc realtime.
Python se luu CSV tren PC va co the nhan snapshot JPEG base64 khi gui lenh `SNAP` hoac khi dat `snapshot_ms > 0`.

## Cach nap

1. Mo thu muc `ARDUINO/project_fuzzy_logic` bang Arduino IDE
2. Chon board ESP32-CAM phu hop, thong dung la AI Thinker ESP32-CAM
3. Nap chuong trinh vao board
4. Mo `Serial Monitor` de xem:

```text
edge_density=... mean_strength=... hand=yes/no gesture=... code=... bbox=... time_ms=...
```

## File log

- Mac dinh log duoc luu tren PC vao `pc_fuzzy_log.csv`
- Neu muon log tren microSD, doi `kEnableSdLogging = true` trong sketch
- Khi bat microSD, sketch se tao file `/fuzzy_log.csv` tren the
- Moi dong log gom:
  - `time_ms`
  - `edge_density`
  - `mean_strength`
  - `hand_detected`
  - `finger_count`: ma gesture (`0=none`, `1=fist`, `5=open`)
  - `min_x`, `min_y`, `max_x`, `max_y`
  - `frame_time_ms`

Neu khoi tao the nho that bai, sketch van chay va chi in qua `Serial Monitor`.

## Python + Web local

Co the lien ket truc tiep voi Python de xem log realtime tren may tinh:

1. Cai thu vien:

```bash
pip install -r requirements.txt
```

2. Cam ESP32-CAM vao may tinh va xac dinh cong serial, vi du `COM5`

3. Chay dashboard local:

```bash
python local_dashboard.py --port COM5 --baudrate 115200 --web-port 5000
```

4. Mo trinh duyet tai:

```text
http://127.0.0.1:5000
```

Dashboard se:

- doc dong `LOGCSV,...` tu ESP32-CAM
- hien thong so realtime
- luu them file CSV local tren may tinh, mac dinh la `pc_fuzzy_log.csv`
- luu snapshot debug vao `pc_captures/`
- gui lenh tune runtime qua Serial: `quality`, `brightness`, `contrast`, `edge_dark`, `edge_bright`, `hand_threshold`, `snapshot_ms`
- co nut `Snapshot now` gui lenh `SNAP` de chup thu ngay, khong can doi chu ky

## Ghi chu ve hien camera thuc te

Ban cap nhat hien tai da ho tro:

- tuy chon log file tren microSD cua ESP32-CAM neu bat `kEnableSdLogging`
- log realtime qua serial cho Python
- web local de xem gia tri xu ly realtime
- snapshot anh debug theo yeu cau qua nut `Snapshot now` hoac lenh `SNAP`; co the dat `snapshot_ms > 0` neu can chup dinh ky

Ban cap nhat nay khong ghep MJPEG stream lien tuc vao firmware chinh. Ly do: stream lien tuc de lam camera bi tranh chap va lam giam toc do xu ly fuzzy. Huong hien tai la snapshot debug theo yeu cau + thanh tune runtime; neu can stream muot that su thi dung firmware stream rieng hoac thiet ke lai theo mot nguon capture duy nhat co cache.

## Tham so can tune

- `edge_dark`: nguong bien khi anh toi/tuong phan thap
- `edge_bright`: nguong bien khi anh sang/tuong phan cao
- `hand_threshold`: nguong xac dinh co ban tay trong ROI
- `brightness`, `contrast`, `quality`: chinh truc tiep tren web local
- `snapshot_ms`: chu ky gui anh debug qua Serial; mac dinh `0` de uu tien toc do, chi bat khi can debug anh
- `SNAP`: lenh serial chup snapshot thu cong
- `kLoopDelayMs`: do tre giua hai lan xu ly
- Output hien tai khong uu tien dem tung ngon tay nua; `finger_count` duoc giu lai de dashboard cu khong vo, nhung y nghia la ma gesture.

## Ghi chu

- Ma duoc dua ve tu huong `CODEX/Fuzzy logic/PROJECT/GPT_PLUS`
- Ban sketch cu trong thu muc Arduino bi loi cu phap, da duoc thay bang phien ban hoan chinh
- Huong toi uu uu tien chay on dinh tren thiet bi thay vi dung mo hinh xu ly anh nang
