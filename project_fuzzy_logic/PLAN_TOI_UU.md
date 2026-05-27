# Plan toi uu ESP32-CAM Fuzzy Logic

## Muc tieu gan nhat

Uu tien ket qua dung va on dinh tren board ESP32-CAM truoc. Web local chi la cong cu debug/tune, khong phai mot phan bat buoc cua pipeline nhan dang.

## Kien truc nen giu

1. ESP32-CAM chay fuzzy edge detection truc tiep tren board.
2. Firmware dung `GRAYSCALE + QQVGA + ROI + line buffer` de giu tai nhe.
3. Serial chi xuat `LOGCSV` theo chu ky telemetry, mac dinh 200 ms.
4. Snapshot JPEG qua Serial mac dinh tat; chi chup khi gui lenh `SNAP` hoac dat `snapshot_ms > 0`.
5. Web local chi doc Serial, luu CSV, hien snapshot debug va gui lenh tune.

Neu dang chay nhanh bang nhanh local `ARDUINO/esp32cam_local`, cung giu cung nguyen tac: `capture_ms=0` mac dinh, chi chup khi bam `Snapshot now` hoac gui `SNAP`.

## Co nen bo web khong?

Co the bo web khi can toi uu toc do nhung khong nen xoa han. Neu chay board doc lap, dat:

```text
SET snapshot_ms=0
```

Sau do chi xem Serial Monitor hoac CSV tu Python. Luc nay web khong lam nang firmware, vi firmware van chi xu ly camera va gui telemetry ngan.

Web local van huu ich khi can:

- nhin anh debug bang `SNAP`
- tune `brightness`, `contrast`, `edge_dark`, `edge_bright`, `hand_threshold`
- luu log tren PC de so sanh nhieu lan chay

## Co the stream khong?

Co, nhung khong nen ghep MJPEG stream lien tuc vao firmware fuzzy hien tai. Ly do la ESP32-CAM chi co mot nguon camera; stream lien tuc va fuzzy cung tranh frame, dong thoi JPEG encode lien tuc lam cham loop xu ly.

Huong dung neu can hinh anh:

1. Debug nhanh: dung snapshot `SNAP`.
2. Can stream muot: dung firmware stream rieng de can khung hinh va anh sang.
3. Can vua fuzzy vua xem hinh: thiet ke lai theo mot capture loop duy nhat, co cache JPEG thua thoi gian, khong de nhieu noi goi `esp_camera_fb_get()`.

## Thu tu chay de nhung thanh cong

1. Nap `ARDUINO/project_fuzzy_logic/project_fuzzy_logic.ino`.
2. Mo Serial Monitor 115200 va bam RST/EN neu chua co log.
3. Xac nhan co dong:

```text
LOGCSV_HEADER,...
CONFIG_STATUS,...
LOGCSV,...
```

4. Dat che do chay nhe:

```text
SET snapshot_ms=0 brightness=-1 contrast=2 edge_dark=0.32 edge_bright=0.24 hand_threshold=0.035
```

5. Dua tay vao ROI giua camera, theo doi `hand`, `gesture`, `bbox`.
6. Neu can xem anh, gui:

```text
SNAP
```

7. Chi khi can tune nhieu moi chay `local_dashboard.py`.

## Huong tune ket qua

- Anh qua toi: thu `brightness=0`, giu `contrast=2`.
- Anh qua nhieu bien nen: tang `hand_threshold` len `0.045` hoac tang `edge_dark`.
- Khong bat duoc tay: giam `hand_threshold` ve `0.025` den `0.030`.
- Neu `fist/open` nhay: giu tay trong ROI, tang anh sang deu, roi tune `hand_threshold`, `edge_dark`, `edge_bright`.

## Co can them ESP32 thu hai + LCD khong?

Chua can cho giai doan hien tai. Mot ESP32-CAM + web local la du de debug va tune. Them ESP32 thu hai chi nen lam khi:

- thuat toan tren ESP32-CAM da on dinh
- can demo khong dung laptop
- chi can hien ket qua gon nhu `hand/gesture/status`, khong can hien video

Neu them LCD, nen de ESP32-CAM gui ket qua ngan qua UART/I2C/ESP-NOW, khong gui anh.
