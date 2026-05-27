# Note De Tai Fuzzy Logic

## De tai

ESP32-CAM + Fuzzy Logic + Edge Detection + Hand Sign

Muc tieu la xay dung he thong nhan dang ky hieu tay don gian bang camera ESP32-CAM. Logic mo duoc dung de phat hien bien anh ban tay, tu do ho tro tach vung tay va uoc luong so ngon tay.

## Input va output

Input:

- Anh tu camera ESP32-CAM
- Muc xam cua pixel trung tam va lan can trong cua so `3x3`
- Gia tri `delta = max(|Icenter - Ineighbor|)`

Output:

- Muc do bien `edge_strength`
- Mat do bien trong ROI
- Khung bao doi tuong ban tay
- So ngon tay uoc luong tu histogram cot bien

## 5 buoc logic mo

1. Bien ngon ngu
- Dau vao: `Delta_I`
- Dau ra: `Edge_strength`

2. Tap mo
- Dau vao: `Low`, `Medium`, `High`
- Dau ra: `No Edge`, `Weak`, `Strong`

3. Ham thuoc
- `trapLeft(15, 40)` cho `Low`
- `triangle(25, 70, 115)` cho `Medium`
- `trapRight(90, 170)` cho `High`

4. Luat mo
- Neu `Delta_I` thap thi khong phai bien
- Neu `Delta_I` trung binh thi bien yeu
- Neu `Delta_I` cao thi bien manh

5. Suy dien va giai mo
- Tinh muc thuoc cua `delta`
- Tong hop theo singleton:
  - `Low -> 0.10`
  - `Medium -> 0.55`
  - `High -> 1.00`
- So sanh voi nguong thich nghi `edge_dark -> edge_bright` de quyet dinh bien

## Ly do chon phien ban toi uu cho ESP32-CAM

- Khong dung fuzzy 8 luat day du tren moi pixel vi qua nang cho vi dieu khien
- Xu ly theo ROI de giam tai
- Dung line buffer thay vi cap phat anh xam toan khung
- Giu frame size `QQVGA` de dam bao toc do
- Dung `PIXFORMAT_GRAYSCALE` de bo qua buoc chuyen `RGB565 -> gray`
- Dung snapshot JPEG theo yeu cau qua Serial thay vi MJPEG stream lien tuc de tranh lam cham pipeline fuzzy
- Tat microSD logging mac dinh; PC dashboard luu CSV de tranh open/close file tren SD moi frame

## Huong toi uu hien tai

- Camera mac dinh: `brightness=-1`, `contrast=2`, `aec2=1`, `ae_level=-2`, `gainceiling=8x`
- Nguong bien thich nghi:
  - `edge_dark = 0.32`
  - `edge_bright = 0.24`
  - noi suy theo `mean_strength`
- Co the tune runtime tu web local bang lenh serial:
  - `SET quality=18 brightness=-1 contrast=2 edge_dark=0.32 edge_bright=0.24 hand_threshold=0.035 snapshot_ms=0`
- Mac dinh `snapshot_ms=0` de chi giu LOGCSV; khi can xem anh thi gui `SNAP`
- Histogram dem ngon tay chi lay vung tren cua bbox, lam muot 3 cot va bo cac peak yeu de giam nham nen/long ban tay

## Huong phat trien tiep

- Them contour extraction sau buoc edge map
- Thu them cac nguong khac nhau theo dieu kien anh sang
- Ket hop nhan dang cu chi tay on dinh hon thay vi chi dem dinh histogram
- Neu can stream muot, tach firmware stream rieng hoac thiet ke mot capture loop duy nhat co cache JPEG; khong de nhieu noi cung goi `esp_camera_fb_get()`
