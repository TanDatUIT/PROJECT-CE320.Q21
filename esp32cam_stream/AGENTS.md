# AGENTS.md

Huong dan cho AI agents khi lam viec trong thu muc `ARDUINO/esp32cam_stream/`.

## Muc tieu hien tai

Day la workspace cho de tai `PROJECT FUZZY LOGIC` tren ESP32-CAM.

Co 2 sketch chinh:

- `esp32cam_stream.ino`
  - Ban web/Wi-Fi
  - Co `/`, `/stream`, `/capture`, `/metrics`
  - Camera dang de `PIXFORMAT_GRAYSCALE`
  - Stream uu tien xem live tren web
  - Fuzzy analysis chay dinh ky, khong phan tich moi frame
  - Serial da doi sang log theo cua so 10 giay

- `esp32cam_local.ino`
  - Ban local-only
  - Khong dung Wi-Fi, khong HTTP, khong stream
  - Chi log qua `Serial Monitor 115200`
  - Phu hop khi uu tien phan tich on dinh hon xem anh live

## Trang thai ky thuat

- Fuzzy edge detection dang chay tren anh grayscale `QQVGA`
- Dem ngon tay dang dung:
  - histogram theo cot
  - chi lay manh o vung tren cua bbox
  - smoothing histogram
  - peak gap / peak height / peak prominence
  - history 5 frame de lam muot `fingerCount`

- `stream` da duoc toi uu mot buoc:
  - giam delay trong HTTP stream
  - tach toc do stream khoi toc do analyze
  - analyze dinh ky `200 ms`

## Van de chua giai quyet hoan toan

- Stream van chua that su muot nhu firmware MJPEG goc
- Nhan dang `fingerCount` van co the dao dong trong dieu kien thuc te
- Truong hop body/canh tay dinh vao tay van co the lam sai bbox va so ngon
- `capture` hien tai nen xem nhu anh xac nhan theo dieu kien, khong phai ground truth tuyet doi

## Huong uu tien neu sua tiep

1. Neu uu tien stream:
   - toi uu tiep `esp32cam_stream.ino`
   - can nhac giam tan suat analyze hon nua
   - can nhac co `mode` rieng: `stream priority` vs `analyze priority`

2. Neu uu tien nhan dang:
   - uu tien `esp32cam_local.ino`
   - co the tune tiep cac nguong:
     - `kEdgeThreshold`
     - `kHandEdgeDensityThreshold`
     - peak gap / height / prominence

3. Neu muon AI/ML that su:
   - khong train tren Arduino/ESP32
   - thu data tren ESP32-CAM
   - train bang Python tren PC
   - chi deploy model nhe ve ESP32 de inference

## Quy uoc

- Comments va log dung tieng Viet khong dau de tranh loi font
- Khong xoa hai sketch hien co
- Neu tao them bien the moi, dat ten ro theo che do:
  - `*_stream.ino`
  - `*_local.ino`
  - `*_test.ino`
