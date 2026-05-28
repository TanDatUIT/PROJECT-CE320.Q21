"""
ROI selection v2 — skin HSV + background subtraction.

Khac biet so voi v1 (Otsu thuan):
    - Decode JPEG -> RGB (giu mau) -> chuyen HSV
    - Skin mask: dai mau da chuan trong HSV
    - Neu da calibrate background -> AND voi diff mask (tay co di dong)
    - Connected components -> chon blob "main hand":
        + Bo qua blob phia tren 25% khung (loai mat — heuristic don gian)
        + Score = area_ratio / (1 + 2.0 * dist_to_center_norm)

Output ROIResult tuong thich pc_fuzzy.py / pc_finger_count.py / dashboard.
"""

from __future__ import annotations

import io
from dataclasses import dataclass

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from pc_background import BackgroundReference


TARGET_WIDTH = 240
EDGE_PAD_RATIO = 0.03

# Tunable runtime (dashboard co the sua qua /roi_tune)
TUNABLE = {
    "min_area_ratio": 0.025,
    "skin_h_max":     25,     # H tu 0..25 (mau da am)
    "skin_s_min":     15,     # SAT min — giam manh tu 30 -> 15 cho tay chay sang
    "skin_v_min":     40,
    "skin_s_max":     200,
    "skin_v_max":     255,
    "bg_diff_thr":    22,     # |frame - bg| nguong
    "face_band_y":    0.25,   # blob centroid o tren 25% khung -> nghi mat
}

# Neu skin_ratio < SKIN_FALLBACK_THR -> fallback dung BG-only
SKIN_FALLBACK_THR = 0.02


@dataclass
class ROIResult:
    found: bool
    mask: np.ndarray
    gray: np.ndarray
    bbox: tuple[int, int, int, int]
    centroid: tuple[float, float]
    area_ratio: float
    bbox_extent: float
    aspect: float
    touches_edge: bool
    fg_is_dark: bool                 # giu cho tuong thich cu, luon=False (vi dung skin)
    num_blobs_total: int
    reason: str

    # Cac field debug them
    used_background: bool = False
    skin_area_ratio: float = 0.0     # ti le pixel skin truoc khi chon blob


def _empty(gray: np.ndarray, reason: str) -> ROIResult:
    return ROIResult(
        found=False,
        mask=np.zeros(gray.shape if gray.ndim == 2 else gray.shape[:2], dtype=np.uint8),
        gray=gray if gray.ndim == 2 else cv2.cvtColor(gray, cv2.COLOR_RGB2GRAY),
        bbox=(-1, -1, -1, -1),
        centroid=(-1.0, -1.0),
        area_ratio=0.0,
        bbox_extent=0.0,
        aspect=0.0,
        touches_edge=False,
        fg_is_dark=False,
        num_blobs_total=0,
        reason=reason,
    )


def decode_jpeg_rgb(image_bytes: bytes) -> np.ndarray | None:
    """Decode JPEG -> RGB numpy, resize ve TARGET_WIDTH."""
    try:
        img = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    except Exception:
        return None
    w0, h0 = img.size
    if w0 != TARGET_WIDTH:
        scale = TARGET_WIDTH / float(w0)
        new_h = max(1, int(round(h0 * scale)))
        img = img.resize((TARGET_WIDTH, new_h), Image.BILINEAR)
    return np.asarray(img, dtype=np.uint8)


def build_skin_mask(rgb: np.ndarray) -> np.ndarray:
    """Lay mask pixel skin theo dai HSV. Tra uint8 0/255."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h_max = int(TUNABLE["skin_h_max"])
    s_min = int(TUNABLE["skin_s_min"])
    s_max = int(TUNABLE["skin_s_max"])
    v_min = int(TUNABLE["skin_v_min"])
    v_max = int(TUNABLE["skin_v_max"])
    m1 = cv2.inRange(hsv,
                     np.array([0, s_min, v_min], dtype=np.uint8),
                     np.array([h_max, s_max, v_max], dtype=np.uint8))
    # Dai do tham (wrap-around)
    m2 = cv2.inRange(hsv,
                     np.array([180 - h_max, s_min, v_min], dtype=np.uint8),
                     np.array([179, s_max, v_max], dtype=np.uint8))
    skin = cv2.bitwise_or(m1, m2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    skin = cv2.morphologyEx(skin, cv2.MORPH_OPEN, kernel, iterations=1)
    skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, kernel, iterations=2)
    return skin


def extract_main_hand(rgb: np.ndarray, bg: BackgroundReference | None = None) -> ROIResult:
    """
    Tu RGB -> tim blob hand chinh.
    Neu bg co reference -> AND voi diff mask de loai static (mat, ao dung yen).
    """
    if rgb.size == 0:
        return _empty(rgb, "empty_image")

    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    h, w = gray.shape
    total = float(h * w)

    # Sanity guards tren grayscale
    g_mean = float(gray.mean())
    if g_mean < 20.0:
        return _empty(gray, f"too_dark(mean={g_mean:.1f})")
    if g_mean > 240.0:
        return _empty(gray, f"too_bright(mean={g_mean:.1f})")

    min_area_ratio = float(TUNABLE["min_area_ratio"])
    bg_thr = int(TUNABLE["bg_diff_thr"])

    # 1) Skin mask
    skin_mask = build_skin_mask(rgb)
    skin_area = float((skin_mask > 0).sum())
    skin_ratio = skin_area / total

    # 2) Background diff (neu co)
    bg_mask = None
    bg_area = 0.0
    if bg is not None and bg.is_ready():
        bg_mask = bg.diff_mask(gray, threshold=bg_thr)
        if bg_mask is not None:
            bg_area = float((bg_mask > 0).sum())

    # Chien luoc chon work_mask:
    #   - Neu ca skin va bg deu lon -> AND (chinh xac nhat)
    #   - Neu skin qua nho (chay sang) nhung bg lon -> dung bg-only (fallback)
    #   - Neu khong co bg -> dung skin
    used_bg = False
    used_fallback = False
    if bg_mask is not None and skin_ratio >= SKIN_FALLBACK_THR:
        combined = cv2.bitwise_and(skin_mask, bg_mask)
        if (combined > 0).sum() >= 0.3 * min(skin_area, bg_area):
            work_mask = combined
            used_bg = True
        else:
            # AND ra qua it -> dung skin OR (bg AND mask gan tay) — uu tien bg
            work_mask = bg_mask if bg_area > skin_area else skin_mask
            used_bg = bg_area > skin_area
    elif bg_mask is not None and bg_area / total >= min_area_ratio:
        # Skin fail (cháy sáng) -> fallback BG-only
        work_mask = bg_mask
        used_bg = True
        used_fallback = True
    elif skin_ratio >= min_area_ratio:
        work_mask = skin_mask
    else:
        result = _empty(gray, f"no_skin_no_bg(skin={skin_ratio:.3f},bg={bg_area/total:.3f})")
        result.skin_area_ratio = round(skin_ratio, 4)
        return result

    # 3) Connected components -> chon blob hand
    labeled, num_blobs = ndimage.label(work_mask)
    if num_blobs == 0:
        result = _empty(gray, "no_blob_after_combine")
        result.skin_area_ratio = round(skin_ratio, 4)
        result.used_background = used_bg
        return result

    cy_img = h / 2.0
    cx_img = w / 2.0
    diag = (h * h + w * w) ** 0.5

    best_label = -1
    best_score = -1.0
    best_info = None
    face_band_y_px = h * float(TUNABLE["face_band_y"])

    for label_id in range(1, num_blobs + 1):
        ys, xs = np.where(labeled == label_id)
        area = float(xs.size)
        area_ratio = area / total
        if area_ratio < min_area_ratio:
            continue
        cx = float(xs.mean())
        cy = float(ys.mean())

        # Heuristic loai mat: blob ma centroid nam o 30% tren khung VA khong cham
        # canh duoi -> phat 20% score (van xet, neu khong co blob nao tot hon
        # van duoc chon)
        is_likely_face = cy < face_band_y_px and ys.max() < h * 0.7

        dist = ((cx - cx_img) ** 2 + (cy - cy_img) ** 2) ** 0.5
        dist_norm = dist / diag
        score = area_ratio / (1.0 + 2.0 * dist_norm)
        if is_likely_face:
            score *= 0.5     # phat manh

        if score > best_score:
            best_score = score
            best_label = label_id
            best_info = (xs, ys, area, area_ratio, cx, cy, is_likely_face)

    if best_label < 0:
        result = _empty(gray, f"no_blob_large_enough(num={num_blobs})")
        result.skin_area_ratio = round(skin_ratio, 4)
        result.used_background = used_bg
        return result

    xs, ys, area, area_ratio, cx, cy, is_likely_face = best_info
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    bw = x1 - x0 + 1
    bh = y1 - y0 + 1
    bbox_area = float(bw * bh)
    extent = area / bbox_area if bbox_area > 0 else 0.0
    aspect = bh / float(bw) if bw > 0 else 0.0

    edge_pad = max(2, int(min(w, h) * EDGE_PAD_RATIO))
    touches_edge = (
        x0 <= edge_pad
        or y0 <= edge_pad
        or x1 >= w - 1 - edge_pad
        or y1 >= h - 1 - edge_pad
    )

    final_mask = np.where(labeled == best_label, 255, 0).astype(np.uint8)

    reason = "ok"
    if used_fallback:
        reason = "ok_bg_fallback"
    if is_likely_face:
        reason += "_maybe_face"

    return ROIResult(
        found=True,
        mask=final_mask,
        gray=gray,
        bbox=(x0, y0, x1, y1),
        centroid=(cx, cy),
        area_ratio=round(area_ratio, 4),
        bbox_extent=round(extent, 3),
        aspect=round(aspect, 3),
        touches_edge=touches_edge,
        fg_is_dark=False,
        num_blobs_total=num_blobs,
        reason=reason,
        used_background=used_bg,
        skin_area_ratio=round(skin_ratio, 4),
    )


def extract_from_jpeg(
    image_bytes: bytes,
    bg: BackgroundReference | None = None,
) -> ROIResult:
    rgb = decode_jpeg_rgb(image_bytes)
    if rgb is None:
        return ROIResult(
            found=False,
            mask=np.zeros((1, 1), dtype=np.uint8),
            gray=np.zeros((1, 1), dtype=np.uint8),
            bbox=(-1, -1, -1, -1),
            centroid=(-1.0, -1.0),
            area_ratio=0.0,
            bbox_extent=0.0,
            aspect=0.0,
            touches_edge=False,
            fg_is_dark=False,
            num_blobs_total=0,
            reason="decode_failed",
        )
    return extract_main_hand(rgb, bg=bg)


def draw_debug_overlay(roi: ROIResult) -> np.ndarray:
    """Anh BGR co overlay: gray nen + mask xanh la + bbox + centroid + status BG."""
    bgr = cv2.cvtColor(roi.gray, cv2.COLOR_GRAY2BGR)
    if roi.found:
        overlay = bgr.copy()
        overlay[roi.mask > 0] = (0, 200, 0)
        bgr = cv2.addWeighted(overlay, 0.40, bgr, 0.60, 0)
        x0, y0, x1, y1 = roi.bbox
        color_bbox = (0, 255, 255)
        if "face" in roi.reason:
            color_bbox = (0, 100, 255)   # cam-do canh bao
        cv2.rectangle(bgr, (x0, y0), (x1, y1), color_bbox, 2)
        cx, cy = int(roi.centroid[0]), int(roi.centroid[1])
        cv2.circle(bgr, (cx, cy), 4, (0, 0, 255), -1)
    else:
        cv2.putText(
            bgr, f"NO ROI: {roi.reason}", (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA,
        )

    # Goc duoi: cho biet co dung BG khong
    tag = "BG:USED" if roi.used_background else ("BG:READY?" if roi.found else "BG:OFF")
    cv2.putText(
        bgr, tag, (5, bgr.shape[0] - 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
        (100, 220, 255) if roi.used_background else (160, 160, 160),
        1, cv2.LINE_AA,
    )
    return bgr
