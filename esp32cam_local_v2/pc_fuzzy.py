"""
Fuzzy logic phia PC cho ESP32-CAM hand gesture.
Quyet dinh: open vs fist vs none. Uu tien "open" khi ban tay vuot khung hinh.

Yeu cau:
    pip install pillow numpy
"""

from __future__ import annotations

import io
from dataclasses import dataclass, asdict

try:
    import numpy as np
    from PIL import Image
    AVAILABLE = True
    IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover
    AVAILABLE = False
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


@dataclass
class FuzzyResult:
    gesture: str           # "open" / "fist" / "none"
    confidence: float      # 0..1
    area_ratio: float
    bbox_extent: float     # blob_area / bbox_area
    aspect: float          # h / w
    finger_peaks: int      # so dinh tren histogram top-half
    touches_edge: bool     # bbox cham canh khung
    fg_is_dark: bool
    bbox: tuple[int, int, int, int]
    reason: str

    def to_dict(self) -> dict:
        d = asdict(self)
        d["bbox"] = list(self.bbox)
        return d


def _otsu_threshold(hist: np.ndarray, total: int) -> int:
    if total <= 0:
        return 128
    idx = np.arange(256, dtype=np.float64)
    sum_all = float((idx * hist).sum())
    w_bg, sum_bg = 0.0, 0.0
    best_t, best_var = 128, -1.0
    for t in range(256):
        w_bg += hist[t]
        if w_bg == 0:
            continue
        w_fg = total - w_bg
        if w_fg == 0:
            break
        sum_bg += t * hist[t]
        m_bg = sum_bg / w_bg
        m_fg = (sum_all - sum_bg) / w_fg
        var = w_bg * w_fg * (m_bg - m_fg) ** 2
        if var > best_var:
            best_var = var
            best_t = t
    return best_t


def _count_peaks(profile: np.ndarray, min_height: float, min_gap: int, min_prom: float) -> int:
    if profile.size == 0:
        return 0
    smoothed = np.convolve(profile, np.ones(3) / 3.0, mode="same")
    peaks = 0
    last_idx = -min_gap
    n = smoothed.size
    half = max(2, min_gap // 2)
    for i in range(half, n - half):
        v = smoothed[i]
        if v < min_height:
            continue
        window = smoothed[i - half:i + half + 1]
        if v < window.max() - 1e-6:
            continue
        valley = window.min()
        if (v - valley) < min_prom:
            continue
        if (i - last_idx) < min_gap:
            continue
        peaks += 1
        last_idx = i
    return min(peaks, 5)


def analyze_jpeg(image_bytes: bytes) -> FuzzyResult:
    """Phan tich JPG -> ket luan gesture. Co bias ve 'open' khi tay cham canh."""
    if not AVAILABLE:
        return FuzzyResult("none", 0.0, 0.0, 0.0, 0.0, 0, False, False, (-1, -1, -1, -1),
                           f"deps_missing:{IMPORT_ERROR}")

    try:
        img = Image.open(io.BytesIO(image_bytes)).convert("L")
    except Exception as exc:
        return FuzzyResult("none", 0.0, 0.0, 0.0, 0.0, 0, False, False, (-1, -1, -1, -1),
                           f"decode_failed:{exc}")

    # Resize ve <= 200px wide cho nhanh
    w0, h0 = img.size
    if w0 > 200:
        scale = 200.0 / w0
        img = img.resize((200, int(h0 * scale)), Image.BILINEAR)
    arr = np.asarray(img, dtype=np.uint8)
    h, w = arr.shape

    # Guard: anh qua toi / qua sang / qua phang -> khong the phan tich
    g_mean = float(arr.mean())
    g_std = float(arr.std())
    if g_mean < 25.0:
        return FuzzyResult("none", 0.9, 0.0, 0.0, 0.0, 0, False, True, (-1, -1, -1, -1),
                           f"image_too_dark(mean={g_mean:.1f})")
    if g_mean > 235.0:
        return FuzzyResult("none", 0.9, 0.0, 0.0, 0.0, 0, False, False, (-1, -1, -1, -1),
                           f"image_too_bright(mean={g_mean:.1f})")
    if g_std < 10.0:
        return FuzzyResult("none", 0.85, 0.0, 0.0, 0.0, 0, False, False, (-1, -1, -1, -1),
                           f"image_too_flat(std={g_std:.1f})")

    # ROI: bo viet 8% moi canh de tranh vignette
    mx, my = int(w * 0.08), int(h * 0.08)
    roi = arr[my:h - my, mx:w - mx]
    roi_h, roi_w = roi.shape
    total = roi.size

    hist, _ = np.histogram(roi, bins=256, range=(0, 256))
    thr = _otsu_threshold(hist, total)

    mask_dark = roi <= max(1, thr - 8)
    mask_bright = roi >= min(254, thr + 8)
    n_dark = int(mask_dark.sum())
    n_bright = int(mask_bright.sum())

    # Foreground = nhom co dien tich nho hon trong khoang hop ly
    fg_is_dark = n_dark <= n_bright
    fg = mask_dark if fg_is_dark else mask_bright
    fg_count = n_dark if fg_is_dark else n_bright
    area_ratio = fg_count / total

    if area_ratio < 0.06:
        return FuzzyResult("none", 0.85, area_ratio, 0.0, 0.0, 0, False, fg_is_dark,
                           (-1, -1, -1, -1), f"area_too_small({area_ratio:.3f})")
    if area_ratio > 0.85:
        return FuzzyResult("none", 0.7, area_ratio, 0.0, 0.0, 0, False, fg_is_dark,
                           (-1, -1, -1, -1), f"area_too_large({area_ratio:.3f})")

    ys, xs = np.where(fg)
    y0, y1 = int(ys.min()), int(ys.max())
    x0, x1 = int(xs.min()), int(xs.max())
    bw = x1 - x0 + 1
    bh = y1 - y0 + 1
    bbox_area = bw * bh
    extent = fg_count / max(1, bbox_area)
    aspect = bh / max(1, bw)

    # Cham canh ROI -> tay vuot khung
    edge_pad = max(2, int(min(roi_w, roi_h) * 0.04))
    touches_top = y0 <= edge_pad
    touches_bottom = y1 >= roi_h - 1 - edge_pad
    touches_left = x0 <= edge_pad
    touches_right = x1 >= roi_w - 1 - edge_pad
    touches_edge = touches_top or touches_bottom or touches_left or touches_right

    # Dem dinh tren nua tren bbox (ngon tay) — yeu cau prominence lon hon
    top_h = max(3, bh // 2)
    top_slice = fg[y0:y0 + top_h, x0:x1 + 1].astype(np.uint32)
    col_proj = top_slice.sum(axis=0)
    if col_proj.size > 0 and bw >= 16:
        peak_min_h = max(3.0, top_h * 0.30)
        peak_min_gap = max(5, bw // 8)
        peak_min_prom = max(2.5, top_h * 0.20)
        peaks = _count_peaks(col_proj.astype(np.float64), peak_min_h, peak_min_gap, peak_min_prom)
    else:
        peaks = 0

    # ============ Fuzzy rules (peaks-driven) ============
    # Logic: peaks la tin hieu chinh phan biet open vs fist.
    # Khi co tay (area du lon) — peaks >=2 -> open, peaks <=1 -> fist.
    open_score = 0.0
    fist_score = 0.0
    reasons: list[str] = []

    has_hand = area_ratio >= 0.10 and 0.30 <= aspect <= 3.0

    # R1: peaks la tin hieu chinh
    if peaks >= 3:
        open_score += 0.85 + 0.05 * min(peaks - 3, 2)
        reasons.append(f"peaks={peaks}")
    elif peaks == 2:
        open_score += 0.55
        reasons.append("peaks=2")
    elif peaks <= 1 and has_hand:
        fist_score += 0.55
        reasons.append(f"no_peaks(peaks={peaks})")

    # R2: extent + aspect modifier
    if extent < 0.50 and peaks >= 2:
        open_score += 0.20
        reasons.append(f"sparse_extent={extent:.2f}")
    elif extent > 0.75:
        # extent cao = blob nguyen khoi compact -> fist (ngon dam chum lai)
        fist_score += 0.25
        reasons.append(f"compact_extent={extent:.2f}")

    # R3: aspect modifier
    if 0.65 <= aspect <= 1.35 and has_hand and peaks <= 1:
        # gan vuong + khong co ngon -> nam dam dien hinh
        fist_score += 0.25
        reasons.append("square_nopeaks")
    elif aspect > 1.4 and peaks >= 2:
        # cao hon rong + co ngon -> tay xoe dung
        open_score += 0.20
        reasons.append("tall_with_peaks")

    # R4: bias open khi cham canh + area lon + co it nhat 1 dau hieu xoe
    if touches_edge and area_ratio >= 0.20 and peaks >= 2:
        open_score += 0.25
        reasons.append("edge_open_bias")

    # R4b: tay xoay ngang (aspect rat thap/cao) + area du -> bias open
    # User uu tien ket qua: tay xoay co spread lon -> coi nhu open palm
    if has_hand and area_ratio >= 0.15 and (aspect < 0.65 or aspect > 2.2):
        open_score += 0.45
        # tat fist_score tu R5 square_nopeaks neu co (aspect khong vao range do nen ok)
        reasons.append(f"rotated_open(asp={aspect:.2f})")

    # R5: area qua nho ma cham canh -> co the la mep tay -> none
    if area_ratio < 0.12 and touches_edge:
        reasons.append("small_at_edge")
        # giu nguyen score, se rot vao weak_evidence neu yeu

    # Final decision
    total_score = open_score + fist_score
    if not has_hand or total_score < 0.30:
        gesture = "none"
        conf = max(0.4, 1.0 - total_score)
        if not has_hand:
            reasons.append(f"no_hand(area={area_ratio:.2f},asp={aspect:.2f})")
        else:
            reasons.append("weak_evidence")
    elif open_score >= fist_score * 1.15:
        gesture = "open"
        conf = min(1.0, open_score / (total_score + 0.01))
    elif fist_score >= open_score * 1.15:
        gesture = "fist"
        conf = min(1.0, fist_score / (total_score + 0.01))
    else:
        gesture = "none"
        conf = 0.45
        reasons.append("tie")

    # Bbox tra ve theo toa do anh goc (sau resize)
    bbox = (x0 + mx, y0 + my, x1 + mx, y1 + my)

    return FuzzyResult(
        gesture=gesture,
        confidence=round(conf, 3),
        area_ratio=round(area_ratio, 4),
        bbox_extent=round(extent, 3),
        aspect=round(aspect, 3),
        finger_peaks=peaks,
        touches_edge=touches_edge,
        fg_is_dark=fg_is_dark,
        bbox=bbox,
        reason=";".join(reasons) if reasons else "default",
    )
