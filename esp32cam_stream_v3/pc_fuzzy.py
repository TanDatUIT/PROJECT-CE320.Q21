"""
Fuzzy logic v3 — phan loai cu chi tay tu ROI + finger count.

3 class output:
    OPEN     : ban tay xoe, >= 3 ngon
    POINTING : 1-2 ngon (chi tro / V-sign)
    FIST     : nam dam, 0 ngon, solidity cao
    NONE     : khong xac dinh duoc (ROI fail)

Crisp output bo sung:
    finger_count: 0..5 (raw tu pc_finger_count)
    confidence  : 0..1 (defuzzify theo Mamdani max)

Inputs cho fuzzy (sau ROI selection):
    finger_count (0..5)
    solidity     (0..1)  - contour_area / hull_area
    area_ratio   (0..1)  - blob_area / frame_area
    aspect       (h/w)
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from pc_roi import ROIResult, extract_from_jpeg
from pc_finger_count import FingerResult, count_fingers
from pc_background import BackgroundReference


# ============ Membership functions ============

def _trap_left(x: float, a: float, b: float) -> float:
    if x <= a:
        return 1.0
    if x >= b:
        return 0.0
    return (b - x) / (b - a)


def _trap_right(x: float, a: float, b: float) -> float:
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    return (x - a) / (b - a)


def _triangle(x: float, a: float, b: float, c: float) -> float:
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / (b - a) if b > a else 1.0
    return (c - x) / (c - b) if c > b else 1.0


# ============ Result types ============

@dataclass
class FuzzyResult:
    gesture: str            # "open" / "pointing" / "fist" / "none"
    confidence: float
    finger_count: int
    solidity: float
    area_ratio: float
    aspect: float
    touches_edge: bool
    bbox: tuple[int, int, int, int]
    roi_found: bool
    reason: str

    def to_dict(self) -> dict:
        d = asdict(self)
        d["bbox"] = list(self.bbox)
        return d


# ============ Fuzzy decision ============

def classify(roi: ROIResult, fr: FingerResult) -> FuzzyResult:
    if not roi.found:
        return FuzzyResult(
            gesture="none", confidence=0.0, finger_count=0,
            solidity=0.0, area_ratio=roi.area_ratio,
            aspect=roi.aspect, touches_edge=roi.touches_edge,
            bbox=roi.bbox, roi_found=False,
            reason=f"roi:{roi.reason}",
        )

    n = fr.count
    sol = fr.solidity
    area = roi.area_ratio
    asp = roi.aspect

    # Membership cho "so ngon"
    mu_zero = _trap_left(n, 0, 1)              # n=0 -> 1, n>=1 -> 0
    mu_few  = _triangle(n, 0.5, 1.5, 2.8)      # 1-2 ngon
    mu_many = _trap_right(n, 2.2, 3.5)         # >=3 ngon

    # Membership cho solidity (do dac cua hand silhouette)
    mu_compact = _trap_right(sol, 0.78, 0.92)  # cao -> fist
    mu_loose   = _trap_left(sol, 0.55, 0.78)   # thap -> tay xoe

    # Membership cho area (tay du gan camera)
    mu_area_ok = _trap_right(area, 0.04, 0.10)

    # ============ Mamdani rules ============
    # Score moi class la max cua min(membership_i) qua cac rule
    open_scores = []
    point_scores = []
    fist_scores = []
    reasons: list[str] = []

    # R1: many fingers + loose silhouette -> OPEN
    s = min(mu_many, max(mu_loose, 0.4), mu_area_ok)
    if s > 0:
        open_scores.append(s)
        reasons.append(f"R1_open(n={n},sol={sol:.2f}):{s:.2f}")

    # R2: few fingers + medium solidity -> POINTING
    s = min(mu_few, mu_area_ok)
    if s > 0:
        point_scores.append(s)
        reasons.append(f"R2_point(n={n}):{s:.2f}")

    # R3: zero fingers + compact + area ok -> FIST
    s = min(mu_zero, mu_compact, mu_area_ok)
    if s > 0:
        fist_scores.append(s)
        reasons.append(f"R3_fist(n=0,sol={sol:.2f}):{s:.2f}")

    # R4: many fingers du solidity thap ma area lon (tay xoe gan) -> bias OPEN
    if area >= 0.18 and n >= 3:
        s = 0.85
        open_scores.append(s)
        reasons.append(f"R4_open_large_area:{s:.2f}")

    # R5: tay xoay ngang (aspect cuc) + area lon + co >=2 ngon -> OPEN
    if (asp < 0.55 or asp > 2.0) and area >= 0.15 and n >= 2:
        s = 0.70
        open_scores.append(s)
        reasons.append(f"R5_rotated_open(asp={asp:.2f}):{s:.2f}")

    # R6: cham canh + co ngon -> ban tay vuot khung, uu tien OPEN
    if roi.touches_edge and n >= 2:
        s = 0.55
        open_scores.append(s)
        reasons.append(f"R6_edge_open:{s:.2f}")

    # R7: contradiction guard — solidity rat cao nhung defects bao co ngon
    # -> tin solidity hon (fist)
    if sol >= 0.92 and n >= 1:
        s = 0.65
        fist_scores.append(s)
        reasons.append(f"R7_solidity_override:{s:.2f}")

    # ============ Defuzzify: max-score ============
    score_open  = max(open_scores)  if open_scores  else 0.0
    score_point = max(point_scores) if point_scores else 0.0
    score_fist  = max(fist_scores)  if fist_scores  else 0.0

    scores = {
        "open":     score_open,
        "pointing": score_point,
        "fist":     score_fist,
    }
    best = max(scores, key=scores.get)
    best_score = scores[best]
    total = sum(scores.values())

    if best_score < 0.30:
        gesture = "none"
        conf = round(max(0.3, 1.0 - total), 3)
        reasons.append(f"weak_evidence(max={best_score:.2f})")
    else:
        gesture = best
        # Confidence theo do troi cua best so voi tong
        conf = round(best_score / (total + 1e-6), 3)

    return FuzzyResult(
        gesture=gesture,
        confidence=conf,
        finger_count=n,
        solidity=sol,
        area_ratio=area,
        aspect=asp,
        touches_edge=roi.touches_edge,
        bbox=roi.bbox,
        roi_found=True,
        reason=";".join(reasons) if reasons else "default",
    )


def analyze_jpeg(
    image_bytes: bytes,
    bg: BackgroundReference | None = None,
) -> tuple[FuzzyResult, ROIResult, FingerResult]:
    """One-shot: JPEG -> (fuzzy, roi, fingers). Tat ca 3 cho dashboard."""
    roi = extract_from_jpeg(image_bytes, bg=bg)
    if roi.found:
        fr = count_fingers(roi.mask)
    else:
        fr = FingerResult(0, 0, 0, 0.0, 0.0, 0.0, [], "no_roi")
    result = classify(roi, fr)
    return result, roi, fr
