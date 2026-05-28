"""
Dem so ngon tay bang OpenCV convexity defects.

Pipeline:
    mask binary (ROI hand) -> contour lon nhat -> convex hull
    -> convexity defects -> loc theo:
        - chieu sau defect (cao = khe giua 2 ngon)
        - goc tao boi (start, far, end) <= 90 do
    -> so ngon = so defect hop le + 1 (neu co it nhat 1 defect hop le)
    -> neu khong defect, kiem tra ti le area/convex_hull_area:
        - thap (~< 0.65) -> co the la ngon don le (pointing)
        - cao (> 0.85)   -> fist (nam dam)

Tham khao: tutorial chuan OpenCV cho hand gesture.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


# Tham so dem ngon — adaptive theo bbox + tunable runtime
# convexityDefects tra depth fixed-point (don vi: pixel * 256)
# -> nguong thuc te = MIN_DEFECT_DEPTH_RATIO * max(bbox_w, bbox_h) * 256
TUNABLE = {
    "min_defect_depth_ratio": 0.07,   # khe ngon >= 7% canh dai bbox -> hop le
    "max_defect_angle": 100.0,        # noi rong tu 95
    "min_tip_dist_ratio": 0.08,
    "solidity_fist_high": 0.92,
    "solidity_fist_med":  0.85,
}


@dataclass
class FingerResult:
    count: int                       # 0..5
    defects_used: int                # so defect duoc tinh
    defects_total: int               # tong defect truoc khi loc
    solidity: float                  # contour_area / hull_area (0..1)
    hull_area: float
    contour_area: float
    fingertips: list[tuple[int, int]]   # toa do dau ngon (xap xi)
    reason: str


def _angle_deg(a: tuple[int, int], b: tuple[int, int], c: tuple[int, int]) -> float:
    """Goc ABC (tai diem b) theo do."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    v1 = (ax - bx, ay - by)
    v2 = (cx - bx, cy - by)
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    n1 = (v1[0] ** 2 + v1[1] ** 2) ** 0.5
    n2 = (v2[0] ** 2 + v2[1] ** 2) ** 0.5
    if n1 == 0 or n2 == 0:
        return 180.0
    cos_t = max(-1.0, min(1.0, dot / (n1 * n2)))
    return float(np.degrees(np.arccos(cos_t)))


def count_fingers(mask: np.ndarray) -> FingerResult:
    """
    mask: uint8, 0/255, chi chua blob hand chinh.
    """
    if mask.size == 0 or mask.max() == 0:
        return FingerResult(0, 0, 0, 0.0, 0.0, 0.0, [],"empty_mask")

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return FingerResult(0, 0, 0, 0.0, 0.0, 0.0, [],"no_contour")

    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    if contour_area < 50:
        return FingerResult(0, 0, 0.0, 0.0, contour_area, [], "contour_too_small")

    hull_indices = cv2.convexHull(contour, returnPoints=False)
    hull_points = cv2.convexHull(contour, returnPoints=True)
    hull_area = float(cv2.contourArea(hull_points))
    solidity = contour_area / hull_area if hull_area > 0 else 0.0

    if hull_indices is None or len(hull_indices) < 3:
        return FingerResult(0, 0, solidity, hull_area, contour_area, [], "hull_too_small")

    try:
        defects = cv2.convexityDefects(contour, hull_indices)
    except cv2.error:
        defects = None

    fingertips: list[tuple[int, int]] = []
    valid_defects = 0
    defects_total = 0 if defects is None else defects.shape[0]

    if defects is not None:
        x, y, w, h = cv2.boundingRect(contour)
        bbox_long = max(w, h)
        # Adaptive nguong: 7% canh dai bbox, fixed-point *256
        min_depth_fp = int(bbox_long * TUNABLE["min_defect_depth_ratio"] * 256)
        max_angle = TUNABLE["max_defect_angle"]
        min_tip_dist = max(4, int(h * TUNABLE["min_tip_dist_ratio"]))

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            if d < min_depth_fp:
                continue
            start = tuple(int(v) for v in contour[s][0])
            end = tuple(int(v) for v in contour[e][0])
            far = tuple(int(v) for v in contour[f][0])
            angle = _angle_deg(start, far, end)
            if angle > max_angle:
                continue
            valid_defects += 1
            for tip in (start, end):
                if all((abs(tip[0] - t[0]) + abs(tip[1] - t[1])) > min_tip_dist for t in fingertips):
                    fingertips.append(tip)
                if len(fingertips) >= 5:
                    break

    if valid_defects >= 1:
        count = min(5, valid_defects + 1)
        reason = f"defects={valid_defects}/{defects_total}"
    else:
        if solidity >= TUNABLE["solidity_fist_high"]:
            count = 0
            reason = f"high_sol={solidity:.2f}_fist"
        elif solidity >= TUNABLE["solidity_fist_med"]:
            count = 0
            reason = f"med_sol={solidity:.2f}_fist"
        else:
            count = 1
            reason = f"low_sol={solidity:.2f}_pointing"

    return FingerResult(
        count=count,
        defects_used=valid_defects,
        defects_total=defects_total,
        solidity=round(solidity, 3),
        hull_area=hull_area,
        contour_area=contour_area,
        fingertips=fingertips[:5],
        reason=reason,
    )


def draw_finger_overlay(bgr: np.ndarray, mask: np.ndarray, fr: FingerResult) -> np.ndarray:
    """Ve hull + fingertips len anh BGR (in-place tren copy)."""
    out = bgr.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(contour)
        cv2.drawContours(out, [hull], -1, (255, 0, 255), 1)
        cv2.drawContours(out, [contour], -1, (0, 255, 0), 1)
    for (tx, ty) in fr.fingertips:
        cv2.circle(out, (tx, ty), 5, (0, 255, 255), -1)
        cv2.circle(out, (tx, ty), 7, (0, 0, 0), 1)
    cv2.putText(
        out, f"fingers={fr.count}", (5, out.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return out
