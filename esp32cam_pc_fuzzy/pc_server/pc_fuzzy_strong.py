from __future__ import annotations

import time
from dataclasses import asdict, dataclass

try:
    import cv2
    import numpy as np
except Exception as exc:  # pragma: no cover
    cv2 = None
    np = None
    CV_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
else:
    CV_IMPORT_ERROR = ""

try:
    import mediapipe as mp
except Exception as exc:  # pragma: no cover
    mp = None
    MP_IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
else:
    MP_IMPORT_ERROR = ""


@dataclass
class FuzzyResult:
    gesture: str
    confidence: float
    fingers: int
    mode: str
    bbox: tuple[int, int, int, int]
    area_ratio: float
    reason: str
    frame_ms: float

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["bbox"] = list(self.bbox)
        return data


class PCFuzzyStrong:
    def __init__(self, prefer_mediapipe: bool = True) -> None:
        self.prefer_mediapipe = prefer_mediapipe
        self.hands = None
        self.backend_note = ""
        if prefer_mediapipe and mp is not None:
            try:
                solutions = getattr(mp, "solutions", None)
                hands_module = getattr(solutions, "hands", None) if solutions is not None else None
                if hands_module is None:
                    self.backend_note = "mediapipe_no_legacy_solutions_api"
                else:
                    self.hands = hands_module.Hands(
                        static_image_mode=False,
                        max_num_hands=1,
                        model_complexity=1,
                        min_detection_confidence=0.55,
                        min_tracking_confidence=0.50,
                    )
                    self.backend_note = "mediapipe_legacy_hands"
            except Exception as exc:
                self.hands = None
                self.backend_note = f"mediapipe_init_failed:{type(exc).__name__}:{exc}"
        elif prefer_mediapipe:
            self.backend_note = f"mediapipe_import_failed:{MP_IMPORT_ERROR}"
        else:
            self.backend_note = "mediapipe_disabled"

    def analyze_jpeg(self, image_bytes: bytes) -> tuple[FuzzyResult, bytes | None]:
        start = time.perf_counter()
        if cv2 is None or np is None:
            return self._none(start, f"opencv_missing:{CV_IMPORT_ERROR}"), None

        arr = np.frombuffer(image_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return self._none(start, "decode_failed"), None

        if self.hands is not None:
            result = self._analyze_mediapipe(frame, start)
        else:
            result = self._analyze_opencv_fallback(frame, start)

        annotated = self._draw_overlay(frame, result)
        ok, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return result, encoded.tobytes() if ok else None

    def _analyze_mediapipe(self, frame, start: float) -> FuzzyResult:
        h, w = frame.shape[:2]
        roi = self._roi_rect(w, h)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hand_result = self.hands.process(rgb)
        if not hand_result.multi_hand_landmarks:
            return self._none(start, "mp_no_hand")

        lm = hand_result.multi_hand_landmarks[0].landmark
        xs = [p.x for p in lm]
        ys = [p.y for p in lm]
        x0 = max(0, int(min(xs) * w))
        y0 = max(0, int(min(ys) * h))
        x1 = min(w - 1, int(max(xs) * w))
        y1 = min(h - 1, int(max(ys) * h))
        if not self._bbox_center_in_roi((x0, y0, x1, y1), roi):
            return self._none(start, f"mp_outside_roi bbox={x0},{y0},{x1},{y1}")

        area_ratio = ((x1 - x0 + 1) * (y1 - y0 + 1)) / float(max(1, w * h))

        # Finger state by tip-vs-PIP. Coordinate y grows downward.
        finger_pairs = [(8, 6), (12, 10), (16, 14), (20, 18)]
        extended = 0
        reasons: list[str] = []
        for tip, pip in finger_pairs:
            if lm[tip].y < lm[pip].y - 0.025:
                extended += 1

        thumb_tip = lm[4]
        thumb_ip = lm[3]
        thumb_mcp = lm[2]
        thumb_span = abs(thumb_tip.x - thumb_mcp.x)
        thumb_inner = abs(thumb_ip.x - thumb_mcp.x)
        if thumb_span > max(0.060, thumb_inner * 1.45):
            extended += 1

        if extended >= 3:
            gesture = "open"
            confidence = min(0.98, 0.68 + 0.06 * extended)
            reasons.append(f"mp_extended={extended}")
        elif extended <= 1:
            gesture = "fist"
            confidence = 0.82 if area_ratio >= 0.04 else 0.70
            reasons.append(f"mp_folded={extended}")
        else:
            gesture = "none"
            confidence = 0.52
            reasons.append(f"mp_uncertain={extended}")

        return FuzzyResult(
            gesture=gesture,
            confidence=round(confidence, 3),
            fingers=extended,
            mode="mediapipe",
            bbox=(x0, y0, x1, y1),
            area_ratio=round(area_ratio, 4),
            reason=";".join(reasons),
            frame_ms=round((time.perf_counter() - start) * 1000.0, 2),
        )

    def _analyze_opencv_fallback(self, frame, start: float) -> FuzzyResult:
        h, w = frame.shape[:2]
        roi = self._roi_rect(w, h)
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        lower = np.array([0, 133, 77], dtype=np.uint8)
        upper = np.array([255, 173, 127], dtype=np.uint8)
        mask = cv2.inRange(ycrcb, lower, upper)
        rx0, ry0, rx1, ry1 = roi
        roi_mask = np.zeros_like(mask)
        roi_mask[ry0:ry1 + 1, rx0:rx1 + 1] = mask[ry0:ry1 + 1, rx0:rx1 + 1]
        mask = roi_mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self._none(start, self._with_backend_note("cv_no_skin_contour"))

        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area <= 0.0:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            bbox = (x, y, x + bw - 1, y + bh - 1)
            if not self._bbox_center_in_roi(bbox, roi):
                continue
            if bw < max(24, int(w * 0.08)) or bh < max(24, int(h * 0.10)):
                continue
            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / max(1.0, hull_area)
            center_bonus = self._center_bonus(bbox, roi)
            # Face/ao o canh tren-duoi thuong compact; hand open co contour thua hon.
            score = area * (1.0 + center_bonus) * (1.15 if solidity < 0.78 else 0.80)
            candidates.append((score, contour))

        if not candidates:
            return self._none(start, self._with_backend_note("cv_no_roi_hand_candidate"))

        contour = max(candidates, key=lambda item: item[0])[1]
        area = float(cv2.contourArea(contour))
        area_ratio = area / float(max(1, w * h))
        if area_ratio < 0.025:
            return self._none(start, self._with_backend_note(f"cv_area_small({area_ratio:.3f})"))

        x, y, bw, bh = cv2.boundingRect(contour)
        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / max(1.0, hull_area)
        aspect = bh / max(1, bw)
        fingers, finger_reason = self._estimate_opencv_fingers(mask, contour, (x, y, bw, bh))
        one_finger_open = fingers == 1 and area_ratio > 0.030 and aspect >= 1.20 and solidity <= 0.88

        if area_ratio > 0.035 and (fingers >= 2 or solidity < 0.72 or one_finger_open):
            gesture = "open"
            confidence = min(0.88, max(0.66 if one_finger_open else 0.70, 1.15 - solidity))
            if fingers <= 0:
                fingers = 2
            reason = self._with_backend_note(
                f"cv_open_palm(sol={solidity:.2f},asp={aspect:.2f},{finger_reason},one={one_finger_open},roi=on)"
            )
        elif 0.55 <= aspect <= 1.85:
            gesture = "fist"
            confidence = 0.68
            fingers = 0
            reason = self._with_backend_note(f"cv_compact_solidity={solidity:.2f},asp={aspect:.2f},{finger_reason},roi=on")
        else:
            gesture = "none"
            confidence = 0.50
            fingers = 0
            reason = self._with_backend_note(f"cv_uncertain(sol={solidity:.2f},asp={aspect:.2f},{finger_reason},roi=on)")

        return FuzzyResult(
            gesture=gesture,
            confidence=round(confidence, 3),
            fingers=fingers,
            mode="opencv_fallback",
            bbox=(x, y, x + bw - 1, y + bh - 1),
            area_ratio=round(area_ratio, 4),
            reason=reason,
            frame_ms=round((time.perf_counter() - start) * 1000.0, 2),
        )

    def _draw_overlay(self, frame, result: FuzzyResult):
        h, w = frame.shape[:2]
        rx0, ry0, rx1, ry1 = self._roi_rect(w, h)
        cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), (255, 180, 60), 1)
        cv2.putText(frame, "hand ROI", (rx0 + 4, max(18, ry0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 180, 60), 1, cv2.LINE_AA)
        x0, y0, x1, y1 = result.bbox
        color = (80, 220, 80) if result.gesture != "none" else (80, 80, 240)
        if x0 >= 0 and y0 >= 0 and x1 > x0 and y1 > y0:
            cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
        label = f"{result.gesture} {result.confidence:.2f} {result.mode}"
        cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        cv2.putText(frame, result.reason[:70], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return frame

    def _none(self, start: float, reason: str) -> FuzzyResult:
        mode = "mediapipe" if self.hands is not None else "opencv_fallback"
        return FuzzyResult(
            gesture="none",
            confidence=0.9,
            fingers=0,
            mode=mode,
            bbox=(-1, -1, -1, -1),
            area_ratio=0.0,
            reason=reason,
            frame_ms=round((time.perf_counter() - start) * 1000.0, 2),
        )

    def _with_backend_note(self, reason: str) -> str:
        if self.hands is not None or not self.backend_note:
            return reason
        return f"{reason};{self.backend_note}"

    def _estimate_opencv_fingers(self, mask, contour, rect: tuple[int, int, int, int]) -> tuple[int, str]:
        x, y, bw, bh = rect
        if bw <= 0 or bh <= 0:
            return 0, "finger_est=bad_bbox"

        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)

        top_h = max(1, int(bh * 0.62))
        top = contour_mask[y:y + top_h, x:x + bw]
        proj_count = self._count_projection_fingers(top, bw, top_h)
        defect_count = self._count_defect_fingers(contour, rect)
        fingers = max(proj_count, defect_count)
        if fingers > 5:
            fingers = 5
        return fingers, f"finger_est={fingers},proj={proj_count},def={defect_count}"

    def _count_projection_fingers(self, top_mask, bw: int, top_h: int) -> int:
        if top_mask.size == 0 or bw < 12 or top_h < 8:
            return 0

        profile = (top_mask > 0).sum(axis=0).astype(np.float32)
        if profile.size < 5 or float(profile.max()) < 3.0:
            return 0

        smooth = np.convolve(profile, np.ones(5, dtype=np.float32) / 5.0, mode="same")
        threshold = max(3.0, float(smooth.max()) * 0.32)
        min_width = max(3, int(bw * 0.035))
        min_gap = max(2, int(bw * 0.025))

        runs: list[tuple[int, int, float]] = []
        start_idx: int | None = None
        peak = 0.0
        for idx, value in enumerate(smooth):
            if value >= threshold:
                if start_idx is None:
                    start_idx = idx
                    peak = float(value)
                else:
                    peak = max(peak, float(value))
            elif start_idx is not None:
                if idx - start_idx >= min_width:
                    runs.append((start_idx, idx - 1, peak))
                start_idx = None
                peak = 0.0
        if start_idx is not None and len(smooth) - start_idx >= min_width:
            runs.append((start_idx, len(smooth) - 1, peak))

        merged: list[tuple[int, int, float]] = []
        for run in runs:
            if merged and run[0] - merged[-1][1] <= min_gap:
                prev = merged[-1]
                merged[-1] = (prev[0], run[1], max(prev[2], run[2]))
            else:
                merged.append(run)

        return min(5, len(merged))

    def _count_defect_fingers(self, contour, rect: tuple[int, int, int, int]) -> int:
        x, y, bw, bh = rect
        if contour is None or len(contour) < 5:
            return 0

        hull_idx = cv2.convexHull(contour, returnPoints=False)
        if hull_idx is None or len(hull_idx) < 4:
            return 0

        defects = cv2.convexityDefects(contour, hull_idx)
        if defects is None:
            return 0

        valid_valleys = 0
        min_depth = max(6.0, min(bw, bh) * 0.045)
        bottom_limit = y + int(bh * 0.82)
        for idx in range(defects.shape[0]):
            start_i, end_i, far_i, depth_raw = defects[idx, 0]
            start = contour[start_i][0]
            end = contour[end_i][0]
            far = contour[far_i][0]
            depth = depth_raw / 256.0
            if depth < min_depth:
                continue
            if far[1] > bottom_limit:
                continue

            a = np.linalg.norm(end - start)
            b = np.linalg.norm(far - start)
            c = np.linalg.norm(end - far)
            if b <= 1e-6 or c <= 1e-6:
                continue
            angle = np.degrees(np.arccos(np.clip((b * b + c * c - a * a) / (2.0 * b * c), -1.0, 1.0)))
            if angle > 105.0:
                continue
            valid_valleys += 1

        return min(5, valid_valleys + 1) if valid_valleys > 0 else 0

    def _roi_rect(self, w: int, h: int) -> tuple[int, int, int, int]:
        return (
            int(w * 0.12),
            int(h * 0.10),
            int(w * 0.88),
            int(h * 0.86),
        )

    def _bbox_center_in_roi(self, bbox: tuple[int, int, int, int], roi: tuple[int, int, int, int]) -> bool:
        x0, y0, x1, y1 = bbox
        rx0, ry0, rx1, ry1 = roi
        cx = (x0 + x1) // 2
        cy = (y0 + y1) // 2
        return rx0 <= cx <= rx1 and ry0 <= cy <= ry1

    def _center_bonus(self, bbox: tuple[int, int, int, int], roi: tuple[int, int, int, int]) -> float:
        x0, y0, x1, y1 = bbox
        rx0, ry0, rx1, ry1 = roi
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        rcx = (rx0 + rx1) * 0.5
        rcy = (ry0 + ry1) * 0.5
        rdiag = max(1.0, ((rx1 - rx0) ** 2 + (ry1 - ry0) ** 2) ** 0.5)
        dist = ((cx - rcx) ** 2 + (cy - rcy) ** 2) ** 0.5
        return max(0.0, 1.0 - dist / rdiag)
