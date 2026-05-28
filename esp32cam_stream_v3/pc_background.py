"""
Background reference cho ROI selection.

Y tuong:
    - User nhan "Calibrate" -> dashboard yeu cau rut tay khoi khung
    - Worker chup N frame, lay median -> luu lam background ref (gray)
    - Voi moi frame moi: diff = |frame - bg|, threshold -> "motion/change mask"
    - Mask nay AND voi skin mask -> chi giu vung vua co skin, vua co thay doi

Trang thai background luu trong RAM (process-local), khong persist.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np


class BackgroundReference:
    def __init__(self):
        self._lock = threading.Lock()
        self._bg_gray: np.ndarray | None = None   # uint8 grayscale, shape (H, W)
        self._calibrated_ts: float = 0.0
        # Calibrate session state
        self._collecting: bool = False
        self._frames: list[np.ndarray] = []
        self._target_count: int = 0
        self._started_ts: float = 0.0

    # ---- Calibration session ----

    def start_calibration(self, target_count: int = 10) -> None:
        with self._lock:
            self._collecting = True
            self._frames = []
            self._target_count = max(3, int(target_count))
            self._started_ts = time.time()

    def feed_frame(self, gray: np.ndarray) -> bool:
        """
        Dua 1 frame vao session calibrate. Tra ve True neu DA xong (du frame).
        """
        with self._lock:
            if not self._collecting:
                return False
            self._frames.append(gray.copy())
            if len(self._frames) >= self._target_count:
                # Median qua N frame -> robust voi noise/lighting flicker
                stack = np.stack(self._frames, axis=0)
                self._bg_gray = np.median(stack, axis=0).astype(np.uint8)
                self._calibrated_ts = time.time()
                self._collecting = False
                self._frames = []
                return True
            return False

    def cancel_calibration(self) -> None:
        with self._lock:
            self._collecting = False
            self._frames = []

    # ---- Status ----

    def is_collecting(self) -> bool:
        with self._lock:
            return self._collecting

    def progress(self) -> tuple[int, int]:
        """(collected, target). (0, 0) neu khong dang calibrate."""
        with self._lock:
            if not self._collecting:
                return (0, 0)
            return (len(self._frames), self._target_count)

    def is_ready(self) -> bool:
        with self._lock:
            return self._bg_gray is not None

    def calibrated_age_s(self) -> float:
        with self._lock:
            if self._calibrated_ts == 0:
                return -1.0
            return time.time() - self._calibrated_ts

    # ---- Diff ----

    def diff_mask(self, gray: np.ndarray, threshold: int = 25) -> np.ndarray | None:
        """
        |frame - bg| > threshold -> uint8 0/255.
        Tra None neu chua calibrate hoac shape mismatch.
        """
        with self._lock:
            if self._bg_gray is None:
                return None
            if self._bg_gray.shape != gray.shape:
                return None
            bg = self._bg_gray

        # Tinh ngoai lock cho nhanh
        diff = cv2.absdiff(gray, bg)
        _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
        # Morphology: bot noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def reset(self) -> None:
        with self._lock:
            self._bg_gray = None
            self._calibrated_ts = 0.0
            self._collecting = False
            self._frames = []
