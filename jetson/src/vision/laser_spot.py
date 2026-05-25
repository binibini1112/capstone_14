"""Laser dot detector scoped to the tracked drone bbox."""
from collections import deque

import cv2
import numpy as np

import src.config as config


def _clamp_int(value, lo, hi):
    return max(lo, min(hi, int(round(float(value)))))


class LaserSpotDetector:
    """Detect a small bright laser blob near the current YOLO target bbox."""

    def __init__(self):
        self.enabled = bool(getattr(config, "LASER_SPOT_ENABLED", True))
        self.color = str(getattr(config, "LASER_SPOT_COLOR", "red")).strip().lower()
        self.margin_ratio = float(getattr(config, "LASER_SPOT_ROI_MARGIN_RATIO", 0.20))
        self.margin_px = int(getattr(config, "LASER_SPOT_ROI_MARGIN_PX", 24))
        self.min_area = max(1, int(getattr(config, "LASER_SPOT_MIN_AREA", 1)))
        self.max_area = max(self.min_area, int(getattr(config, "LASER_SPOT_MAX_AREA", 180)))
        self.min_value = int(getattr(config, "LASER_SPOT_MIN_VALUE", 145))
        self.min_sat = int(getattr(config, "LASER_SPOT_MIN_SAT", 70))
        self.red_dominance = int(getattr(config, "LASER_SPOT_RED_DOMINANCE", 28))
        self.hit_window = max(1, int(getattr(config, "LASER_SPOT_HIT_WINDOW", 5)))
        self.hit_min_frames = max(1, int(getattr(config, "LASER_SPOT_HIT_MIN_FRAMES", 3)))
        self.hit_inner_ratio = max(0.05, min(1.0, float(getattr(config, "LASER_SPOT_HIT_INNER_RATIO", 1.0))))
        self.hit_margin_px = max(0, int(getattr(config, "LASER_SPOT_HIT_MARGIN_PX", 0)))
        self._hits = deque(maxlen=self.hit_window)

    def reset(self):
        self._hits.clear()

    def update(self, frame, target_box, prev_frame=None, skip_eval=False):
        if skip_eval:
            return self._result(False)

        if not self.enabled or frame is None or not target_box:
            self.reset()
            return self._result(False)

        box = target_box.get("box", target_box)
        try:
            x1, y1, x2, y2 = [int(round(float(v))) for v in box]
        except (TypeError, ValueError):
            self.reset()
            return self._result(False)

        frame_h, frame_w = frame.shape[:2]
        if frame_w <= 0 or frame_h <= 0:
            self.reset()
            return self._result(False)

        x1 = _clamp_int(x1, 0, frame_w - 1)
        y1 = _clamp_int(y1, 0, frame_h - 1)
        x2 = _clamp_int(x2, x1 + 1, frame_w)
        y2 = _clamp_int(y2, y1 + 1, frame_h)
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        margin = max(self.margin_px, int(round(max(bw, bh) * self.margin_ratio)))
        rx1 = _clamp_int(x1 - margin, 0, frame_w - 1)
        ry1 = _clamp_int(y1 - margin, 0, frame_h - 1)
        rx2 = _clamp_int(x2 + margin, rx1 + 1, frame_w)
        ry2 = _clamp_int(y2 + margin, ry1 + 1, frame_h)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            self._hits.append(False)
            return self._result(False, roi=[rx1, ry1, rx2, ry2])

        if prev_frame is not None:
            prev_roi = prev_frame[ry1:ry2, rx1:rx2]
            if prev_roi.shape == roi.shape:
                mask = self._threshold_diff(roi, prev_roi)
            else:
                mask = self._threshold(roi)
        else:
            mask = self._threshold(roi)
        mask = cv2.medianBlur(mask, 3)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            blob_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [contour], -1, 255, -1)
            area = int(cv2.countNonZero(blob_mask[y:y + h, x:x + w]))
            if area < self.min_area or area > self.max_area:
                continue
            mean_bgr = cv2.mean(roi, mask=blob_mask)[:3]
            mean_hsv = cv2.mean(hsv_roi, mask=blob_mask)[:3]
            moments = cv2.moments(contour)
            if moments["m00"]:
                cx = int(round(moments["m10"] / moments["m00"]))
                cy = int(round(moments["m01"] / moments["m00"]))
            else:
                cx = x + w // 2
                cy = y + h // 2
            abs_x = rx1 + cx
            abs_y = ry1 + cy
            inside_bbox = x1 <= abs_x <= x2 and y1 <= abs_y <= y2
            inner_margin_x = int(round((1.0 - self.hit_inner_ratio) * bw * 0.5))
            inner_margin_y = int(round((1.0 - self.hit_inner_ratio) * bh * 0.5))
            hit_x1 = max(0, x1 + inner_margin_x - self.hit_margin_px)
            hit_y1 = max(0, y1 + inner_margin_y - self.hit_margin_px)
            hit_x2 = min(frame_w, x2 - inner_margin_x + self.hit_margin_px)
            hit_y2 = min(frame_h, y2 - inner_margin_y + self.hit_margin_px)
            inside_hit_bbox = hit_x1 <= abs_x <= hit_x2 and hit_y1 <= abs_y <= hit_y2
            compact = area / max(1.0, float(w * h))
            size_penalty = area / float(self.max_area)
            score = float(mean_hsv[2]) + float(mean_hsv[1]) * 0.25 + compact * 35.0 - size_penalty * 30.0
            if inside_hit_bbox:
                score += 20.0
            candidate = {
                "detected": True,
                "x": int(abs_x),
                "y": int(abs_y),
                "score": round(score, 3),
                "area": int(area),
                "inside_bbox": bool(inside_bbox),
                "inside_hit_bbox": bool(inside_hit_bbox),
                "hit_bbox": [hit_x1, hit_y1, hit_x2, hit_y2],
                "roi": [rx1, ry1, rx2, ry2],
                "bbox": [x1, y1, x2, y2],
                "mean_bgr": [round(float(v), 1) for v in mean_bgr],
            }
            if best is None or candidate["score"] > best["score"]:
                best = candidate

        if best is None:
            self._hits.append(False)
            return self._result(False, roi=[rx1, ry1, rx2, ry2], bbox=[x1, y1, x2, y2])

        self._hits.append(bool(best.get("inside_hit_bbox", False)))
        best.update(self._hit_state())
        return best

    def _threshold_diff(self, roi, prev_roi):
        diff = cv2.absdiff(roi, prev_roi)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        min_val = int(getattr(config, "LASER_SPOT_DIFF_MIN_VALUE", 50))
        _, mask = cv2.threshold(gray, min_val, 255, cv2.THRESH_BINARY)
        return mask

    def _threshold(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        b, g, r = cv2.split(roi)
        bright = v >= self.min_value
        saturated = s >= self.min_sat

        red_hue = (h <= 12) | (h >= 168)
        red_dom = r.astype(np.int16) >= (np.maximum(g, b).astype(np.int16) + self.red_dominance)
        red_mask = red_hue & saturated & bright & red_dom

        green_hue = (h >= 35) & (h <= 95)
        green_dom = g.astype(np.int16) >= (np.maximum(r, b).astype(np.int16) + self.red_dominance)
        green_mask = green_hue & saturated & bright & green_dom

        if self.color == "green":
            mask = green_mask
        elif self.color == "auto":
            mask = red_mask | green_mask
        else:
            mask = red_mask

        return (mask.astype(np.uint8) * 255)

    def _hit_state(self):
        hit_count = sum(1 for value in self._hits if value)
        return {
            "hit_detected": bool(hit_count >= self.hit_min_frames),
            "hit_count": int(hit_count),
            "hit_window": int(self.hit_window),
            "hit_min_frames": int(self.hit_min_frames),
        }

    def _result(self, detected, roi=None, bbox=None):
        hit_state = self._hit_state()
        return {
            "detected": bool(detected),
            "x": None,
            "y": None,
            "score": 0.0,
            "area": 0,
            "inside_bbox": False,
            "inside_hit_bbox": False,
            "hit_bbox": bbox,
            "roi": roi,
            "bbox": bbox,
            **hit_state,
        }
