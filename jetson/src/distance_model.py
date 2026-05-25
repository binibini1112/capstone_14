import json
import os
import statistics


class DistanceEstimator:
    """Small calibration hook for bbox-size based distance estimation.

    Calibration JSON can contain either:
      {"samples": [{"bbox_h": 42, "distance_mm": 3000}, ...]}
    or:
      {"table": [[42, 3000], ...]}

    Until real 1m/2m/3m/4m/5m data is collected, this returns a conservative
    default distance so the laser offset compensation path can be exercised.
    """

    def __init__(self, path=None, default_mm=3000, min_mm=500, max_mm=8000):
        self.path = path
        self.default_mm = int(default_mm)
        self.min_mm = int(min_mm)
        self.max_mm = int(max_mm)
        self.feature = os.getenv("DISTANCE_ESTIMATE_FEATURE", "bbox_w").strip().lower()
        self.samples = self._load_samples(path)

    def _load_samples(self, path):
        if not path or not os.path.exists(path):
            return []
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        raw = data.get("samples", data.get("table", []))
        grouped = {}
        for item in raw:
            if isinstance(item, dict):
                bbox_w = item.get("bbox_w", item.get("width_px"))
                bbox_h = item.get("bbox_h", item.get("height_px"))
                distance = item.get("distance_mm")
            else:
                bbox_h, distance = item[0], item[1]
                bbox_w = None
            feature_value = bbox_w if self.feature in ("bbox_w", "w", "width") and bbox_w is not None else bbox_h
            feature_value = float(feature_value)
            distance = float(distance)
            if feature_value > 0 and distance > 0:
                grouped.setdefault(int(round(distance)), []).append(feature_value)
        samples = [
            (float(statistics.median(values)), float(distance))
            for distance, values in grouped.items()
            if values
        ]
        samples.sort(key=lambda pair: pair[0])
        return samples

    def estimate(self, bbox_width, bbox_height):
        if self.feature in ("bbox_w", "w", "width"):
            size = float(max(1, int(bbox_width or bbox_height or 0)))
        else:
            size = float(max(1, int(bbox_height or bbox_width or 0)))
        if len(self.samples) < 2:
            return self._clamp(self.default_mm)

        samples = self.samples
        if size <= samples[0][0]:
            return self._clamp(samples[0][1])
        if size >= samples[-1][0]:
            return self._clamp(samples[-1][1])
        for (h0, d0), (h1, d1) in zip(samples, samples[1:]):
            if h0 <= size <= h1:
                ratio = (size - h0) / max(1e-6, h1 - h0)
                return self._clamp(d0 + (d1 - d0) * ratio)
        return self._clamp(self.default_mm)

    def _clamp(self, value):
        return int(max(self.min_mm, min(self.max_mm, round(float(value)))))


class LaserTickEstimator:
    """Interpolate laser C-motor base tick directly from detected bbox height."""

    def __init__(self, path=None, min_tick=0, max_tick=4095):
        self.path = path
        self.min_tick = int(min_tick)
        self.max_tick = int(max_tick)
        self.samples = self._load_samples(path)

    def _load_samples(self, path):
        if not path or not os.path.exists(path):
            return []
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        raw = data.get("samples", data.get("table", []))
        samples = []
        for item in raw:
            if isinstance(item, dict):
                bbox_h = item.get("bbox_h", item.get("height_px"))
                tick = item.get("laser_tilt_tick", item.get("laser_tick"))
            else:
                bbox_h, tick = item[0], item[1]
            if bbox_h is None or tick is None:
                continue
            bbox_h = float(bbox_h)
            tick = float(tick)
            if bbox_h > 0 and tick >= 0:
                samples.append((bbox_h, tick))
        samples.sort(key=lambda pair: pair[0])
        return samples

    def estimate(self, bbox_width, bbox_height):
        del bbox_width
        h = float(max(1, int(bbox_height or 0)))
        if len(self.samples) < 2:
            return None

        samples = self.samples
        if h <= samples[0][0]:
            return self._clamp(samples[0][1])
        if h >= samples[-1][0]:
            return self._clamp(samples[-1][1])
        for (h0, t0), (h1, t1) in zip(samples, samples[1:]):
            if h0 <= h <= h1:
                ratio = (h - h0) / max(1e-6, h1 - h0)
                return self._clamp(t0 + (t1 - t0) * ratio)
        return None

    def _clamp(self, value):
        return int(max(self.min_tick, min(self.max_tick, round(float(value)))))
