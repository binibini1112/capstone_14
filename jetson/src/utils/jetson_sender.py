import json
import math
import socket
import time


def _clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


def _as_number(value, default=None, cast=float):
    try:
        number = cast(value)
    except (TypeError, ValueError):
        return default
    try:
        if not math.isfinite(float(number)):
            return default
    except (TypeError, ValueError):
        return default
    return number


def normalize_ultraps(raw):
    """Normalize optional UltraPS direction input to the dashboard contract."""
    if raw is None:
        return None

    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore").strip()

    if isinstance(raw, str):
        raw = raw.strip()
        if not raw:
            return None
        if raw.startswith("{"):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError:
                return None
        else:
            parts = [part.strip() for part in raw.split(",")]
            try:
                data = {"motor_deg": float(parts[0])}
                if len(parts) > 1:
                    data["confidence"] = float(parts[1])
                raw = data
            except (TypeError, ValueError):
                return None

    if not isinstance(raw, dict):
        return None

    for key in ("motor_deg", "motor_direction_deg", "heading_deg", "direction_deg", "fan_deg"):
        value = raw.get(key)
        if value is None:
            continue
        try:
            motor_deg = float(value) % 360.0
        except (TypeError, ValueError):
            return None
        timestamp = _as_number(raw.get("timestamp"), default=time.time())
        result = {
            "motor_deg": motor_deg,
            "timestamp": float(timestamp),
        }
        confidence = _as_number(raw.get("confidence"))
        if confidence is not None:
            result["confidence"] = confidence
        if raw.get("source") is not None:
            result["source"] = str(raw["source"])
        return result

    return None


def _pan_tick_to_direction_deg(pan_tick, pan_min, pan_max, pan_dir=1):
    pan = _as_number(pan_tick)
    if pan is None:
        return None
    pan_min = float(pan_min)
    pan_max = float(pan_max)
    half_range = (pan_max - pan_min) / 2.0
    if half_range <= 0.0:
        return None
    center = (pan_max + pan_min) / 2.0
    signed_deg = ((pan - center) / half_range) * 90.0 * float(pan_dir or 1)
    if abs(signed_deg) < 0.1:
        signed_deg = 0.0
    return float((-signed_deg) % 360.0)


class JetsonTelemetrySender:
    """Best-effort UDP JSON sender for the Raspberry Pi telemetry contract."""

    def __init__(self, host, port=5005, rate_hz=20.0, enabled=True):
        self.host = str(host or "").strip()
        self.port = int(port)
        self.enabled = bool(enabled and self.host)
        requested_rate = _as_number(rate_hz, default=20.0)
        self.rate_hz = max(10.0, min(30.0, float(requested_rate)))
        self.min_interval = 1.0 / self.rate_hz
        self._last_send = 0.0
        self._last_error_log = 0.0
        self._sock = None
        if self.enabled:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def maybe_send(
        self,
        *,
        frame_id,
        frame_w,
        frame_h,
        state_value,
        fps,
        target_bbox=None,
        target_conf=0.0,
        motor_info=None,
        audio_info=None,
        laser_info=None,
        ultraps_raw=None,
        pan_min=0,
        pan_max=4095,
        pan_dir=1,
        force=False,
    ):
        if not self.enabled or self._sock is None:
            return False

        now = time.time()
        if not force and now - self._last_send < self.min_interval:
            return False

        payload = self._build_payload(
            timestamp=now,
            frame_id=frame_id,
            frame_w=frame_w,
            frame_h=frame_h,
            state_value=state_value,
            fps=fps,
            target_bbox=target_bbox,
            target_conf=target_conf,
            motor_info=motor_info,
            audio_info=audio_info,
            laser_info=laser_info,
            ultraps_raw=ultraps_raw,
            pan_min=pan_min,
            pan_max=pan_max,
            pan_dir=pan_dir,
        )
        data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        try:
            self._sock.sendto(data, (self.host, self.port))
            self._last_send = now
            return True
        except OSError as exc:
            if now - self._last_error_log >= 3.0:
                self._last_error_log = now
                print(f"[JETSON-SENDER] UDP send failed {self.host}:{self.port}: {exc}")
            return False

    def _build_payload(
        self,
        *,
        timestamp,
        frame_id,
        frame_w,
        frame_h,
        state_value,
        fps,
        target_bbox,
        target_conf,
        motor_info,
        audio_info,
        laser_info,
        ultraps_raw,
        pan_min,
        pan_max,
        pan_dir,
    ):
        frame_w = max(1, int(_as_number(frame_w, default=1, cast=int)))
        frame_h = max(1, int(_as_number(frame_h, default=1, cast=int)))
        target_found = target_bbox is not None
        payload = {
            "timestamp": float(timestamp),
            "target_found": bool(target_found),
            "state": str(state_value),
            "frame": {
                "width": frame_w,
                "height": frame_h,
            },
        }
        frame_id_value = _as_number(frame_id, cast=int)
        if frame_id_value is not None:
            payload["frame_id"] = int(frame_id_value)
        fps_value = _as_number(fps)
        if fps_value is not None:
            payload["fps"] = float(fps_value)

        if target_found:
            try:
                x1, y1, x2, y2 = [_as_number(v) for v in target_bbox]
            except (TypeError, ValueError):
                x1 = y1 = x2 = y2 = None
            if None in (x1, y1, x2, y2):
                payload["target_found"] = False
            else:
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                frame_cx = frame_w / 2.0
                frame_cy = frame_h / 2.0
                x_px = cx - frame_cx
                y_px = cy - frame_cy
                confidence = _as_number(target_conf)
                if confidence is not None:
                    payload["confidence"] = float(confidence)
                payload["bbox"] = {
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                    "cx": float(cx),
                    "cy": float(cy),
                    "w": float(x2 - x1),
                    "h": float(y2 - y1),
                }
                payload["error"] = {
                    "x_px": float(x_px),
                    "y_px": float(y_px),
                    "x_norm": float(_clamp(x_px / frame_cx)),
                    "y_norm": float(_clamp(y_px / frame_cy)),
                }

        motor = motor_info or {}
        present_pan = _as_number(motor.get("present_pan"), cast=int)
        present_tilt = _as_number(motor.get("present_tilt"), cast=int)
        present_timestamp = _as_number(motor.get("present_timestamp"))
        present_fresh = (
            present_timestamp is None
            or (
                present_timestamp > 0.0
                and (float(timestamp) - float(present_timestamp)) <= 2.0
            )
        )
        pan_cmd = present_pan if present_fresh else None
        if pan_cmd is None:
            pan_cmd = _as_number(motor.get("pan"), cast=int)
        tilt_cmd = present_tilt if present_fresh else None
        if tilt_cmd is None:
            tilt_cmd = _as_number(motor.get("tilt"), cast=int)
        pan_deg = _as_number(motor.get("pan_deg"))
        if pan_deg is None:
            pan_deg = _pan_tick_to_direction_deg(pan_cmd, pan_min, pan_max, pan_dir)
        tilt_deg = _as_number(motor.get("tilt_deg"))
        normalized_ultraps = normalize_ultraps(ultraps_raw)
        if pan_deg is not None or tilt_deg is not None or pan_cmd is not None or tilt_cmd is not None:
            ptz = {}
            if pan_deg is not None:
                ptz["pan_deg"] = float(pan_deg)
            if tilt_deg is not None:
                ptz["tilt_deg"] = float(tilt_deg)
            if tilt_cmd is not None:
                ptz["tilt_cmd"] = int(tilt_cmd)
            if pan_cmd is not None:
                ptz["pan_cmd"] = int(pan_cmd)
            payload["ptz"] = ptz

        if normalized_ultraps is not None:
            payload["ultra_ps"] = {
                "motor_deg": float(normalized_ultraps["motor_deg"]),
            }
        elif pan_deg is not None:
            payload["ultra_ps"] = {
                "motor_deg": float(pan_deg),
            }

        audio_source = audio_info or {}
        audio = {
            "enabled": bool(audio_source.get("enabled", False)),
        }
        audio_direction = _as_number(audio_source.get("direction_deg"))
        audio_confidence = _as_number(audio_source.get("confidence"), default=0.0)
        if audio_direction is not None:
            audio["direction_deg"] = float(audio_direction)
        audio["drone_detected"] = bool(audio_source.get("drone_detected", False))
        audio_sector = audio_source.get("sector")
        if audio_sector is not None:
            audio["sector"] = str(audio_sector)
        audio_sector_index = _as_number(audio_source.get("sector_index"), cast=int)
        if audio_sector_index is not None:
            audio["sector_index"] = int(audio_sector_index)
        audio_sector_count = _as_number(audio_source.get("sector_count"), cast=int)
        if audio_sector_count is not None:
            audio["sector_count"] = int(audio_sector_count)
        audio_target_motor_deg = _as_number(audio_source.get("target_motor_deg"))
        if audio_target_motor_deg is not None:
            audio["target_motor_deg"] = float(audio_target_motor_deg) % 360.0
        audio["confidence"] = float(_clamp(audio_confidence, 0.0, 1.0))
        audio["detected"] = bool(audio_source.get("detected", False))
        audio["fallback_active"] = bool(audio_source.get("fallback_active", False))
        audio["status"] = str(
            audio_source.get("status", "IDLE" if audio["enabled"] else "OFF")
        )
        payload["audio"] = audio

        laser = {
            "armed": bool((laser_info or {}).get("armed", False)),
            "fired": bool((laser_info or {}).get("fired", False)),
            "shot_count": int(_as_number((laser_info or {}).get("shot_count"), default=0, cast=int)),
            "hit_detected": bool((laser_info or {}).get("hit_detected", False)),
        }
        laser_dot = (laser_info or {}).get("laser_dot")
        if isinstance(laser_dot, dict):
            laser["dot"] = {
                "detected": bool(laser_dot.get("detected", False)),
                "x": _as_number(laser_dot.get("x"), cast=int),
                "y": _as_number(laser_dot.get("y"), cast=int),
                "score": _as_number(laser_dot.get("score"), default=0.0),
                "area": _as_number(laser_dot.get("area"), default=0, cast=int),
                "inside_bbox": bool(laser_dot.get("inside_bbox", False)),
                "hit_count": _as_number(laser_dot.get("hit_count"), default=0, cast=int),
                "hit_window": _as_number(laser_dot.get("hit_window"), default=5, cast=int),
            }
        else:
            laser["dot"] = {
                "detected": False,
                "x": None,
                "y": None,
                "score": 0.0,
                "area": 0,
                "inside_bbox": False,
                "hit_count": 0,
                "hit_window": 5,
            }
        laser_source = laser_info or {}
        laser["fire"] = {
            "active": bool(laser_source.get("fire_active", False)),
            "result": str(laser_source.get("fire_result", "idle")),
            "id": _as_number(laser_source.get("fire_id"), default=0, cast=int),
            "hit_frames": _as_number(laser_source.get("fire_hit_frames"), default=0, cast=int),
            "sample_frames": _as_number(laser_source.get("fire_sample_frames"), default=0, cast=int),
            "window_sec": _as_number(laser_source.get("fire_window_sec"), default=1.0),
        }
        payload["laser"] = laser

        return payload

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
