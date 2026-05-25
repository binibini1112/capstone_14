"""Thread-safe state store for dashboard telemetry."""

from __future__ import annotations

import time
import math
from collections import deque
from dataclasses import replace
from threading import RLock
from typing import Any

from .telemetry_model import (
    DashboardSnapshot,
    EventLogEntry,
    JetsonTrackingData,
    JETSON_DISCONNECTED_SEC,
    JETSON_TIMEOUT_SEC,
    TelloStatus,
)


class TelemetryStore:
    """Keeps latest state and bounded history for dashboard clients."""

    def __init__(self, max_samples: int = 300, max_events: int = 200) -> None:
        self._lock = RLock()
        self._max_samples = max_samples
        self._history: deque[dict[str, Any]] = deque(maxlen=max_samples)
        self._events: deque[EventLogEntry] = deque(maxlen=max_events)
        self._command_times: deque[float] = deque(maxlen=200)
        self._tracking_receive_times: deque[float] = deque(maxlen=200)
        self._latest_tracking: JetsonTrackingData | None = None
        self._last_received_monotonic: float | None = None
        self._tello = TelloStatus()
        self._last_target_found: bool | None = None
        self._last_state: str | None = None
        self._last_jetson_status = "DISCONNECTED"
        self._last_audio_active = False
        self._last_laser_armed = False
        self._last_laser_fired = False
        self._last_laser_hit = False
        self._last_reported_shot_count: int | None = None
        self._hit_count = 0
        self._shot_count = 0

    def update_tracking(self, data: JetsonTrackingData, now: float | None = None) -> None:
        monotonic_now = time.monotonic() if now is None else now
        with self._lock:
            self._tracking_receive_times.append(monotonic_now)
            self._trim_tracking_receive_times(monotonic_now)
            self._latest_tracking = data
            self._last_received_monotonic = monotonic_now
            self._history.append(self._sample_from_tracking(data, monotonic_now))
            self._record_tracking_events(data)

    def update_tello(
        self,
        *,
        connected: bool | None = None,
        airborne: bool | None = None,
        battery: int | None = None,
        speed: int | None = None,
        last_command: str | None = None,
    ) -> None:
        with self._lock:
            self._tello = replace(
                self._tello,
                connected=self._tello.connected if connected is None else connected,
                airborne=self._tello.airborne if airborne is None else airborne,
                battery=self._tello.battery if battery is None else battery,
                speed=self._tello.speed if speed is None else speed,
                last_command=self._tello.last_command if last_command is None else last_command,
            )

    def record_command(self, command: str, now: float | None = None) -> None:
        monotonic_now = time.monotonic() if now is None else now
        with self._lock:
            self._command_times.append(monotonic_now)
            self._trim_command_times(monotonic_now)
            self._tello = replace(
                self._tello,
                last_command=command,
                command_rate=float(len(self._command_times)),
            )

    def add_event(self, message: str, level: str = "INFO", now: float | None = None) -> None:
        timestamp = time.time() if now is None else now
        with self._lock:
            self._events.append(EventLogEntry(timestamp=timestamp, level=level, message=message))

    def snapshot(self, now: float | None = None) -> DashboardSnapshot:
        monotonic_now = time.monotonic() if now is None else now
        with self._lock:
            command_rate = self._command_rate(monotonic_now)
            tello = replace(self._tello, command_rate=command_rate)
            age = (
                None
                if self._last_received_monotonic is None
                else max(0.0, monotonic_now - self._last_received_monotonic)
            )
            jetson_status = self._jetson_status(age)
            self._record_jetson_status_event(jetson_status, age)
            return DashboardSnapshot(
                timestamp=time.time(),
                tello=tello,
                jetson_status=jetson_status,
                last_received_age=age,
                tracking=self._latest_tracking,
                history=list(self._history)[-self._max_samples :],
                events=list(self._events),
                hit_count=self._hit_count,
                shot_count=self._shot_count,
            )

    def _record_tracking_events(self, data: JetsonTrackingData) -> None:
        event_timestamp = time.time()
        if self._last_target_found is None:
            self._last_target_found = data.target_found
        elif data.target_found and not self._last_target_found:
            confidence = "" if data.confidence is None else f" (conf {data.confidence:.2f})"
            self._events.append(EventLogEntry(event_timestamp, "INFO", f"VISION TARGET ACQUIRED{confidence}"))
            self._last_target_found = True
        elif not data.target_found and self._last_target_found:
            self._events.append(EventLogEntry(event_timestamp, "WARN", "TARGET LOST - Searching..."))
            self._last_target_found = False

        if data.state != self._last_state:
            self._events.append(EventLogEntry(event_timestamp, "INFO", f"STATE {data.state}"))
            self._last_state = data.state

        audio = data.audio
        audio_active = bool(audio and audio.confidence is not None and audio.confidence >= 0.5)
        if audio_active and not self._last_audio_active:
            bearing = "-" if audio.direction_deg is None else f"{audio.direction_deg:03.0f} deg"
            confidence = "-" if audio.confidence is None else f"{audio.confidence:.2f}"
            self._events.append(
                EventLogEntry(event_timestamp, "INFO", f"AUDIO DETECTED - Bearing {bearing} (conf {confidence})")
            )
        self._last_audio_active = audio_active

        laser = data.laser
        laser_armed = self._bool_flag(laser.armed if laser else None)
        laser_fired = self._bool_flag(laser.fired if laser else None)
        laser_hit = self._bool_flag(laser.hit_detected if laser else None)
        incoming_shot_count = self._int_value(laser.shot_count if laser else None)
        shot_count_delta = self._shot_count_delta(incoming_shot_count)
        shot_event = laser_fired and not self._last_laser_fired
        if shot_count_delta > 0:
            self._shot_count += shot_count_delta
            self._events.append(EventLogEntry(event_timestamp, "INFO", "LASER FIRED"))
        elif shot_event:
            self._shot_count += 1
            self._events.append(EventLogEntry(event_timestamp, "INFO", "LASER FIRED"))
        if laser_hit and not self._last_laser_hit:
            self._hit_count += 1
            self._events.append(EventLogEntry(event_timestamp, "INFO", f"HIT CONFIRMED (#{self._hit_count})"))
        self._last_laser_armed = laser_armed
        self._last_laser_fired = laser_fired
        self._last_laser_hit = laser_hit

    def _record_jetson_status_event(self, status: str, age: float | None) -> None:
        if status == self._last_jetson_status:
            return
        previous = self._last_jetson_status
        self._last_jetson_status = status
        if age is None and previous == "DISCONNECTED":
            return
        if status == "CONNECTED":
            self._events.append(EventLogEntry(time.time(), "INFO", "Jetson data connected"))
        elif status == "STALE":
            self._events.append(EventLogEntry(time.time(), "WARN", "Jetson data timeout"))
        elif status == "DISCONNECTED":
            self._events.append(EventLogEntry(time.time(), "ERROR", "Jetson data disconnected"))

    def _sample_from_tracking(self, data: JetsonTrackingData, now: float) -> dict[str, Any]:
        error = data.error
        audio = data.audio
        laser = data.laser
        tracking_error_px = self._tracking_error_px(error)
        return {
            "timestamp": data.timestamp,
            "received_at": now,
            "fps": data.fps,
            "confidence": data.confidence,
            "x_px": error.x_px if error else None,
            "y_px": error.y_px if error else None,
            "tracking_error_px": tracking_error_px,
            "latency_ms": self._latency_ms(data.timestamp),
            "battery": self._tello.battery,
            "telemetry_rate_hz": self._telemetry_rate(now),
            "command_rate": self._command_rate(now),
            "target_found": 1 if data.target_found else 0,
            "audio_confidence": audio.confidence if audio and audio.fallback_active else None,
            "audio_fallback_active": 1 if audio and audio.fallback_active else 0,
            "laser_hit": 1 if laser and laser.hit_detected else 0,
            "hit_count": self._hit_count,
            "shot_count": self._shot_count,
        }

    def _trim_command_times(self, now: float) -> None:
        while self._command_times and now - self._command_times[0] > 1.0:
            self._command_times.popleft()

    def _command_rate(self, now: float) -> float:
        self._trim_command_times(now)
        return float(len(self._command_times))

    def _trim_tracking_receive_times(self, now: float) -> None:
        while self._tracking_receive_times and now - self._tracking_receive_times[0] > 1.0:
            self._tracking_receive_times.popleft()

    def _telemetry_rate(self, now: float) -> float:
        self._trim_tracking_receive_times(now)
        return float(len(self._tracking_receive_times))

    @staticmethod
    def _jetson_status(age: float | None) -> str:
        if age is None:
            return "DISCONNECTED"
        if age <= JETSON_TIMEOUT_SEC:
            return "CONNECTED"
        if age <= JETSON_DISCONNECTED_SEC:
            return "STALE"
        return "DISCONNECTED"

    @staticmethod
    def _tracking_error_px(error: Any) -> float | None:
        if error is None or error.x_px is None or error.y_px is None:
            return None
        return math.hypot(float(error.x_px), float(error.y_px))

    @staticmethod
    def _latency_ms(timestamp: float) -> float | None:
        latency = time.time() - timestamp
        if latency < 0 or latency > 60:
            return None
        return latency * 1000.0

    @staticmethod
    def _bool_flag(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"true", "1", "yes", "on"}
        if isinstance(value, (int, float)):
            return value != 0
        return False

    @staticmethod
    def _int_value(value: Any) -> int | None:
        if value is None or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _shot_count_delta(self, incoming: int | None) -> int:
        if incoming is None:
            return 0
        if self._last_reported_shot_count is None or incoming < self._last_reported_shot_count:
            self._last_reported_shot_count = incoming
            return 0
        delta = incoming - self._last_reported_shot_count
        self._last_reported_shot_count = incoming
        return delta
