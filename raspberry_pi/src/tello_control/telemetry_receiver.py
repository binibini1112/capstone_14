"""UDP receiver for Jetson tracking telemetry."""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from collections.abc import Callable
from typing import Any

from .logger import TelemetryLogger
from .telemetry_model import (
    AudioData,
    ErrorData,
    JetsonTrackingData,
    LaserData,
    PTZData,
    UltraPSData,
)
from .telemetry_store import TelemetryStore


class TelemetryReceiver:
    """Receives UDP JSON telemetry and updates a store."""

    def __init__(
        self,
        store: TelemetryStore,
        host: str = "0.0.0.0",
        port: int = 5005,
        logger: TelemetryLogger | None = None,
        on_hit: Callable[[JetsonTrackingData], None] | None = None,
    ) -> None:
        self.store = store
        self.host = host
        self.port = port
        self.logger = logger
        self.on_hit = on_hit
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._last_laser_hit = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._thread = threading.Thread(target=self.run, name="telemetry-receiver", daemon=True)
        self._thread.start()
        self._ready.wait(1.0)

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        if self._thread is not None:
            self._thread.join(timeout)

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock = sock
        try:
            sock.bind((self.host, self.port))
            sock.settimeout(0.2)
            self._ready.set()
            self.store.add_event(f"Telemetry receiver listening on UDP {self.host}:{self.port}")
            while not self._stop.is_set():
                try:
                    packet, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break
                data = parse_tracking_packet(packet)
                if data is None:
                    continue
                self.store.update_tracking(data)
                if self.logger:
                    self.logger.log_tracking(data)
                laser = data.laser
                laser_hit = bool(laser and laser.hit_detected)
                if self.on_hit and laser_hit and not self._last_laser_hit:
                    threading.Thread(target=self.on_hit, args=(data,), daemon=True).start()
                self._last_laser_hit = laser_hit
        finally:
            self._ready.set()
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None


class FakeJetsonTelemetry:
    """Generates moving telemetry samples for dashboard dry-runs."""

    STATES = ("SCANNING", "DETECTED", "TRACKING", "LOCKED", "LOST")

    def __init__(
        self,
        store: TelemetryStore,
        interval: float = 0.05,
        logger: TelemetryLogger | None = None,
    ) -> None:
        self.store = store
        self.interval = interval
        self.logger = logger
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self.run, name="fake-jetson", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def run(self) -> None:
        self.store.add_event("Fake Jetson telemetry started")
        start = time.monotonic()
        frame_id = 0
        while not self._stop.is_set():
            elapsed = time.monotonic() - start
            cycle = int(elapsed / 3.0) % len(self.STATES)
            state = self.STATES[cycle]
            target_found = state not in {"SCANNING", "LOST"}
            x_norm = math.sin(elapsed * 0.8) * 0.75
            y_norm = math.cos(elapsed * 0.6) * 0.65
            confidence = 0.0 if not target_found else 0.5 + 0.45 * abs(math.sin(elapsed * 0.5))
            data = JetsonTrackingData(
                timestamp=time.time(),
                frame_id=frame_id,
                fps=25.0 + 5.0 * math.sin(elapsed * 0.4),
                target_found=target_found,
                confidence=confidence,
                error=ErrorData(
                    x_px=x_norm * 640.0,
                    y_px=y_norm * 360.0,
                    x_norm=x_norm,
                    y_norm=y_norm,
                ),
                ptz=PTZData(pan_deg=x_norm * 45.0, tilt_deg=y_norm * 25.0),
                audio=AudioData(
                    enabled=True,
                    direction_deg=(elapsed * 40.0) % 360.0,
                    confidence=0.5 + 0.4 * abs(math.cos(elapsed * 0.7)),
                ),
                ultra_ps=UltraPSData(
                    motor_deg=(elapsed * 28.0 + 20.0 * math.sin(elapsed * 0.9)) % 360.0,
                ),
                laser=LaserData(
                    armed=state == "LOCKED",
                    fired=state == "LOCKED" and frame_id % 20 == 0,
                    hit_detected=state == "LOCKED" and frame_id % 40 == 0,
                ),
                state=state,
            )
            self.store.update_tracking(data)
            if self.logger:
                self.logger.log_tracking(data)
            frame_id += 1
            self._stop.wait(self.interval)


def parse_tracking_packet(packet: bytes) -> JetsonTrackingData | None:
    try:
        payload: Any = json.loads(packet.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    return JetsonTrackingData.from_dict(payload)
