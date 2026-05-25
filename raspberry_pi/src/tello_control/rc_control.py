"""Continuous RC control loop for smooth manual Tello flight."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .controller import DroneController, DroneError


@dataclass
class RCControlState:
    """Thread-safe velocity state for Tello send_rc_control."""

    hold_seconds: float = 0.22
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _channels: dict[str, int] = field(
        default_factory=lambda: {"lr": 0, "fb": 0, "ud": 0, "yaw": 0}
    )
    _expires_at: dict[str, float] = field(
        default_factory=lambda: {"lr": 0.0, "fb": 0.0, "ud": 0.0, "yaw": 0.0}
    )

    def pulse(self, channel: str, value: int, now: float | None = None) -> None:
        current = time.monotonic() if now is None else now
        with self._lock:
            self._channels[channel] = value
            self._expires_at[channel] = current + self.hold_seconds

    def stop_motion(self) -> None:
        with self._lock:
            for channel in self._channels:
                self._channels[channel] = 0
                self._expires_at[channel] = 0.0

    def snapshot(self, now: float | None = None) -> tuple[int, int, int, int]:
        current = time.monotonic() if now is None else now
        with self._lock:
            values: dict[str, int] = {}
            for channel, value in self._channels.items():
                if current > self._expires_at[channel]:
                    self._channels[channel] = 0
                    values[channel] = 0
                else:
                    values[channel] = value
            return values["lr"], values["fb"], values["ud"], values["yaw"]


class RCControlLoop:
    """Background loop that sends RC commands independently from UI/dashboard work."""

    def __init__(
        self,
        controller: DroneController,
        state: RCControlState,
        rate_hz: float = 20.0,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be greater than zero")
        self.controller = controller
        self.state = state
        self.interval = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.last_error: Exception | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tello-rc-control", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                self._stop.wait(next_tick - now)
                continue
            next_tick = now + self.interval

            if not self.controller.connected or not self.controller.airborne:
                continue

            lr, fb, ud, yaw = self.state.snapshot(now)
            try:
                self.controller.rc_control(lr, fb, ud, yaw)
            except DroneError as exc:
                self.last_error = exc
            except Exception as exc:  # noqa: BLE001
                self.last_error = exc
