"""Terminal pilot UI for SSH-based manual flight."""

from __future__ import annotations

import shutil
import sys
import threading
import time
import os
from dataclasses import dataclass
from typing import TextIO

from .controller import DroneController
from .rc_control import RCControlState
from .telemetry_store import TelemetryStore


@dataclass(frozen=True)
class PilotUIConfig:
    refresh_hz: float = 5.0
    min_width: int = 72


SCENARIO_HELP_LINES = (
    "Scenario 1  RC front-view infinity",
    "Scenario 2  SDK corner: up/right/down/left 50cm, 1s waits",
    "Scenario 3  Demo rectangle center return",
    "Scenario 4  Stage demo: up 50cm, forward/left/right 200cm, up 100cm, hover until hit",
)


class PilotTerminalUI:
    """Small ANSI terminal dashboard that stays inside the active SSH session."""

    def __init__(
        self,
        controller: DroneController,
        store: TelemetryStore | None,
        rc_state: RCControlState,
        *,
        config: PilotUIConfig | None = None,
        stream: TextIO = sys.stdout,
    ) -> None:
        self.controller = controller
        self.store = store
        self.rc_state = rc_state
        self.config = config or PilotUIConfig()
        self.stream = stream
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._message = "Ready"
        self._message_level = "INFO"

    def __enter__(self) -> "PilotTerminalUI":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.stop()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        with self._lock:
            self.stream.write("\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H")
            self.stream.flush()
        self._thread = threading.Thread(target=self._run, name="pilot-terminal-ui", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)
        with self._lock:
            self.stream.write("\x1b[?25h\x1b[?1049l")
            self.stream.flush()

    def notify(self, message: str, level: str = "INFO") -> None:
        self._message = message
        self._message_level = level
        self.render_once()

    def render_once(self) -> None:
        text = render_pilot_ui(
            self.controller,
            self.store,
            self.rc_state,
            message=self._message,
            message_level=self._message_level,
            size=shutil.get_terminal_size((100, 32)),
        )
        with self._lock:
            self.stream.write("\x1b[H\x1b[2J")
            self.stream.write(text)
            self.stream.flush()

    def _run(self) -> None:
        interval = 1.0 / max(1.0, self.config.refresh_hz)
        while not self._stop.is_set():
            self.render_once()
            self._stop.wait(interval)


def render_pilot_ui(
    controller: DroneController,
    store: TelemetryStore | None,
    rc_state: RCControlState,
    *,
    message: str,
    message_level: str,
    size: os.terminal_size,
) -> str:
    width = max(72, size.columns)
    height = max(20, size.lines)
    snapshot = store.snapshot() if store else None
    tello = snapshot.tello if snapshot else None
    events = list(snapshot.events)[-7:] if snapshot else []
    battery = controller.battery if controller.battery is not None else (tello.battery if tello else None)
    speed = controller.safety.speed
    lr, fb, ud, yaw = rc_state.snapshot()

    lines = [
        "Tello Pilot",
        rule(width),
        "Status  "
        f"connected={yes_no(controller.connected)}  "
        f"airborne={yes_no(controller.airborne)}  "
        f"battery={fmt(battery, '%')}  "
        f"speed={speed} cm/s",
        f"Battery {bar(battery, 0, 100, 34)}",
        "",
        "RC channels",
        f"  left/right   {signed(lr):>4}  {center_bar(lr, 100, 30)}",
        f"  forward/back {signed(fb):>4}  {center_bar(fb, 100, 30)}",
        f"  up/down      {signed(ud):>4}  {center_bar(ud, 100, 30)}",
        f"  yaw          {signed(yaw):>4}  {center_bar(yaw, 100, 30)}",
        "",
        "Controls",
        "  t takeoff   l land   w/s/a/d move   arrows up/down/yaw",
        "  1 flip left   2 flip forward   3 flip back   4 flip right",
        "  p run scenario   x land now   e emergency stop   q quit",
        "",
        *SCENARIO_HELP_LINES,
        "",
        f"Safety  min battery={controller.safety.min_battery}%  "
        f"flip min battery={controller.safety.min_flip_battery}%  "
        f"rc speed active until key repeat stops",
        "",
        f"Message [{message_level}] {message}",
    ]

    if tello and tello.last_command:
        lines.append(f"Last command  {tello.last_command}")
    if snapshot:
        lines.append(f"Command rate  {snapshot.tello.command_rate:.0f}/sec")

    if events:
        lines.extend(["", "Recent events"])
        for event in events:
            stamp = time.strftime("%H:%M:%S", time.localtime(event.timestamp))
            lines.append(f"  {stamp} {event.level:<5} {event.message}")

    return "\n".join(trim(line, width) for line in lines[: height - 1]) + "\n"


def rule(width: int) -> str:
    return "-" * min(width, 120)


def fmt(value: int | float | None, suffix: str = "") -> str:
    if value is None:
        return "-"
    if isinstance(value, float) and not value.is_integer():
        return f"{value:.1f}{suffix}"
    return f"{int(value)}{suffix}"


def yes_no(value: bool) -> str:
    return "yes" if value else "no"


def signed(value: int) -> str:
    return f"{value:+d}"


def bar(value: int | float | None, minimum: float, maximum: float, width: int) -> str:
    if value is None:
        return "[" + "-" * width + "] -"
    ratio = max(0.0, min(1.0, (float(value) - minimum) / (maximum - minimum)))
    filled = round(ratio * width)
    return "[" + "#" * filled + "-" * (width - filled) + f"] {fmt(value, '%')}"


def center_bar(value: int, limit: int, width: int) -> str:
    clamped = max(-limit, min(limit, value))
    half = width // 2
    left_fill = round((abs(min(0, clamped)) / limit) * half)
    right_fill = round((max(0, clamped) / limit) * half)
    left = "-" * (half - left_fill) + "#" * left_fill
    right = "#" * right_fill + "-" * (half - right_fill)
    return f"[{left}|{right}]"


def trim(line: str, width: int) -> str:
    if len(line) <= width:
        return line
    if width <= 1:
        return ""
    return line[: width - 1]
