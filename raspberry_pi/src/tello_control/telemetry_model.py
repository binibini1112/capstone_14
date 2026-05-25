"""Telemetry dataclasses for Jetson tracking and dashboard snapshots."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

JETSON_TIMEOUT_SEC = 1.0
JETSON_DISCONNECTED_SEC = 3.0


@dataclass(frozen=True)
class BBox:
    x1: float | None = None
    y1: float | None = None
    x2: float | None = None
    y2: float | None = None
    cx: float | None = None
    cy: float | None = None
    w: float | None = None
    h: float | None = None


@dataclass(frozen=True)
class FrameData:
    width: int | None = None
    height: int | None = None


@dataclass(frozen=True)
class ErrorData:
    x_px: float | None = None
    y_px: float | None = None
    x_norm: float | None = None
    y_norm: float | None = None


@dataclass(frozen=True)
class PTZData:
    pan_deg: float | None = None
    tilt_deg: float | None = None
    pan_cmd: float | None = None
    tilt_cmd: float | None = None


@dataclass(frozen=True)
class AudioData:
    enabled: bool | None = None
    drone_detected: bool | None = None
    sector: str | None = None
    sector_index: int | None = None
    sector_count: int | None = None
    target_motor_deg: float | None = None
    direction_deg: float | None = None
    confidence: float | None = None
    detected: bool | None = None
    fallback_active: bool | None = None
    status: str | None = None


@dataclass(frozen=True)
class UltraPSData:
    motor_deg: float | None = None
    motor_direction_deg: float | None = None
    fan_deg: float | None = None
    heading_deg: float | None = None
    direction_deg: float | None = None
    front_pan: float | None = None
    pan_tick: float | None = None


@dataclass(frozen=True)
class LaserData:
    armed: bool | None = None
    fired: bool | None = None
    shot_count: int | None = None
    hit_detected: bool | None = None


@dataclass(frozen=True)
class JetsonTrackingData:
    timestamp: float
    target_found: bool
    state: str
    frame_id: int | None = None
    fps: float | None = None
    confidence: float | None = None
    bbox: BBox | None = None
    frame: FrameData | None = None
    error: ErrorData | None = None
    ptz: PTZData | None = None
    audio: AudioData | None = None
    ultra_ps: UltraPSData | None = None
    laser: LaserData | None = None

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "JetsonTrackingData | None":
        if not {"timestamp", "target_found", "state"}.issubset(payload):
            return None

        try:
            timestamp = float(payload["timestamp"])
        except (TypeError, ValueError):
            return None
        target_found = _optional_bool(payload.get("target_found"))
        if target_found is None:
            return None

        return cls(
            timestamp=timestamp,
            target_found=target_found,
            state=str(payload["state"]),
            frame_id=_optional_int(payload.get("frame_id")),
            fps=_optional_float(payload.get("fps")),
            confidence=_optional_float(payload.get("confidence")),
            bbox=_parse_dataclass(BBox, payload.get("bbox")),
            frame=_parse_dataclass(FrameData, payload.get("frame")),
            error=_parse_dataclass(ErrorData, payload.get("error")),
            ptz=_parse_dataclass(PTZData, payload.get("ptz")),
            audio=_parse_dataclass(AudioData, payload.get("audio")),
            ultra_ps=_parse_dataclass(UltraPSData, _first_payload(payload, "ultra_ps", "ultraps", "ultraPs")),
            laser=_parse_dataclass(LaserData, payload.get("laser")),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TelloStatus:
    connected: bool = False
    airborne: bool = False
    battery: int | None = None
    speed: int | None = None
    last_command: str | None = None
    command_rate: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class EventLogEntry:
    timestamp: float
    level: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DashboardSnapshot:
    timestamp: float
    tello: TelloStatus
    jetson_status: str
    last_received_age: float | None
    tracking: JetsonTrackingData | None = None
    history: list[dict[str, Any]] = field(default_factory=list)
    events: list[EventLogEntry] = field(default_factory=list)
    hit_count: int = 0
    shot_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "tello": self.tello.to_dict(),
            "jetson_status": self.jetson_status,
            "last_received_age": self.last_received_age,
            "tracking": self.tracking.to_dict() if self.tracking else None,
            "history": self.history,
            "events": [event.to_dict() for event in self.events],
            "hit_count": self.hit_count,
            "shot_count": self.shot_count,
        }


def _parse_dataclass(model: type[Any], value: Any) -> Any:
    if not isinstance(value, dict):
        return None
    fields = getattr(model, "__dataclass_fields__", {})
    return model(**{name: _coerce_value(value.get(name)) for name in fields})


def _first_payload(payload: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in payload:
            return payload[key]
    return None


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes"}:
            return True
        if lowered in {"false", "0", "no"}:
            return False
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    return None


def _coerce_value(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, float)):
        return value
    try:
        return float(value)
    except (TypeError, ValueError):
        return value
