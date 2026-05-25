"""Safety configuration for conservative Tello control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyConfig:
    """Runtime limits for manual Tello commands."""

    min_battery: int = 25
    move_distance: int = 30
    vertical_distance: int = 30
    rotation_degrees: int = 30
    speed: int = 30
    min_flip_battery: int = 50

    def validate(self) -> None:
        _validate_range("min_battery", self.min_battery, 1, 100)
        _validate_range("move_distance", self.move_distance, 20, 500)
        _validate_range("vertical_distance", self.vertical_distance, 20, 500)
        _validate_range("rotation_degrees", self.rotation_degrees, 1, 360)
        _validate_range("speed", self.speed, 10, 100)
        _validate_range("min_flip_battery", self.min_flip_battery, 1, 100)


def _validate_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}, got {value}")
