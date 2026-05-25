"""Small visualization helpers shared by tests and the browser UI."""

from __future__ import annotations


def clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def radar_to_canvas(
    x_norm: float | None,
    y_norm: float | None,
    width: int,
    height: int,
    padding: int = 12,
) -> tuple[float, float]:
    """Map normalized -1..1 target error to a circular radar canvas point."""

    x = clamp(float(x_norm or 0.0))
    y = clamp(float(y_norm or 0.0))
    radius = max(0.0, min(width, height) / 2.0 - padding)
    center_x = width / 2.0
    center_y = height / 2.0
    return center_x + x * radius, center_y + y * radius
