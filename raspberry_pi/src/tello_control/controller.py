"""High-level controller around a Tello-compatible client."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .safety import SafetyConfig


class DroneError(RuntimeError):
    """Raised when a safety check or drone operation fails."""


TELLO_CONNECT_HELP = (
    "Could not connect to the Tello. Check that the drone is powered on, "
    "this computer is connected to the Tello Wi-Fi, and UDP 8889 to "
    "192.168.10.1 is reachable."
)


@dataclass
class DroneController:
    """Safe command wrapper for DJI Tello manual control."""

    tello: Any
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    connected: bool = False
    airborne: bool = False
    battery: int | None = None

    def connect(self) -> int:
        self.safety.validate()
        try:
            self.tello.connect()
        except Exception as exc:  # noqa: BLE001
            raise DroneError(f"{TELLO_CONNECT_HELP} Original error: {exc}") from exc
        self.connected = True

        try:
            battery = int(self.tello.get_battery())
        except Exception as exc:  # noqa: BLE001
            raise DroneError(f"{TELLO_CONNECT_HELP} Battery read failed: {exc}") from exc
        self.battery = battery
        if battery < self.safety.min_battery:
            self.end()
            raise DroneError(
                f"Battery too low: {battery}%. Minimum required: {self.safety.min_battery}%."
            )

        try:
            self.tello.set_speed(self.safety.speed)
        except Exception as exc:  # noqa: BLE001
            raise DroneError(f"{TELLO_CONNECT_HELP} Speed setup failed: {exc}") from exc
        return battery

    def takeoff(self) -> None:
        self._require_connected()
        if self.airborne:
            return
        self.tello.takeoff()
        self.airborne = True

    def land(self) -> None:
        self._require_connected()
        if not self.airborne:
            return
        self.tello.land()
        self.airborne = False

    def emergency(self) -> None:
        self._require_connected()
        self.tello.emergency()
        self.airborne = False

    def move_forward(self) -> None:
        self._require_airborne()
        self.tello.move_forward(self.safety.move_distance)

    def move_back(self) -> None:
        self._require_airborne()
        self.tello.move_back(self.safety.move_distance)

    def move_left(self) -> None:
        self._require_airborne()
        self.tello.move_left(self.safety.move_distance)

    def move_right(self) -> None:
        self._require_airborne()
        self.tello.move_right(self.safety.move_distance)

    def move_up(self) -> None:
        self._require_airborne()
        self.tello.move_up(self.safety.vertical_distance)

    def move_down(self) -> None:
        self._require_airborne()
        self.tello.move_down(self.safety.vertical_distance)

    def rotate_left(self) -> None:
        self._require_airborne()
        self.tello.rotate_counter_clockwise(self.safety.rotation_degrees)

    def rotate_right(self) -> None:
        self._require_airborne()
        self.tello.rotate_clockwise(self.safety.rotation_degrees)

    def flip_forward(self) -> None:
        self._flip("f", "flip_forward")

    def flip_back(self) -> None:
        self._flip("b", "flip_back")

    def flip_left(self) -> None:
        self._flip("l", "flip_left")

    def flip_right(self) -> None:
        self._flip("r", "flip_right")

    def rc_control(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> None:
        self._require_airborne()
        self.tello.send_rc_control(
            self._clamp_rc(left_right),
            self._clamp_rc(forward_back),
            self._clamp_rc(up_down),
            self._clamp_rc(yaw),
        )

    def go_xyz_speed(self, x: int, y: int, z: int, speed: int) -> None:
        self._require_airborne()
        self.tello.go_xyz_speed(
            self._clamp_go_axis(x),
            self._clamp_go_axis(y),
            self._clamp_go_axis(z),
            max(10, min(100, int(speed))),
        )

    def shutdown(self) -> None:
        if self.connected and self.airborne:
            self.land()
        if self.connected:
            self.end()

    def end(self) -> None:
        self.tello.end()
        self.connected = False

    def refresh_battery(self) -> int:
        self._require_connected()
        self.battery = int(self.tello.get_battery())
        return self.battery

    def _flip(self, direction: str, method_name: str) -> None:
        self._require_airborne()
        battery = self.refresh_battery()
        if battery < self.safety.min_flip_battery:
            raise DroneError(
                f"Battery too low for flip: {battery}%. "
                f"Minimum required: {self.safety.min_flip_battery}%."
            )

        method = getattr(self.tello, method_name, None)
        if callable(method):
            method()
            return

        flip = getattr(self.tello, "flip", None)
        if callable(flip):
            flip(direction)
            return

        raise DroneError("Tello client does not support flip commands.")

    def _require_connected(self) -> None:
        if not self.connected:
            raise DroneError("Drone is not connected.")

    def _require_airborne(self) -> None:
        self._require_connected()
        if not self.airborne:
            raise DroneError("Drone must be airborne before movement commands.")

    @staticmethod
    def _clamp_rc(value: int) -> int:
        return max(-100, min(100, int(value)))

    @staticmethod
    def _clamp_go_axis(value: int) -> int:
        return max(-500, min(500, int(value)))
