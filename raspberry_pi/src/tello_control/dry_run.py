"""Dry-run Tello client for testing the CLI without a drone."""

from __future__ import annotations


class DryRunTello:
    """Small djitellopy-compatible stand-in that never talks to hardware."""

    def __init__(self, battery: int = 85) -> None:
        self.battery = battery
        self.speed = 0
        self.calls: list[tuple[str, int | tuple[int, int, int, int] | None]] = []

    def connect(self) -> None:
        self.calls.append(("connect", None))

    def get_battery(self) -> int:
        self.calls.append(("get_battery", None))
        return self.battery

    def set_speed(self, speed: int) -> None:
        self.speed = speed
        self.calls.append(("set_speed", speed))

    def takeoff(self) -> None:
        self.calls.append(("takeoff", None))

    def land(self) -> None:
        self.calls.append(("land", None))

    def emergency(self) -> None:
        self.calls.append(("emergency", None))

    def end(self) -> None:
        self.calls.append(("end", None))

    def move_forward(self, distance: int) -> None:
        self.calls.append(("move_forward", distance))

    def move_back(self, distance: int) -> None:
        self.calls.append(("move_back", distance))

    def move_left(self, distance: int) -> None:
        self.calls.append(("move_left", distance))

    def move_right(self, distance: int) -> None:
        self.calls.append(("move_right", distance))

    def move_up(self, distance: int) -> None:
        self.calls.append(("move_up", distance))

    def move_down(self, distance: int) -> None:
        self.calls.append(("move_down", distance))

    def rotate_counter_clockwise(self, degrees: int) -> None:
        self.calls.append(("rotate_counter_clockwise", degrees))

    def rotate_clockwise(self, degrees: int) -> None:
        self.calls.append(("rotate_clockwise", degrees))

    def flip_forward(self) -> None:
        self.calls.append(("flip_forward", None))

    def flip_back(self) -> None:
        self.calls.append(("flip_back", None))

    def flip_left(self) -> None:
        self.calls.append(("flip_left", None))

    def flip_right(self) -> None:
        self.calls.append(("flip_right", None))

    def send_rc_control(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> None:
        self.calls.append(("send_rc_control", (left_right, forward_back, up_down, yaw)))
        self.last_rc = (left_right, forward_back, up_down, yaw)

    def go_xyz_speed(self, x: int, y: int, z: int, speed: int) -> None:
        self.calls.append(("go_xyz_speed", (x, y, z, speed)))
