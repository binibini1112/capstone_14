"""Single-key terminal input helpers."""

from __future__ import annotations

import sys
import termios
import tty
from dataclasses import dataclass


KEY_UP = "up"
KEY_DOWN = "down"
KEY_LEFT = "left"
KEY_RIGHT = "right"


@dataclass
class RawTerminal:
    """Context manager that puts stdin into cbreak mode."""

    fd: int | None = None
    previous: list[int | bytes] | None = None

    def __enter__(self) -> "RawTerminal":
        if self.fd is None:
            self.fd = sys.stdin.fileno()
        self.previous = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        if self.previous is not None and self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.previous)


def read_key() -> str:
    first = sys.stdin.read(1)
    if first != "\x1b":
        return first.lower()

    second = sys.stdin.read(1)
    third = sys.stdin.read(1)
    if second == "[":
        return {
            "A": KEY_UP,
            "B": KEY_DOWN,
            "C": KEY_RIGHT,
            "D": KEY_LEFT,
        }.get(third, "")
    return ""
