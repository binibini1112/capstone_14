"""Saved flight scenario loading and execution."""

from __future__ import annotations

import json
import time
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from .controller import DroneController, DroneError


@dataclass(frozen=True)
class ScenarioStep:
    command: str
    params: dict[str, Any]

    @property
    def label(self) -> str:
        details = ", ".join(f"{key}={value}" for key, value in self.params.items())
        return self.command if not details else f"{self.command}({details})"


@dataclass(frozen=True)
class FlightScenario:
    name: str
    loops: int
    steps: tuple[ScenarioStep, ...]
    description: str = ""


StepCallback = Callable[[int, int, ScenarioStep], None]


def load_scenario(path: str | Path) -> FlightScenario:
    scenario_path = Path(path)
    try:
        payload = json.loads(scenario_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise DroneError(f"Scenario file cannot be read: {scenario_path}") from exc
    except json.JSONDecodeError as exc:
        raise DroneError(f"Scenario file is not valid JSON: {scenario_path}: {exc}") from exc

    if not isinstance(payload, dict):
        raise DroneError("Scenario root must be a JSON object.")

    name = str(payload.get("name") or scenario_path.stem)
    description = str(payload.get("description") or "")
    loops = _int_field(payload, "loops", default=1, minimum=1, maximum=100)
    raw_steps = payload.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise DroneError("Scenario must contain a non-empty steps list.")

    steps = tuple(_parse_step(item) for item in raw_steps)
    return FlightScenario(name=name, description=description, loops=loops, steps=steps)


def execute_scenario(
    controller: DroneController,
    scenario: FlightScenario,
    *,
    loops: int | None = None,
    on_step: StepCallback | None = None,
    stop_event: threading.Event | None = None,
) -> None:
    controller._require_airborne()
    total_loops = scenario.loops if loops is None else loops
    if total_loops < 1:
        raise DroneError("Scenario loops must be at least 1.")

    for loop_index in range(1, total_loops + 1):
        for step in scenario.steps:
            if stop_event and stop_event.is_set():
                raise DroneError("Scenario stopped by operator.")
            if on_step:
                on_step(loop_index, total_loops, step)
            execute_step(controller, step, stop_event=stop_event)


def execute_step(
    controller: DroneController,
    step: ScenarioStep,
    *,
    stop_event: threading.Event | None = None,
) -> None:
    command = step.command
    params = step.params
    if command == "wait":
        wait_interruptibly(
            _float_param(params, "seconds", minimum=0.0, maximum=60.0),
            stop_event,
        )
        return
    if command == "wait_until_stop":
        controller._require_airborne()
        wait_until_stop(stop_event)
        return
    if command == "rc":
        execute_rc_step(
            controller,
            left_right=_int_param(params, "left_right", minimum=-100, maximum=100),
            forward_back=_int_param(params, "forward_back", minimum=-100, maximum=100),
            up_down=_int_param(params, "up_down", minimum=-100, maximum=100),
            yaw=_int_param(params, "yaw", minimum=-100, maximum=100),
            seconds=_float_param(params, "seconds", minimum=0.05, maximum=10.0),
            stop_event=stop_event,
        )
        return
    if command == "go":
        controller.go_xyz_speed(
            _int_param(params, "x", minimum=-500, maximum=500),
            _int_param(params, "y", minimum=-500, maximum=500),
            _int_param(params, "z", minimum=-500, maximum=500),
            _int_param(params, "speed", minimum=10, maximum=100),
        )
        return
    if command in {"rotate_left", "rotate_right"}:
        degrees = _int_param(params, "degrees", minimum=1, maximum=360)
        controller._require_airborne()
        if command == "rotate_left":
            controller.tello.rotate_counter_clockwise(degrees)
        else:
            controller.tello.rotate_clockwise(degrees)
        return

    cm = _int_param(params, "cm", minimum=20, maximum=500)
    actions = {
        "forward": controller.tello.move_forward,
        "back": controller.tello.move_back,
        "left": controller.tello.move_left,
        "right": controller.tello.move_right,
        "up": controller.tello.move_up,
        "down": controller.tello.move_down,
    }
    action = actions.get(command)
    if action is None:
        raise DroneError(f"Unsupported scenario command: {command}")
    controller._require_airborne()
    action(cm)


def _parse_step(value: Any) -> ScenarioStep:
    if not isinstance(value, dict):
        raise DroneError("Each scenario step must be an object.")
    command = value.get("command")
    if not isinstance(command, str) or not command.strip():
        raise DroneError("Each scenario step requires a command string.")
    params = {key: item for key, item in value.items() if key != "command"}
    step = ScenarioStep(command=command.strip().lower(), params=params)
    _validate_step(step)
    return step


def _validate_step(step: ScenarioStep) -> None:
    if step.command == "wait":
        _float_param(step.params, "seconds", minimum=0.0, maximum=60.0)
        return
    if step.command == "wait_until_stop":
        return
    if step.command == "rc":
        _int_param(step.params, "left_right", minimum=-100, maximum=100)
        _int_param(step.params, "forward_back", minimum=-100, maximum=100)
        _int_param(step.params, "up_down", minimum=-100, maximum=100)
        _int_param(step.params, "yaw", minimum=-100, maximum=100)
        _float_param(step.params, "seconds", minimum=0.05, maximum=10.0)
        return
    if step.command == "go":
        x = _int_param(step.params, "x", minimum=-500, maximum=500)
        y = _int_param(step.params, "y", minimum=-500, maximum=500)
        z = _int_param(step.params, "z", minimum=-500, maximum=500)
        _int_param(step.params, "speed", minimum=10, maximum=100)
        if max(abs(x), abs(y), abs(z)) < 20:
            raise DroneError("Scenario go command must move at least 20 cm on one axis.")
        return
    if step.command in {"rotate_left", "rotate_right"}:
        _int_param(step.params, "degrees", minimum=1, maximum=360)
        return
    if step.command in {"forward", "back", "left", "right", "up", "down"}:
        _int_param(step.params, "cm", minimum=20, maximum=500)
        return
    raise DroneError(f"Unsupported scenario command: {step.command}")


def execute_rc_step(
    controller: DroneController,
    *,
    left_right: int,
    forward_back: int,
    up_down: int,
    yaw: int,
    seconds: float,
    rate_hz: float = 20.0,
    stop_event: threading.Event | None = None,
) -> None:
    controller._require_airborne()
    interval = 1.0 / rate_hz
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            if stop_event and stop_event.is_set():
                break
            controller.rc_control(left_right, forward_back, up_down, yaw)
            wait_interruptibly(interval, stop_event)
    finally:
        controller.rc_control(0, 0, 0, 0)


def wait_interruptibly(seconds: float, stop_event: threading.Event | None) -> None:
    if stop_event is None:
        time.sleep(seconds)
        return
    stop_event.wait(seconds)


def wait_until_stop(stop_event: threading.Event | None) -> None:
    if stop_event is None:
        raise DroneError("Scenario wait_until_stop requires a stop event.")
    stop_event.wait()
    raise DroneError("Scenario stopped by operator.")


def _int_field(
    payload: dict[str, Any],
    name: str,
    *,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    value = payload.get(name, default)
    return _coerce_int(value, name, minimum=minimum, maximum=maximum)


def _int_param(params: dict[str, Any], name: str, *, minimum: int, maximum: int) -> int:
    if name not in params:
        raise DroneError(f"Scenario command missing parameter: {name}")
    return _coerce_int(params[name], name, minimum=minimum, maximum=maximum)


def _float_param(params: dict[str, Any], name: str, *, minimum: float, maximum: float) -> float:
    if name not in params:
        raise DroneError(f"Scenario command missing parameter: {name}")
    try:
        value = float(params[name])
    except (TypeError, ValueError) as exc:
        raise DroneError(f"Scenario parameter {name} must be a number.") from exc
    if not minimum <= value <= maximum:
        raise DroneError(f"Scenario parameter {name} must be between {minimum} and {maximum}.")
    return value


def _coerce_int(value: Any, name: str, *, minimum: int, maximum: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise DroneError(f"Scenario parameter {name} must be an integer.") from exc
    if not minimum <= parsed <= maximum:
        raise DroneError(f"Scenario parameter {name} must be between {minimum} and {maximum}.")
    return parsed
