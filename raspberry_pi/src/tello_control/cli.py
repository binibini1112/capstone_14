"""Keyboard CLI for DJI Tello manual flight."""

from __future__ import annotations

import argparse
import sys
import threading
from collections.abc import Callable
from contextlib import ExitStack
from pathlib import Path

from .controller import DroneController, DroneError
from .dry_run import DryRunTello
from .keys import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP, RawTerminal, read_key
from .logger import TelemetryLogger
from .rc_control import RCControlLoop, RCControlState
from .safety import SafetyConfig
from .scenario import FlightScenario, ScenarioStep, execute_scenario, load_scenario
from .telemetry_receiver import FakeJetsonTelemetry, TelemetryReceiver
from .telemetry_store import TelemetryStore


BATTERY_REFRESH_SECONDS = 5.0
HIT_RESPONSE_FLIP_LAND = "flip-land"
HIT_RESPONSE_FREE_FALL = "free-fall"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Conservative keyboard control for DJI Tello.")
    parser.add_argument("--min-battery", type=int, default=25, help="Minimum battery percent.")
    parser.add_argument("--move-distance", type=int, default=30, help="Horizontal move step in cm.")
    parser.add_argument("--vertical-distance", type=int, default=30, help="Vertical move step in cm.")
    parser.add_argument("--rotation-degrees", type=int, default=30, help="Rotation step in degrees.")
    parser.add_argument("--speed", type=int, default=30, help="Flight speed in cm/s.")
    parser.add_argument("--min-flip-battery", type=int, default=50, help="Minimum battery percent for flips.")
    parser.add_argument("--rc-speed", type=int, default=35, help="RC horizontal/vertical speed, -100..100.")
    parser.add_argument("--rc-yaw-speed", type=int, default=55, help="RC yaw speed, -100..100.")
    parser.add_argument("--rc-rate-hz", type=float, default=20.0, help="RC command send rate in Hz.")
    parser.add_argument("--rc-hold-seconds", type=float, default=0.22, help="How long a key pulse remains active.")
    parser.add_argument("--dry-run", action="store_true", help="Use a fake Tello client.")
    parser.add_argument(
        "--scenario",
        default="1",
        help="Scenario id or JSON file loaded for the p key.",
    )
    parser.add_argument(
        "--scenario-loops",
        type=int,
        default=None,
        help="Override the scenario file loop count.",
    )
    parser.add_argument("--no-scenario", action="store_true", help="Disable p-key scenario execution.")
    parser.add_argument(
        "--no-control-ui",
        action="store_true",
        help="Disable the SSH terminal pilot UI and use plain key echo output.",
    )
    parser.add_argument("--dashboard", action="store_true", help="Start the local web dashboard.")
    parser.add_argument("--dashboard-host", default="0.0.0.0", help="Dashboard bind host.")
    parser.add_argument("--dashboard-port", type=int, default=8000, help="Dashboard HTTP port.")
    parser.add_argument("--telemetry-host", default="0.0.0.0", help="Jetson UDP bind host.")
    parser.add_argument("--telemetry-port", type=int, default=5005, help="Jetson UDP port.")
    parser.add_argument("--fake-jetson", action="store_true", help="Generate fake Jetson tracking data.")
    parser.add_argument("--log-dir", default="logs", help="Directory for flight/tracking logs.")
    parser.add_argument("--open-browser", action="store_true", help="Open the dashboard in the local browser.")
    parser.add_argument("--browser-display", default=":0", help="Display used when opening a browser from SSH.")
    parser.set_defaults(auto_hit_response=True)
    parser.add_argument(
        "--auto-hit-response",
        dest="auto_hit_response",
        action="store_true",
        help="Trigger the selected hit response when Jetson telemetry reports laser hit_detected. Enabled by default.",
    )
    parser.add_argument(
        "--no-auto-hit-response",
        dest="auto_hit_response",
        action="store_false",
        help="Disable automatic response to Jetson laser hit_detected telemetry.",
    )
    parser.add_argument(
        "--hit-response",
        choices=(HIT_RESPONSE_FLIP_LAND, HIT_RESPONSE_FREE_FALL),
        default=HIT_RESPONSE_FLIP_LAND,
        help="Action when Jetson laser hit_detected telemetry is received.",
    )
    return parser


def make_safety(args: argparse.Namespace) -> SafetyConfig:
    safety = SafetyConfig(
        min_battery=args.min_battery,
        move_distance=args.move_distance,
        vertical_distance=args.vertical_distance,
        rotation_degrees=args.rotation_degrees,
        speed=args.speed,
        min_flip_battery=args.min_flip_battery,
    )
    safety.validate()
    return safety


def run_cli(
    controller: DroneController,
    store: TelemetryStore | None = None,
    logger: TelemetryLogger | None = None,
    rc_speed: int = 35,
    rc_yaw_speed: int = 55,
    rc_rate_hz: float = 20.0,
    rc_hold_seconds: float = 0.22,
    control_ui: bool = True,
    scenario_loops: int | None = None,
    scenarios_enabled: bool = True,
    hit_response: "HitResponseCoordinator | None" = None,
) -> int:
    battery = controller.connect()
    update_tello_store(store, controller, battery=battery)
    record_event(store, logger, "Tello connected", f"battery={battery}")
    record_event(store, logger, "Battery check passed", f"battery={battery}")
    if battery <= controller.safety.min_battery + 5:
        record_event(store, logger, "Low battery warning", f"battery={battery}", level="WARN")
    print(f"Connected. Battery: {battery}%")
    print("Type ARM and press Enter to enable keyboard control.")

    if input("> ").strip() != "ARM":
        print("Aborted before flight.")
        controller.shutdown()
        update_tello_store(store, controller)
        record_event(store, logger, "Aborted before flight")
        return 1

    rc_state = RCControlState(hold_seconds=rc_hold_seconds)
    rc_loop = RCControlLoop(controller, rc_state, rate_hz=rc_rate_hz)
    battery_poller = BatteryPoller(controller, store)
    rc_loop.start()
    battery_poller.start()
    ui = None
    use_control_ui = control_ui and sys.stdin.isatty() and sys.stdout.isatty()
    scenario_runner = ScenarioRunner(
        scenario_loops,
        controller,
        rc_state,
        rc_loop,
        store,
        logger,
    )
    if hit_response is not None:
        hit_response.bind(controller, rc_state, rc_loop, scenario_runner, store, logger)

    try:
        with ExitStack() as control_stack:
            if use_control_ui:
                from .pilot_ui import PilotTerminalUI

                ui = control_stack.enter_context(PilotTerminalUI(controller, store, rc_state))
                ui.notify(control_armed_message(scenarios_enabled, scenario_loops))
            else:
                print_controls(rc_rate_hz, rc_speed, rc_yaw_speed)
            control_stack.enter_context(RawTerminal())
            while True:
                key = read_key()
                if key == "q":
                    notify(ui, "Quitting. Landing first if airborne.")
                    if scenario_runner.is_running():
                        scenario_runner.request_land()
                        scenario_runner.join(timeout=8.0)
                    rc_state.stop_motion()
                    controller.shutdown()
                    update_tello_store(store, controller)
                    record_event(store, logger, "Quit")
                    return 0
                if key == "x":
                    if scenario_runner.is_running():
                        scenario_runner.request_land()
                        notify(ui, "Scenario landing requested. Landing after current command.")
                        record_event(store, logger, "Scenario landing requested")
                        continue
                    notify(ui, "Landing now.")
                    rc_state.stop_motion()
                    controller.land()
                    record_command(store, logger, controller, "land now")
                    continue
                if key == "e":
                    notify(ui, "Emergency stop.")
                    scenario_runner.request_stop()
                    rc_state.stop_motion()
                    controller.emergency()
                    record_command(store, logger, controller, "emergency")
                    controller.end()
                    update_tello_store(store, controller)
                    return 2
                if key == "p":
                    if not scenarios_enabled:
                        notify(ui, "Scenarios disabled.", level="WARN")
                        continue
                    scenario = choose_scenario(ui)
                    if scenario is None:
                        continue
                    scenario_runner.ui = ui
                    scenario_runner.start(scenario)
                    continue

                if scenario_runner.is_running():
                    notify(ui, "Scenario running. x lands, e emergency, q quits.", level="WARN")
                    continue

                try:
                    handled = handle_key(
                        key,
                        controller,
                        rc_state,
                        rc_speed=rc_speed,
                        rc_yaw_speed=rc_yaw_speed,
                    )
                    if handled:
                        record_command(store, logger, controller, command_label(key))
                        print_status(key, ui)
                except DroneError as exc:
                    record_event(store, logger, "Ignored command", str(exc), level="WARN")
                    notify(ui, f"Ignored: {exc}", level="WARN")
    finally:
        battery_poller.stop()
        scenario_runner.request_stop()
        scenario_runner.join(timeout=3.0)
        rc_state.stop_motion()
        rc_loop.stop()


def handle_key(
    key: str,
    controller: DroneController,
    rc_state: RCControlState,
    *,
    rc_speed: int,
    rc_yaw_speed: int,
) -> bool:
    speed = clamp_rc_speed(rc_speed)
    yaw_speed = clamp_rc_speed(rc_yaw_speed)
    actions: dict[str, Callable[[], None]] = {
        "t": controller.takeoff,
        "l": controller.land,
        "1": controller.flip_left,
        "2": controller.flip_forward,
        "3": controller.flip_back,
        "4": controller.flip_right,
    }
    if key in actions:
        rc_state.stop_motion()
        actions[key]()
        return True

    pulses: dict[str, tuple[str, int]] = {
        "w": ("fb", speed),
        "s": ("fb", -speed),
        "a": ("lr", -speed),
        "d": ("lr", speed),
        KEY_UP: ("ud", speed),
        KEY_DOWN: ("ud", -speed),
        KEY_LEFT: ("yaw", -yaw_speed),
        KEY_RIGHT: ("yaw", yaw_speed),
    }
    pulse = pulses.get(key)
    if pulse is None:
        return False
    controller._require_airborne()
    channel, value = pulse
    rc_state.pulse(channel, value)
    return True


def clamp_rc_speed(value: int) -> int:
    return max(1, min(100, int(value)))


def print_controls(rate_hz: float, rc_speed: int, rc_yaw_speed: int) -> None:
    print(
        "Controls: t takeoff | l land | hold/repeat w/s/a/d move | "
        "arrows up/down/yaw | 1/2/3/4 flips | p scenario | x land now | e emergency | q quit"
    )
    print(f"RC mode: {rate_hz:.1f} Hz, speed={rc_speed}, yaw={rc_yaw_speed}")


def print_status(key: str, ui: object | None = None) -> None:
    notify(ui, command_label(key))


def notify(ui: object | None, message: str, level: str = "INFO") -> None:
    if ui is not None:
        ui.notify(message, level=level)
    else:
        print(f"\n{message}")


def command_label(key: str) -> str:
    label = {
        "t": "takeoff",
        "l": "land",
        "w": "forward",
        "s": "back",
        "a": "left",
        "d": "right",
        KEY_UP: "up",
        KEY_DOWN: "down",
        KEY_LEFT: "rotate left",
        KEY_RIGHT: "rotate right",
        "1": "flip left",
        "2": "flip forward",
        "3": "flip back",
        "4": "flip right",
        "p": "scenario",
    }.get(key, key)
    return label


def control_armed_message(scenarios_enabled: bool, scenario_loops: int | None) -> str:
    if not scenarios_enabled:
        return "Keyboard control armed. Scenarios disabled."
    loops = scenario_loops if scenario_loops is not None else "file"
    return f"Keyboard control armed. Press p then scenario number. loops={loops}."


def choose_scenario(ui: object | None) -> FlightScenario | None:
    notify(ui, scenario_selection_message())
    key = read_key()
    if key in {"", "\x1b", "q"}:
        notify(ui, "Scenario selection cancelled.")
        return None
    try:
        scenario = load_scenario(resolve_scenario_path(key))
    except DroneError as exc:
        notify(ui, f"Scenario load failed: {exc}", level="WARN")
        return None
    detail = f"{scenario.name} - {scenario.description}" if scenario.description else scenario.name
    notify(ui, f"Selected scenario {key}: {detail}. Starting.")
    return scenario


def scenario_selection_message(ids: tuple[str, ...] = ("1", "2", "3", "4")) -> str:
    lines = ["Select scenario number:"]
    for scenario_id in ids:
        try:
            scenario = load_scenario(resolve_scenario_path(scenario_id))
            lines.append(f"{scenario_id}: {scenario.name} - {scenario.description}")
        except DroneError as exc:
            lines.append(f"{scenario_id}: unavailable ({exc})")
    lines.append("Esc/q: cancel")
    return "\n".join(lines)


class ScenarioRunner:
    def __init__(
        self,
        scenario_loops: int | None,
        controller: DroneController,
        rc_state: RCControlState,
        rc_loop: RCControlLoop,
        store: TelemetryStore | None,
        logger: TelemetryLogger | None,
        ui: object | None = None,
    ) -> None:
        self.scenario: FlightScenario | None = None
        self.scenario_loops = scenario_loops
        self.controller = controller
        self.rc_state = rc_state
        self.rc_loop = rc_loop
        self.store = store
        self.logger = logger
        self.ui = ui
        self._stop_event = threading.Event()
        self._land_requested = threading.Event()
        self._resume_rc_after_stop = True
        self._thread: threading.Thread | None = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, scenario: FlightScenario | None) -> None:
        if scenario is None:
            notify(self.ui, "No scenario loaded. Use --scenario <file>.", level="WARN")
            return
        if self.is_running():
            notify(self.ui, "Scenario already running.", level="WARN")
            return
        self.scenario = scenario
        self._stop_event.clear()
        self._land_requested.clear()
        self._resume_rc_after_stop = True
        self._thread = threading.Thread(target=self._run, name="scenario-runner", daemon=True)
        self._thread.start()

    def request_stop(self, *, resume_rc: bool = True) -> None:
        self._resume_rc_after_stop = resume_rc
        self._stop_event.set()

    def request_land(self) -> None:
        self._land_requested.set()
        self._stop_event.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    def _run(self) -> None:
        scenario = self.scenario
        assert scenario is not None
        loops = scenario.loops if self.scenario_loops is None else self.scenario_loops
        self.rc_state.stop_motion()
        self.rc_loop.stop()
        detail = f"{scenario.name} loops={loops}"
        if scenario.description:
            detail = f"{detail} - {scenario.description}"
        record_event(self.store, self.logger, "Scenario start", detail)
        try:
            execute_scenario(
                self.controller,
                scenario,
                loops=loops,
                stop_event=self._stop_event,
                on_step=lambda loop, total, step: record_scenario_step(
                    self.store,
                    self.logger,
                    self.controller,
                    self.ui,
                    scenario,
                    loop,
                    total,
                    step,
                ),
            )
            update_tello_store(self.store, self.controller)
            record_event(self.store, self.logger, "Scenario complete", scenario.name)
            notify(self.ui, f"Scenario complete: {scenario.name} x{loops}")
        except DroneError as exc:
            record_event(self.store, self.logger, "Scenario stopped", str(exc), level="WARN")
            notify(self.ui, f"Scenario stopped: {exc}", level="WARN")
        finally:
            self.rc_state.stop_motion()
            if self.controller.connected and self.controller.airborne:
                try:
                    self.controller.rc_control(0, 0, 0, 0)
                except DroneError:
                    pass
            if self._land_requested.is_set():
                try:
                    notify(self.ui, "Landing now.")
                    self.controller.land()
                    record_command(self.store, self.logger, self.controller, "land now")
                except DroneError as exc:
                    record_event(self.store, self.logger, "Scenario landing failed", str(exc), level="WARN")
                    notify(self.ui, f"Landing failed: {exc}", level="WARN")
            if self._resume_rc_after_stop:
                self.rc_loop.start()


def record_scenario_step(
    store: TelemetryStore | None,
    logger: TelemetryLogger | None,
    controller: DroneController,
    ui: object | None,
    scenario: FlightScenario,
    loop: int,
    total: int,
    step: ScenarioStep,
) -> None:
    label = f"scenario {scenario.name} {loop}/{total}: {step.label}"
    notify(ui, label)
    record_command(store, logger, controller, label)


def update_tello_store(
    store: TelemetryStore | None,
    controller: DroneController,
    *,
    battery: int | None = None,
) -> None:
    if not store:
        return
    store.update_tello(
        connected=controller.connected,
        airborne=controller.airborne,
        battery=controller.battery if battery is None else battery,
        speed=controller.safety.speed,
    )


class BatteryPoller:
    def __init__(
        self,
        controller: DroneController,
        store: TelemetryStore | None,
        *,
        interval: float = BATTERY_REFRESH_SECONDS,
    ) -> None:
        self.controller = controller
        self.store = store
        self.interval = max(1.0, float(interval))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_error: str | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="battery-poller", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def poll_once(self) -> int | None:
        try:
            battery = self.controller.refresh_battery()
        except Exception as exc:  # noqa: BLE001
            self._record_error(exc)
            return None
        self._last_error = None
        update_tello_store(self.store, self.controller, battery=battery)
        return battery

    def _run(self) -> None:
        while not self._stop.wait(self.interval):
            if not self.controller.connected:
                continue
            self.poll_once()

    def _record_error(self, exc: Exception) -> None:
        message = str(exc)
        if message == self._last_error:
            return
        self._last_error = message
        if self.store:
            self.store.add_event(f"Battery refresh failed: {message}", level="WARN")


def record_command(
    store: TelemetryStore | None,
    logger: TelemetryLogger | None,
    controller: DroneController,
    command: str,
) -> None:
    if store:
        store.record_command(command)
        update_tello_store(store, controller)
        store.add_event(command.capitalize())
    if logger:
        logger.log_flight_event(command)


def record_event(
    store: TelemetryStore | None,
    logger: TelemetryLogger | None,
    event: str,
    detail: str = "",
    level: str = "INFO",
) -> None:
    if store:
        store.add_event(event if not detail else f"{event}: {detail}", level=level)
    if logger:
        logger.log_flight_event(event, detail)


class HitResponseCoordinator:
    def __init__(self, action: str = HIT_RESPONSE_FLIP_LAND) -> None:
        self.action = action
        self._lock = threading.Lock()
        self._triggered = False
        self._runtime: tuple[
            DroneController,
            RCControlState,
            RCControlLoop,
            ScenarioRunner,
            TelemetryStore | None,
            TelemetryLogger | None,
        ] | None = None

    def bind(
        self,
        controller: DroneController,
        rc_state: RCControlState,
        rc_loop: RCControlLoop,
        scenario_runner: ScenarioRunner,
        store: TelemetryStore | None,
        logger: TelemetryLogger | None,
    ) -> None:
        with self._lock:
            self._runtime = (controller, rc_state, rc_loop, scenario_runner, store, logger)

    def trigger(self, data: object | None = None) -> None:
        with self._lock:
            if self._triggered or self._runtime is None:
                return
            self._triggered = True
            runtime = self._runtime
        assert runtime is not None
        threading.Thread(target=self._run, args=(*runtime, data), daemon=True).start()

    def _run(
        self,
        controller: DroneController,
        rc_state: RCControlState,
        rc_loop: RCControlLoop,
        scenario_runner: ScenarioRunner,
        store: TelemetryStore | None,
        logger: TelemetryLogger | None,
        data: object | None,
    ) -> None:
        handle_hit_response(
            controller,
            rc_state,
            rc_loop,
            scenario_runner,
            store,
            logger,
            action=self.action,
            data=data,
        )


def handle_hit_response(
    controller: DroneController,
    rc_state: RCControlState,
    rc_loop: RCControlLoop,
    scenario_runner: ScenarioRunner,
    store: TelemetryStore | None,
    logger: TelemetryLogger | None,
    action: str = HIT_RESPONSE_FLIP_LAND,
    data: object | None = None,
) -> None:
    if not controller.connected or not controller.airborne:
        return

    if scenario_runner.is_running():
        scenario_runner.request_stop(resume_rc=False)

    rc_state.stop_motion()
    rc_loop.stop()

    detail = "emergency motor stop (free fall)" if action == HIT_RESPONSE_FREE_FALL else "flip forward then land"
    if data is not None:
        detail = f"{detail} ({getattr(data, 'state', 'hit')})"
    record_event(store, logger, "Jetson hit command received", detail, level="WARN")

    if action == HIT_RESPONSE_FREE_FALL:
        try:
            controller.emergency()
            record_command(store, logger, controller, "emergency free fall")
            controller.end()
        except DroneError as exc:
            record_event(store, logger, "Emergency free fall failed", str(exc), level="WARN")
        update_tello_store(store, controller)
        return

    try:
        controller.flip_forward()
        record_command(store, logger, controller, "flip forward")
    except DroneError as exc:
        record_event(store, logger, "Flip failed", str(exc), level="WARN")

    try:
        controller.land()
        record_command(store, logger, controller, "land now")
    except DroneError as exc:
        record_event(store, logger, "Landing failed", str(exc), level="WARN")

    update_tello_store(store, controller)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    controller: DroneController | None = None
    logger: TelemetryLogger | None = None
    hit_response = HitResponseCoordinator(args.hit_response) if args.auto_hit_response else None
    try:
        safety = make_safety(args)
        store = TelemetryStore()
        logger = TelemetryLogger(args.log_dir)

        with ExitStack() as stack:
            stack.callback(logger.close)
            if args.dashboard:
                from .dashboard import DashboardServer

                dashboard = DashboardServer(
                    store,
                    host=args.dashboard_host,
                    port=args.dashboard_port,
                )
                try:
                    dashboard.start()
                except OSError as exc:
                    raise ValueError(
                        f"Dashboard bind failed on {args.dashboard_host}:{args.dashboard_port}: {exc}. "
                        "Use --dashboard-port to choose another port."
                    ) from exc
                stack.callback(dashboard.stop)
                if args.open_browser:
                    from .dashboard import open_dashboard_browser

                    open_dashboard_browser(args.dashboard_host, dashboard.port, display=args.browser_display)
                from .dashboard import print_network_help

                print_network_help(args.dashboard_host, dashboard.port, args.telemetry_port)

            if args.fake_jetson:
                fake = FakeJetsonTelemetry(store, logger=logger)
                fake.start()
                stack.callback(fake.stop)
            elif args.dashboard or args.auto_hit_response:
                receiver = TelemetryReceiver(
                    store,
                    host=args.telemetry_host,
                    port=args.telemetry_port,
                    logger=logger,
                    on_hit=hit_response.trigger if hit_response else None,
                )
                receiver.start()
                stack.callback(receiver.stop)

            if args.dry_run:
                tello = DryRunTello()
            else:
                from djitellopy import Tello

                tello = Tello()

            controller = DroneController(tello, safety)
            try:
                return run_cli(
                    controller,
                    store,
                    logger,
                    rc_speed=args.rc_speed,
                    rc_yaw_speed=args.rc_yaw_speed,
                    rc_rate_hz=args.rc_rate_hz,
                    rc_hold_seconds=args.rc_hold_seconds,
                    control_ui=not args.no_control_ui,
                    scenario_loops=args.scenario_loops,
                    scenarios_enabled=not args.no_scenario,
                    hit_response=hit_response,
                )
            except BaseException:
                try:
                    controller.shutdown()
                    update_tello_store(store, controller)
                except Exception as shutdown_exc:  # noqa: BLE001
                    print(f"Shutdown warning: {shutdown_exc}", file=sys.stderr)
                raise
    except ImportError as exc:
        if exc.name in {"djitellopy", "fastapi", "uvicorn"}:
            print(
                f'Error: {exc.name} is not installed. Run: python -m pip install -e "."',
                file=sys.stderr,
            )
            return 1
        raise
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        if controller:
            try:
                controller.shutdown()
            except Exception as exc:  # noqa: BLE001
                print(f"Shutdown warning: {exc}", file=sys.stderr)
        return 130
    except (DroneError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        if controller:
            try:
                controller.shutdown()
            except Exception as shutdown_exc:  # noqa: BLE001
                print(f"Shutdown warning: {shutdown_exc}", file=sys.stderr)
        return 1


def resolve_scenario_path(value: str) -> Path:
    path = Path(value)
    if path.suffix or path.parent != Path("."):
        return path

    scenario_dir = Path("scenarios")
    direct = scenario_dir / f"{value}.json"
    if direct.exists():
        return direct

    matches = sorted(scenario_dir.glob(f"{value}_*.json"))
    if matches:
        return matches[0]
    return direct


if __name__ == "__main__":
    raise SystemExit(main())
