from __future__ import annotations

import unittest

from tello_control.cli import BatteryPoller, build_parser, command_label, handle_hit_response, handle_key
from tello_control.controller import DroneController, DroneError
from tello_control.rc_control import RCControlLoop
from tello_control.rc_control import RCControlState
from tello_control.safety import SafetyConfig
from tello_control.telemetry_store import TelemetryStore


class FakeTello:
    def __init__(self, battery: int = 80) -> None:
        self.battery = battery
        self.calls: list[tuple[str, int | tuple[int, int, int, int] | None]] = []

    def connect(self) -> None:
        self.calls.append(("connect", None))

    def get_battery(self) -> int:
        self.calls.append(("get_battery", None))
        return self.battery

    def set_speed(self, speed: int) -> None:
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

    def go_xyz_speed(self, x: int, y: int, z: int, speed: int) -> None:
        self.calls.append(("go_xyz_speed", (x, y, z, speed)))


class ConnectFailTello(FakeTello):
    def connect(self) -> None:
        self.calls.append(("connect", None))
        raise RuntimeError("no response")


class DummyScenarioRunner:
    def __init__(self, running: bool = False) -> None:
        self.running = running
        self.requested_resume_rc: bool | None = None

    def is_running(self) -> bool:
        return self.running

    def request_stop(self, *, resume_rc: bool = True) -> None:
        self.requested_resume_rc = resume_rc


class DroneControllerTest(unittest.TestCase):
    def test_auto_hit_response_is_enabled_by_default(self) -> None:
        parser = build_parser()

        self.assertTrue(parser.parse_args([]).auto_hit_response)
        self.assertFalse(parser.parse_args(["--no-auto-hit-response"]).auto_hit_response)
        self.assertTrue(parser.parse_args(["--auto-hit-response"]).auto_hit_response)
        self.assertEqual(parser.parse_args([]).hit_response, "flip-land")
        self.assertEqual(parser.parse_args(["--hit-response", "free-fall"]).hit_response, "free-fall")

    def test_connect_checks_battery_and_sets_speed(self) -> None:
        fake = FakeTello(battery=70)
        controller = DroneController(fake, SafetyConfig(speed=30))

        battery = controller.connect()

        self.assertEqual(battery, 70)
        self.assertTrue(controller.connected)
        self.assertEqual(
            fake.calls,
            [
                ("connect", None),
                ("get_battery", None),
                ("set_speed", 30),
            ],
        )

    def test_connect_blocks_low_battery_and_ends_connection(self) -> None:
        fake = FakeTello(battery=20)
        controller = DroneController(fake, SafetyConfig(min_battery=25))

        with self.assertRaisesRegex(DroneError, "Battery too low"):
            controller.connect()

        self.assertFalse(controller.connected)
        self.assertEqual(fake.calls[-1], ("end", None))

    def test_connect_failure_reports_tello_network_hint(self) -> None:
        fake = ConnectFailTello()
        controller = DroneController(fake)

        with self.assertRaisesRegex(DroneError, "connected to the Tello Wi-Fi"):
            controller.connect()

        self.assertFalse(controller.connected)
        self.assertEqual(fake.calls, [("connect", None)])

    def test_battery_poller_refreshes_controller_and_store(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake)
        store = TelemetryStore()

        controller.connect()
        store.update_tello(connected=True, airborne=False, battery=80, speed=20)
        fake.battery = 75

        refreshed = BatteryPoller(controller, store).poll_once()

        self.assertEqual(refreshed, 75)
        self.assertEqual(controller.battery, 75)
        self.assertEqual(store.snapshot().tello.battery, 75)

    def test_movement_requires_takeoff(self) -> None:
        fake = FakeTello()
        controller = DroneController(fake)
        controller.connect()

        with self.assertRaisesRegex(DroneError, "airborne"):
            controller.move_forward()

        self.assertNotIn(("move_forward", 30), fake.calls)

    def test_takeoff_move_land_sequence(self) -> None:
        fake = FakeTello()
        controller = DroneController(fake, SafetyConfig(move_distance=40, rotation_degrees=45))

        controller.connect()
        controller.takeoff()
        controller.move_forward()
        controller.rotate_right()
        controller.land()

        self.assertIn(("takeoff", None), fake.calls)
        self.assertIn(("move_forward", 40), fake.calls)
        self.assertIn(("rotate_clockwise", 45), fake.calls)
        self.assertEqual(fake.calls[-1], ("land", None))
        self.assertFalse(controller.airborne)

    def test_rc_control_clamps_values(self) -> None:
        fake = FakeTello()
        controller = DroneController(fake)

        controller.connect()
        controller.takeoff()
        controller.rc_control(150, -120, 50, -40)

        self.assertEqual(fake.calls[-1], ("send_rc_control", (100, -100, 50, -40)))

    def test_go_xyz_speed_clamps_values(self) -> None:
        fake = FakeTello()
        controller = DroneController(fake)

        controller.connect()
        controller.takeoff()
        controller.go_xyz_speed(600, -600, 50, 120)

        self.assertEqual(fake.calls[-1], ("go_xyz_speed", (500, -500, 50, 100)))

    def test_flip_requires_airborne_and_minimum_battery(self) -> None:
        fake = FakeTello(battery=49)
        controller = DroneController(fake, SafetyConfig(min_flip_battery=50))

        controller.connect()
        controller.takeoff()

        with self.assertRaisesRegex(DroneError, "Battery too low for flip"):
            controller.flip_forward()

        self.assertNotIn(("flip_forward", None), fake.calls)

    def test_flip_calls_tello_direction_method(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake, SafetyConfig(min_flip_battery=50))

        controller.connect()
        controller.takeoff()
        controller.flip_left()

        self.assertEqual(fake.calls[-1], ("flip_left", None))

    def test_hit_response_flips_then_lands(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake, SafetyConfig(min_flip_battery=50))
        rc_state = RCControlState()
        rc_loop = RCControlLoop(controller, rc_state)
        scenario_runner = DummyScenarioRunner()

        controller.connect()
        controller.takeoff()
        handle_hit_response(controller, rc_state, rc_loop, scenario_runner, None, None)

        self.assertIn(("flip_forward", None), fake.calls)
        self.assertEqual(fake.calls[-1], ("land", None))
        self.assertFalse(controller.airborne)

    def test_hit_response_free_fall_uses_emergency_and_ends_connection(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake, SafetyConfig(min_flip_battery=50))
        rc_state = RCControlState()
        rc_loop = RCControlLoop(controller, rc_state)
        scenario_runner = DummyScenarioRunner()

        controller.connect()
        controller.takeoff()
        handle_hit_response(
            controller,
            rc_state,
            rc_loop,
            scenario_runner,
            None,
            None,
            action="free-fall",
        )

        self.assertIn(("emergency", None), fake.calls)
        self.assertNotIn(("flip_forward", None), fake.calls)
        self.assertNotIn(("land", None), fake.calls)
        self.assertEqual(fake.calls[-1], ("end", None))
        self.assertFalse(controller.airborne)
        self.assertFalse(controller.connected)

    def test_hit_response_stops_running_scenario_without_resuming_rc(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake, SafetyConfig(min_flip_battery=50))
        rc_state = RCControlState()
        rc_loop = RCControlLoop(controller, rc_state)
        scenario_runner = DummyScenarioRunner(running=True)

        controller.connect()
        controller.takeoff()
        handle_hit_response(controller, rc_state, rc_loop, scenario_runner, None, None)

        self.assertFalse(scenario_runner.requested_resume_rc)
        self.assertFalse(controller.airborne)

    def test_cli_maps_number_keys_to_flips(self) -> None:
        fake = FakeTello(battery=80)
        controller = DroneController(fake)
        rc_state = RCControlState()

        controller.connect()
        controller.takeoff()
        handled = handle_key("2", controller, rc_state, rc_speed=35, rc_yaw_speed=55)

        self.assertTrue(handled)
        self.assertEqual(command_label("2"), "flip forward")
        self.assertEqual(fake.calls[-1], ("flip_forward", None))

    def test_shutdown_lands_before_ending_when_airborne(self) -> None:
        fake = FakeTello()
        controller = DroneController(fake)

        controller.connect()
        controller.takeoff()
        controller.shutdown()

        self.assertEqual(fake.calls[-2:], [("land", None), ("end", None)])
        self.assertFalse(controller.connected)
        self.assertFalse(controller.airborne)

    def test_safety_config_rejects_out_of_range_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "move_distance"):
            SafetyConfig(move_distance=10).validate()


if __name__ == "__main__":
    unittest.main()
