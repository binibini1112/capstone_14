from __future__ import annotations

import json
import threading
import tempfile
import unittest
from pathlib import Path

from tello_control.cli import resolve_scenario_path, scenario_selection_message
from tello_control.controller import DroneController, DroneError
from tello_control.dry_run import DryRunTello
from tello_control.scenario import execute_scenario, execute_step, load_scenario
from tello_control.scenario import ScenarioStep


class ScenarioTest(unittest.TestCase):
    def test_resolve_scenario_number_to_json_file(self) -> None:
        self.assertEqual(resolve_scenario_path("1"), Path("scenarios/1.json"))
        self.assertEqual(resolve_scenario_path("2"), Path("scenarios/2.json"))
        self.assertEqual(resolve_scenario_path("3"), Path("scenarios/3.json"))
        self.assertEqual(resolve_scenario_path("4"), Path("scenarios/4.json"))

    def test_scenario_selection_message_uses_json_descriptions(self) -> None:
        message = scenario_selection_message()

        self.assertIn("1: 1_rc_front_view_infinity", message)
        self.assertIn("2: 2_front_view_corner_pause", message)
        self.assertIn("3: 3_demo_rectangle_center_return", message)
        self.assertIn("4: 4_stage_demo_forward_side_climb", message)
        self.assertIn("Esc/q: cancel", message)

    def test_default_scenario_keeps_forward_back_axis_zero(self) -> None:
        scenario = load_scenario("scenarios/1.json")

        rc_steps = [step for step in scenario.steps if step.command == "rc"]

        self.assertTrue(rc_steps)
        self.assertTrue(all(step.params["forward_back"] == 0 for step in rc_steps))
        self.assertTrue(any(step.params["left_right"] != 0 for step in rc_steps))
        self.assertTrue(any(step.params["up_down"] != 0 for step in rc_steps))

    def test_load_and_execute_scenario_loops(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scenario.json"
            path.write_text(
                json.dumps(
                    {
                        "name": "test-pattern",
                        "loops": 2,
                        "steps": [
                            {"command": "down", "cm": 50},
                            {"command": "go", "x": 40, "y": 40, "z": 50, "speed": 30},
                            {"command": "wait", "seconds": 0},
                        ],
                    }
                ),
                encoding="utf-8",
            )

            scenario = load_scenario(path)
            tello = DryRunTello()
            controller = DroneController(tello)
            controller.connect()
            controller.takeoff()
            execute_scenario(controller, scenario)

            self.assertEqual(scenario.name, "test-pattern")
            self.assertEqual(tello.calls.count(("move_down", 50)), 2)
            self.assertEqual(tello.calls.count(("go_xyz_speed", (40, 40, 50, 30))), 2)

    def test_execute_rc_step_keeps_forward_back_zero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scenario.json"
            path.write_text(
                json.dumps(
                    {
                        "steps": [
                            {
                                "command": "rc",
                                "left_right": 22,
                                "forward_back": 0,
                                "up_down": 18,
                                "yaw": 0,
                                "seconds": 0.05,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            scenario = load_scenario(path)
            tello = DryRunTello()
            controller = DroneController(tello)
            controller.connect()
            controller.takeoff()
            execute_scenario(controller, scenario)

            self.assertEqual(tello.last_rc, (0, 0, 0, 0))
            self.assertIn(("send_rc_control", (22, 0, 18, 0)), tello.calls)

    def test_scenario_stop_event_aborts_before_next_step(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scenario.json"
            path.write_text(
                json.dumps(
                    {
                        "steps": [
                            {"command": "wait", "seconds": 0.1},
                            {"command": "up", "cm": 50},
                        ],
                    }
                ),
                encoding="utf-8",
            )

            scenario = load_scenario(path)
            tello = DryRunTello()
            controller = DroneController(tello)
            controller.connect()
            controller.takeoff()
            stop_event = threading.Event()
            stop_event.set()

            with self.assertRaisesRegex(DroneError, "stopped by operator"):
                execute_scenario(controller, scenario, stop_event=stop_event)

            self.assertNotIn(("move_up", 50), tello.calls)

    def test_scenario_two_has_paused_vertices_and_no_forward_rc(self) -> None:
        scenario = load_scenario("scenarios/2.json")
        movement = [step for step in scenario.steps if step.command != "wait"]
        waits = [step for step in scenario.steps if step.command == "wait"]

        self.assertEqual([step.command for step in movement], ["up", "right", "down", "left"])
        self.assertTrue(all(step.params["cm"] == 50 for step in movement))
        self.assertEqual(len(waits), 4)
        self.assertTrue(all(step.params["seconds"] == 1.0 for step in waits))

    def test_scenario_three_is_simple_square_without_forward_back(self) -> None:
        scenario = load_scenario("scenarios/3.json")
        movement = [step for step in scenario.steps if step.command != "wait"]
        waits = [step for step in scenario.steps if step.command == "wait"]

        self.assertEqual([step.command for step in movement], ["right", "forward", "left", "back", "right"])
        self.assertEqual([step.params["cm"] for step in movement], [50, 50, 100, 50, 50])
        self.assertEqual([step.params["seconds"] for step in waits], [1.0, 1.0, 1.0, 1.0, 3.0])

    def test_scenario_four_matches_stage_demo_path(self) -> None:
        scenario = load_scenario("scenarios/4.json")

        self.assertEqual(scenario.name, "4_stage_demo_forward_side_climb")
        self.assertEqual(
            [(step.command, step.params) for step in scenario.steps],
            [
                ("up", {"cm": 50}),
                ("forward", {"cm": 200}),
                ("left", {"cm": 200}),
                ("right", {"cm": 200}),
                ("up", {"cm": 100}),
                ("wait_until_stop", {"reason": "hit_detected"}),
            ],
        )

    def test_wait_until_stop_requires_stop_event(self) -> None:
        tello = DryRunTello()
        controller = DroneController(tello)
        controller.connect()
        controller.takeoff()

        with self.assertRaisesRegex(DroneError, "requires a stop event"):
            execute_step(controller, ScenarioStep("wait_until_stop", {}))

    def test_rejects_tiny_go_step(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.json"
            path.write_text(
                json.dumps(
                    {
                        "steps": [
                            {"command": "go", "x": 10, "y": 0, "z": 0, "speed": 30},
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(DroneError, "at least 20 cm"):
                load_scenario(path)


if __name__ == "__main__":
    unittest.main()
