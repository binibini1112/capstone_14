from __future__ import annotations

import os
import unittest

from tello_control.controller import DroneController
from tello_control.dry_run import DryRunTello
from tello_control.pilot_ui import render_pilot_ui
from tello_control.rc_control import RCControlState
from tello_control.telemetry_store import TelemetryStore


class PilotUITest(unittest.TestCase):
    def test_render_includes_control_status_and_flip_keys(self) -> None:
        store = TelemetryStore()
        controller = DroneController(DryRunTello())
        rc_state = RCControlState()

        battery = controller.connect()
        store.update_tello(connected=True, airborne=False, battery=battery, speed=20)
        store.record_command("takeoff", now=10.0)
        rc_state.pulse("fb", 35, now=10.0)

        output = render_pilot_ui(
            controller,
            store,
            rc_state,
            message="Keyboard control armed",
            message_level="INFO",
            size=os.terminal_size((100, 32)),
        )

        self.assertIn("Tello Pilot", output)
        self.assertIn("battery=85%", output)
        self.assertIn("2 flip forward", output)
        self.assertIn("Scenario 1  RC front-view infinity", output)
        self.assertIn(
            "Scenario 4  Stage demo: up 50cm, forward/left/right 200cm, up 100cm, hover until hit",
            output,
        )
        self.assertIn("Last command  takeoff", output)


if __name__ == "__main__":
    unittest.main()
