from __future__ import annotations

import json
import socket
import tempfile
import time
import unittest
import asyncio
import threading
from pathlib import Path
from unittest.mock import patch

from tello_control.dashboard import DashboardServer, dashboard_urls, print_network_help
from tello_control.dry_run import DryRunTello
from tello_control.logger import TelemetryLogger
from tello_control.network_profiles import SITE_NETWORK_PROFILES, format_site_network_profiles
from tello_control.telemetry_model import ErrorData, JetsonTrackingData, LaserData
from tello_control.telemetry_receiver import FakeJetsonTelemetry, TelemetryReceiver, parse_tracking_packet
from tello_control.telemetry_store import TelemetryStore
from tello_control.visualization import radar_to_canvas


class TelemetryModelTest(unittest.TestCase):
    def test_parse_valid_packet_with_optional_fields(self) -> None:
        packet = json.dumps(
            {
                "timestamp": 1715600000.123,
                "target_found": True,
                "state": "TRACKING",
                "fps": 24.8,
                "confidence": 0.87,
                "error": {"x_px": -170, "y_px": -110, "x_norm": -0.266, "y_norm": -0.306},
                "audio": {"enabled": True, "direction_deg": 35.0, "confidence": 0.62},
                "ultra_ps": {"motor_deg": 90.0, "front_pan": 2048, "pan_tick": 1024},
            }
        ).encode()

        data = parse_tracking_packet(packet)

        self.assertIsNotNone(data)
        assert data is not None
        self.assertEqual(data.state, "TRACKING")
        self.assertEqual(data.error.x_px, -170)
        self.assertEqual(data.audio.direction_deg, 35.0)
        self.assertEqual(data.ultra_ps.motor_deg, 90.0)
        self.assertEqual(data.ultra_ps.front_pan, 2048)
        self.assertEqual(data.ultra_ps.pan_tick, 1024)

    def test_parse_minimal_packet_missing_optional_fields(self) -> None:
        data = parse_tracking_packet(
            b'{"timestamp": 1715600000.123, "target_found": false, "state": "SCANNING"}'
        )

        self.assertIsNotNone(data)
        assert data is not None
        self.assertEqual(data.state, "SCANNING")
        self.assertIsNone(data.fps)
        self.assertIsNone(data.error)

    def test_bad_json_is_ignored(self) -> None:
        self.assertIsNone(parse_tracking_packet(b"{bad json"))
        self.assertIsNone(parse_tracking_packet(b'{"timestamp": 1, "state": "TRACKING"}'))


class TelemetryReceiverTest(unittest.TestCase):
    def test_udp_receiver_updates_store_and_ignores_bad_packets(self) -> None:
        port = free_udp_port()
        store = TelemetryStore()
        receiver = TelemetryReceiver(store, host="127.0.0.1", port=port)
        receiver.start()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(b"{bad json", ("127.0.0.1", port))
            sock.sendto(
                json.dumps(
                    {
                        "timestamp": time.time(),
                        "target_found": True,
                        "state": "TRACKING",
                        "fps": 25.0,
                    }
                ).encode(),
                ("127.0.0.1", port),
            )

            snapshot = wait_for_snapshot_with_tracking(store)
            self.assertEqual(snapshot.tracking.state, "TRACKING")
            self.assertEqual(snapshot.jetson_status, "CONNECTED")
        finally:
            sock.close()
            receiver.stop()

    def test_udp_receiver_triggers_hit_callback_once_on_rising_edge(self) -> None:
        port = free_udp_port()
        store = TelemetryStore()
        hit_event = threading.Event()
        hit_count = 0
        hit_lock = threading.Lock()

        def on_hit(data) -> None:
            nonlocal hit_count
            with hit_lock:
                hit_count += 1
            hit_event.set()

        receiver = TelemetryReceiver(store, host="127.0.0.1", port=port, on_hit=on_hit)
        receiver.start()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            base_packet = {
                "timestamp": time.time(),
                "target_found": True,
                "state": "LOCKED",
                "laser": {"armed": True, "hit_detected": False},
            }
            sock.sendto(json.dumps(base_packet).encode(), ("127.0.0.1", port))
            base_packet["laser"]["hit_detected"] = True
            sock.sendto(json.dumps(base_packet).encode(), ("127.0.0.1", port))
            self.assertTrue(hit_event.wait(2.0))
            sock.sendto(json.dumps(base_packet).encode(), ("127.0.0.1", port))
            time.sleep(0.2)
            self.assertEqual(hit_count, 1)
        finally:
            sock.close()
            receiver.stop()


class TelemetryStoreTest(unittest.TestCase):
    def test_timeout_status_changes_from_connected_to_stale_to_disconnected(self) -> None:
        store = TelemetryStore()
        data = JetsonTrackingData(timestamp=1.0, target_found=True, state="TRACKING")
        store.update_tracking(data, now=100.0)

        self.assertEqual(store.snapshot(now=100.5).jetson_status, "CONNECTED")
        self.assertEqual(store.snapshot(now=102.0).jetson_status, "STALE")
        self.assertEqual(store.snapshot(now=104.0).jetson_status, "DISCONNECTED")

    def test_snapshot_combines_tello_and_jetson_state(self) -> None:
        store = TelemetryStore()
        store.update_tello(connected=True, airborne=True, battery=76, speed=20)
        store.record_command("takeoff", now=10.0)
        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.0,
                target_found=True,
                state="LOCKED",
                error=ErrorData(x_px=-5, y_px=8, x_norm=-0.1, y_norm=0.2),
            ),
            now=10.1,
        )

        snapshot = store.snapshot(now=10.2).to_dict()

        self.assertTrue(snapshot["tello"]["connected"])
        self.assertEqual(snapshot["tello"]["battery"], 76)
        self.assertEqual(snapshot["tracking"]["state"], "LOCKED")
        self.assertEqual(snapshot["history"][-1]["x_px"], -5)
        self.assertEqual(snapshot["history"][-1]["telemetry_rate_hz"], 1.0)

    def test_shots_track_laser_fire_events_not_armed_or_hits(self) -> None:
        store = TelemetryStore()

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.0,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, hit_detected=False),
            ),
            now=10.0,
        )
        self.assertEqual(store.snapshot(now=10.1).shot_count, 0)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.1,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, hit_detected=True),
            ),
            now=10.2,
        )
        snapshot = store.snapshot(now=10.3)
        self.assertEqual(snapshot.shot_count, 0)
        self.assertEqual(snapshot.hit_count, 1)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.2,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=True, hit_detected=True),
            ),
            now=10.4,
        )
        self.assertEqual(store.snapshot(now=10.5).shot_count, 1)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.3,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=True, hit_detected=True),
            ),
            now=10.6,
        )
        self.assertEqual(store.snapshot(now=10.7).shot_count, 1)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.4,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, hit_detected=True),
            ),
            now=10.8,
        )
        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.5,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=True, hit_detected=True),
            ),
            now=11.0,
        )
        self.assertEqual(store.snapshot(now=11.1).shot_count, 2)

    def test_shots_follow_jetson_shot_count_delta_for_current_session(self) -> None:
        store = TelemetryStore()

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.0,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, shot_count=3, hit_detected=False),
            ),
            now=10.0,
        )
        self.assertEqual(store.snapshot(now=10.1).shot_count, 0)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.1,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, shot_count=5, hit_detected=False),
            ),
            now=10.2,
        )
        self.assertEqual(store.snapshot(now=10.3).shot_count, 2)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.2,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, shot_count=1, hit_detected=False),
            ),
            now=10.4,
        )
        self.assertEqual(store.snapshot(now=10.5).shot_count, 2)

        store.update_tracking(
            JetsonTrackingData(
                timestamp=1.3,
                target_found=True,
                state="LOCKED",
                laser=LaserData(armed=True, fired=False, shot_count=2, hit_detected=False),
            ),
            now=10.6,
        )
        self.assertEqual(store.snapshot(now=10.7).shot_count, 3)


class VisualizationTest(unittest.TestCase):
    def test_radar_to_canvas_maps_center_and_clamps(self) -> None:
        self.assertEqual(radar_to_canvas(0, 0, 200, 200, padding=0), (100.0, 100.0))
        self.assertEqual(radar_to_canvas(2, -2, 200, 200, padding=0), (200.0, 0.0))


class DashboardNetworkHelpTest(unittest.TestCase):
    def test_dashboard_urls_include_lan_addresses_when_bound_to_all_interfaces(self) -> None:
        with patch("tello_control.dashboard.local_ipv4_addresses", return_value=["192.168.0.7", "10.69.62.209"]):
            self.assertEqual(
                dashboard_urls("0.0.0.0", 8000),
                [
                    "http://127.0.0.1:8000",
                    "http://192.168.0.7:8000",
                    "http://10.69.62.209:8000",
                ],
            )

    def test_site_network_profiles_include_known_static_iptime_settings(self) -> None:
        self.assertEqual(SITE_NETWORK_PROFILES[0].name, "220호")
        self.assertEqual(SITE_NETWORK_PROFILES[0].ip_address, "113.198.84.249")
        self.assertEqual(SITE_NETWORK_PROFILES[0].secondary_dns, "209.248.252.2")
        self.assertEqual(SITE_NETWORK_PROFILES[1].name, "시현장")
        self.assertEqual(SITE_NETWORK_PROFILES[1].gateway, "223.194.146.254")
        self.assertEqual(SITE_NETWORK_PROFILES[1].secondary_dns, "203.248.252.2")

    def test_print_network_help_includes_site_network_profiles(self) -> None:
        with patch("tello_control.dashboard.local_ipv4_addresses", return_value=[]):
            with patch("builtins.print") as mock_print:
                print_network_help("0.0.0.0", 8000, 5005)

        output = "\n".join(str(call.args[0]) for call in mock_print.call_args_list)
        self.assertIn("Known ipTIME static WAN profiles:", output)
        for line in format_site_network_profiles():
            self.assertIn(line, output)


class LoggerTest(unittest.TestCase):
    def test_log_files_are_created(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            logger = TelemetryLogger(tmp)
            logger.log_flight_event("Takeoff")
            logger.log_tracking(JetsonTrackingData(timestamp=1.0, target_found=True, state="TRACKING"))
            flight_path = logger.flight_path
            tracking_path = logger.tracking_path
            logger.close()

            self.assertTrue(flight_path.exists())
            self.assertTrue(tracking_path.exists())
            self.assertIn("Takeoff", flight_path.read_text(encoding="utf-8"))
            self.assertIn('"state":"TRACKING"', tracking_path.read_text(encoding="utf-8"))


class IntegrationTest(unittest.TestCase):
    def test_fake_tello_fake_jetson_and_dashboard_can_run_together(self) -> None:
        try:
            import fastapi  # noqa: F401
            import uvicorn  # noqa: F401
        except ImportError:
            self.skipTest("fastapi/uvicorn are not installed")

        store = TelemetryStore()
        tello = DryRunTello()
        fake = FakeJetsonTelemetry(store, interval=0.01)
        dashboard = DashboardServer(store, host="127.0.0.1", port=0)
        fake.start()
        dashboard.start()
        try:
            tello.connect()
            store.update_tello(connected=True, airborne=False, battery=tello.get_battery(), speed=20)
            snapshot = wait_for_snapshot_with_tracking(store)
            self.assertTrue(snapshot.tello.connected)
            self.assertIsNotNone(snapshot.tracking)
        finally:
            fake.stop()
            dashboard.stop()

    def test_dashboard_websocket_accepts_clients(self) -> None:
        try:
            import fastapi  # noqa: F401
            import uvicorn  # noqa: F401
            import wsproto  # noqa: F401
            import websockets
        except ImportError:
            self.skipTest("fastapi/uvicorn/wsproto/websockets are not installed")

        store = TelemetryStore()
        port = free_tcp_port()
        dashboard = DashboardServer(store, host="127.0.0.1", port=port)
        dashboard.start()
        try:
            message = asyncio.run(read_one_websocket_message(websockets, port))
            payload = json.loads(message)
            self.assertIn("tello", payload)
            self.assertIn("jetson_status", payload)
        finally:
            dashboard.stop()


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def free_tcp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


async def read_one_websocket_message(websockets_module, port: int) -> str:
    uri = f"ws://127.0.0.1:{port}/ws"
    deadline = time.monotonic() + 2.0
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            async with websockets_module.connect(uri) as websocket:
                return await websocket.recv()
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            await asyncio.sleep(0.05)
    raise AssertionError(f"websocket did not connect: {last_error}")


def wait_for(predicate, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.02)
    raise AssertionError("condition was not met before timeout")


def wait_for_snapshot_with_tracking(store: TelemetryStore):
    return wait_for(lambda: _snapshot_if_tracking(store))


def _snapshot_if_tracking(store: TelemetryStore):
    snapshot = store.snapshot()
    if snapshot.tracking is None:
        return None
    return snapshot


if __name__ == "__main__":
    unittest.main()
