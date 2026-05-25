"""FastAPI dashboard for Tello status and Jetson telemetry."""

import argparse
import asyncio
import os
import socket
import subprocess
import threading
import webbrowser
from pathlib import Path
from typing import Any

from .logger import TelemetryLogger
from .network_profiles import format_site_network_profiles
from .telemetry_receiver import FakeJetsonTelemetry, TelemetryReceiver
from .telemetry_store import TelemetryStore


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tello telemetry dashboard.")
    parser.add_argument("--host", default="0.0.0.0", help="Dashboard bind host.")
    parser.add_argument("--port", type=int, default=8000, help="Dashboard HTTP port.")
    parser.add_argument("--telemetry-host", default="0.0.0.0", help="Jetson UDP bind host.")
    parser.add_argument("--telemetry-port", type=int, default=5005, help="Jetson UDP port.")
    parser.add_argument("--fake-jetson", action="store_true", help="Generate fake tracking telemetry.")
    parser.add_argument("--log-dir", default="logs", help="Directory for flight/tracking logs.")
    parser.add_argument("--open-browser", action="store_true", help="Open the dashboard in the local browser.")
    parser.add_argument("--browser-display", default=":0", help="Display used when opening a browser from SSH.")
    return parser


def create_app(store: TelemetryStore) -> Any:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse
    from fastapi.staticfiles import StaticFiles

    web_dir = Path(__file__).with_name("web")
    ws_interval = max(0.02, float(os.getenv("DASHBOARD_WS_INTERVAL_SEC", "0.033")))
    ws_history = max(20, int(os.getenv("DASHBOARD_WS_HISTORY", "160")))
    ws_events = max(12, int(os.getenv("DASHBOARD_WS_EVENTS", "60")))
    app = FastAPI(title="Tello Dashboard")
    app.mount("/static", StaticFiles(directory=web_dir), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(web_dir / "index.html")

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return store.snapshot().to_dict()

    @app.websocket("/ws")
    async def websocket(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                payload = store.snapshot().to_dict()
                payload["history"] = payload.get("history", [])[-ws_history:]
                payload["events"] = payload.get("events", [])[-ws_events:]
                await websocket.send_json(payload)
                await asyncio.sleep(ws_interval)
        except WebSocketDisconnect:
            return

    return app


class DashboardServer:
    """Runs uvicorn in a daemon thread so the control loop stays independent."""

    def __init__(self, store: TelemetryStore, host: str = "0.0.0.0", port: int = 8000) -> None:
        self.store = store
        self.host = host
        self.port = port
        self._thread: threading.Thread | None = None
        self._server: Any = None
        self._app: Any = None
        self._sockets: list[socket.socket] | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        import uvicorn  # noqa: F401

        self._app = create_app(self.store)
        sock = self._bind_socket()
        self.port = int(sock.getsockname()[1])
        self._sockets = [sock]
        self._thread = threading.Thread(target=self._run, name="dashboard-server", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout)
        if self._thread is None or not self._thread.is_alive():
            self._close_sockets()

    def _bind_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.host, self.port))
            sock.listen(2048)
        except OSError:
            sock.close()
            raise
        return sock

    def _close_sockets(self) -> None:
        sockets = self._sockets
        self._sockets = None
        if not sockets:
            return
        for sock in sockets:
            try:
                sock.close()
            except OSError:
                pass

    def _run(self) -> None:
        import uvicorn

        config = uvicorn.Config(
            self._app,
            host=self.host,
            port=self.port,
            log_level="warning",
            access_log=False,
            ws="wsproto",
        )
        self._server = uvicorn.Server(config)
        self.store.add_event(f"Dashboard listening on http://{self.host}:{self.port}")
        try:
            self._server.run(sockets=self._sockets)
        finally:
            self._close_sockets()


def dashboard_url(host: str, port: int) -> str:
    browser_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    return f"http://{browser_host}:{port}"


def local_ipv4_addresses() -> list[str]:
    """Return likely LAN IPv4 addresses for other devices to reach this host."""
    addresses: list[str] = []

    def add(address: str) -> None:
        if not address or address.startswith("127.") or address.startswith("169.254."):
            return
        if address not in addresses:
            addresses.append(address)

    for target in ("8.8.8.8", "1.1.1.1"):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((target, 80))
            add(str(sock.getsockname()[0]))
        except OSError:
            pass
        finally:
            sock.close()

    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            add(str(info[4][0]))
    except OSError:
        pass

    try:
        output = subprocess.check_output(["hostname", "-I"], text=True, timeout=1.0)
    except (OSError, subprocess.SubprocessError):
        output = ""
    for token in output.split():
        if token.count(".") == 3:
            add(token)

    return addresses


def dashboard_urls(host: str, port: int) -> list[str]:
    urls = [dashboard_url(host, port)]
    if host in {"0.0.0.0", "::"}:
        for address in local_ipv4_addresses():
            urls.append(f"http://{address}:{port}")
    return urls


def print_network_help(dashboard_host: str, dashboard_port: int, telemetry_port: int) -> None:
    urls = dashboard_urls(dashboard_host, dashboard_port)
    print("Dashboard URLs:")
    for url in urls:
        print(f"  {url}")

    addresses = local_ipv4_addresses()
    if addresses:
        print("Jetson UDP target candidates:")
        for address in addresses:
            print(f"  {address}:{telemetry_port}")
    else:
        print(f"Jetson UDP target: <this-device-ip>:{telemetry_port}")

    print("Known ipTIME static WAN profiles:")
    for line in format_site_network_profiles():
        print(line)
    print("After moving sites, reset ipTIME and enter the matching profile in ipTIME setup assistant.")


def open_dashboard_browser(host: str, port: int, delay: float = 0.8, display: str = ":0") -> None:
    url = dashboard_url(host, port)

    def _open() -> None:
        env = browser_environment(display)
        try:
            subprocess.Popen(
                ["xdg-open", url],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
                start_new_session=True,
            )
            return
        except FileNotFoundError:
            pass
        except Exception as exc:  # noqa: BLE001
            print(f"Browser open warning: {exc}")

        old_display = os.environ.get("DISPLAY")
        old_runtime = os.environ.get("XDG_RUNTIME_DIR")
        old_xauthority = os.environ.get("XAUTHORITY")
        os.environ.update(env)
        try:
            ok = webbrowser.open(url)
        except Exception as exc:  # noqa: BLE001
            print(f"Browser open failed: {exc}")
            ok = False
        finally:
            restore_environment("DISPLAY", old_display)
            restore_environment("XDG_RUNTIME_DIR", old_runtime)
            restore_environment("XAUTHORITY", old_xauthority)
        if not ok:
            print(f"Browser open failed. Open manually on the Raspberry Pi monitor: {url}")

    timer = threading.Timer(delay, _open)
    timer.daemon = True
    timer.start()


def browser_environment(display: str = ":0") -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("DISPLAY", display)
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    xauthority = Path.home() / ".Xauthority"
    if xauthority.exists():
        env.setdefault("XAUTHORITY", str(xauthority))
    return env


def restore_environment(name: str, value: str | None) -> None:
    if value is None:
        os.environ.pop(name, None)
    else:
        os.environ[name] = value


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    store = TelemetryStore()
    logger = TelemetryLogger(args.log_dir)
    receiver: TelemetryReceiver | None = None
    fake: FakeJetsonTelemetry | None = None

    try:
        if args.fake_jetson:
            fake = FakeJetsonTelemetry(store, logger=logger)
            fake.start()
        else:
            receiver = TelemetryReceiver(
                store,
                host=args.telemetry_host,
                port=args.telemetry_port,
                logger=logger,
            )
            receiver.start()

        import uvicorn

        if args.open_browser:
            open_dashboard_browser(args.host, args.port, display=args.browser_display)

        print_network_help(args.host, args.port, args.telemetry_port)

        uvicorn.run(
            create_app(store),
            host=args.host,
            port=args.port,
            log_level="info",
            access_log=False,
            ws="wsproto",
        )
        return 0
    finally:
        if receiver:
            receiver.stop()
        if fake:
            fake.stop()
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
