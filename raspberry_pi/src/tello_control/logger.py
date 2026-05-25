"""CSV and JSONL logging for flight events and tracking telemetry."""

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from threading import Lock

from .telemetry_model import JetsonTrackingData


class TelemetryLogger:
    """Writes flight events and tracking samples to timestamped files."""

    def __init__(self, log_dir: str | Path = "logs") -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.flight_path = self.log_dir / f"flight_{stamp}.csv"
        self.tracking_path = self.log_dir / f"tracking_{stamp}.jsonl"
        self._lock = Lock()
        self._flight_file = self.flight_path.open("w", newline="", encoding="utf-8")
        self._tracking_file = self.tracking_path.open("w", encoding="utf-8")
        self._flight_writer = csv.writer(self._flight_file)
        self._flight_writer.writerow(["timestamp", "event", "detail"])
        self._flight_file.flush()

    def log_flight_event(self, event: str, detail: str = "") -> None:
        with self._lock:
            self._flight_writer.writerow([datetime.now().isoformat(timespec="milliseconds"), event, detail])
            self._flight_file.flush()

    def log_tracking(self, data: JetsonTrackingData) -> None:
        with self._lock:
            self._tracking_file.write(json.dumps(data.to_dict(), separators=(",", ":")) + "\n")
            self._tracking_file.flush()

    def close(self) -> None:
        with self._lock:
            self._flight_file.flush()
            self._tracking_file.flush()
            self._flight_file.close()
            self._tracking_file.close()

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()
