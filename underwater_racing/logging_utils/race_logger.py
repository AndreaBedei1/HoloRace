"""File logger for demo trajectories, events, and summaries."""

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


class RaceLogger:
    trajectory_fields = [
        "time",
        "x",
        "y",
        "z",
        "yaw_deg",
        "onboard_active_gate_id",
        "referee_active_gate_id",
        "distance_to_gate",
        "bearing_error",
        "vertical_error",
        "command_surge",
        "command_sway",
        "command_heave",
        "command_yaw",
    ]

    event_fields = ["time", "event", "gate_id", "details"]

    def __init__(self, root: str | Path = "outputs/single_gate_demo") -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = Path(root) / timestamp
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._trajectory_file = (self.output_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._events_file = (self.output_dir / "race_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._trajectory = csv.DictWriter(self._trajectory_file, fieldnames=self.trajectory_fields)
        self._events = csv.DictWriter(self._events_file, fieldnames=self.event_fields)
        self._trajectory.writeheader()
        self._events.writeheader()

    def log_trajectory(self, **row: Any) -> None:
        self._trajectory.writerow({field: row.get(field, "") for field in self.trajectory_fields})

    def log_event(self, time_s: float, event: str, gate_id: int | None, details: str = "") -> None:
        self._events.writerow(
            {
                "time": f"{time_s:.3f}",
                "event": event,
                "gate_id": "" if gate_id is None else gate_id,
                "details": details,
            }
        )

    def write_summary(self, summary: Dict[str, Any]) -> None:
        with (self.output_dir / "summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)

    def close(self) -> None:
        self._trajectory_file.close()
        self._events_file.close()

    def __enter__(self) -> "RaceLogger":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()
