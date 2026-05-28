"""Beacon-only onboard gate selection state."""

from __future__ import annotations

from dataclasses import dataclass, field

from underwater_racing.racing.beacon import BeaconMeasurement
from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


@dataclass(frozen=True)
class OnboardGateUpdate:
    switched: bool
    gate_reached: bool = False
    reached_gate_id: int | None = None
    completed_gate_id: int | None = None
    from_gate_id: int | None = None
    to_gate_id: int | None = None
    reason: str = ""


@dataclass
class OnboardGateSelector:
    """Internal rover mission state driven only by beacon measurements."""

    track: RaceTrack
    distance_threshold_m: float = 0.8
    bearing_threshold_deg: float = 25.0
    vertical_threshold_m: float = 0.6
    consecutive_ticks_required: int = 3
    clearance_margin_m: float = 0.4
    active_index: int = 0
    completed_gate_ids: list[int] = field(default_factory=list)
    gate_reached: bool = False
    min_distance_after_reached_m: float | None = None
    _close_streak: int = 0

    def __post_init__(self) -> None:
        if self.distance_threshold_m <= 0.0:
            raise ValueError("distance_threshold_m must be positive")
        if self.bearing_threshold_deg <= 0.0:
            raise ValueError("bearing_threshold_deg must be positive")
        if self.vertical_threshold_m <= 0.0:
            raise ValueError("vertical_threshold_m must be positive")
        if self.consecutive_ticks_required <= 0:
            raise ValueError("consecutive_ticks_required must be positive")
        if self.clearance_margin_m <= 0.0:
            raise ValueError("clearance_margin_m must be positive")

    @property
    def is_finished(self) -> bool:
        return self.active_index >= len(self.track.gates)

    @property
    def active_gate(self) -> RaceGate | None:
        if self.is_finished:
            return None
        return self.track.gates[self.active_index]

    @property
    def active_gate_id(self) -> int | None:
        gate = self.active_gate
        return gate.id if gate is not None else None

    def update_from_measurement(self, measurement: BeaconMeasurement) -> OnboardGateUpdate:
        gate = self.active_gate
        if gate is None:
            return OnboardGateUpdate(switched=False)

        expected_measurement = measurement.active_gate_id == gate.id
        close_enough = (
            expected_measurement
            and measurement.distance_m < self.distance_threshold_m
            and abs(measurement.bearing_error_deg) < self.bearing_threshold_deg
            and abs(measurement.vertical_error_m) < self.vertical_threshold_m
        )

        if not expected_measurement:
            self._reset_measurement_history()
            return OnboardGateUpdate(switched=False)

        if not self.gate_reached:
            if close_enough:
                self._close_streak += 1
            else:
                self._close_streak = 0

            if self._close_streak >= self.consecutive_ticks_required:
                self.gate_reached = True
                self.min_distance_after_reached_m = measurement.distance_m
                return OnboardGateUpdate(
                    switched=False,
                    gate_reached=True,
                    reached_gate_id=gate.id,
                    reason="close_threshold",
                )

            return OnboardGateUpdate(switched=False)

        if self.min_distance_after_reached_m is None:
            self.min_distance_after_reached_m = measurement.distance_m
        else:
            self.min_distance_after_reached_m = min(
                self.min_distance_after_reached_m,
                measurement.distance_m,
            )

        if measurement.distance_m > self.min_distance_after_reached_m + self.clearance_margin_m:
            return self._advance("range_increased_after_reach")

        return OnboardGateUpdate(switched=False)

    def _advance(self, reason: str) -> OnboardGateUpdate:
        gate = self.active_gate
        if gate is None:
            return OnboardGateUpdate(switched=False)

        completed_gate_id = gate.id
        self.completed_gate_ids.append(completed_gate_id)
        self.active_index += 1
        next_gate_id = self.active_gate_id
        self._reset_measurement_history()

        return OnboardGateUpdate(
            switched=True,
            completed_gate_id=completed_gate_id,
            from_gate_id=completed_gate_id,
            to_gate_id=next_gate_id,
            reason=reason,
        )

    def _reset_measurement_history(self) -> None:
        self._close_streak = 0
        self.gate_reached = False
        self.min_distance_after_reached_m = None
