"""Beacon-only onboard gate selection state."""

from __future__ import annotations

from dataclasses import dataclass, field

from underwater_racing.racing.beacon import BeaconMeasurement
from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


@dataclass(frozen=True)
class OnboardGateUpdate:
    switched: bool
    completed_gate_id: int | None = None
    from_gate_id: int | None = None
    to_gate_id: int | None = None
    reason: str = ""


@dataclass
class OnboardGateSelector:
    """Internal rover mission state driven only by beacon measurements."""

    track: RaceTrack
    distance_threshold_m: float = 0.9
    bearing_threshold_deg: float = 25.0
    vertical_threshold_m: float = 0.6
    consecutive_ticks_required: int = 3
    enable_range_trend: bool = False
    range_trend_ticks_required: int = 3
    range_trend_min_delta_m: float = 0.05
    active_index: int = 0
    completed_gate_ids: list[int] = field(default_factory=list)
    _close_streak: int = 0
    _range_increase_streak: int = 0
    _has_been_close_to_gate: bool = False
    _previous_distance_m: float | None = None

    def __post_init__(self) -> None:
        if self.distance_threshold_m <= 0.0:
            raise ValueError("distance_threshold_m must be positive")
        if self.bearing_threshold_deg <= 0.0:
            raise ValueError("bearing_threshold_deg must be positive")
        if self.vertical_threshold_m <= 0.0:
            raise ValueError("vertical_threshold_m must be positive")
        if self.consecutive_ticks_required <= 0:
            raise ValueError("consecutive_ticks_required must be positive")
        if self.range_trend_ticks_required <= 0:
            raise ValueError("range_trend_ticks_required must be positive")
        if self.range_trend_min_delta_m < 0.0:
            raise ValueError("range_trend_min_delta_m cannot be negative")

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

        if close_enough:
            self._close_streak += 1
            self._has_been_close_to_gate = True
        else:
            self._close_streak = 0

        if expected_measurement:
            self._update_range_trend(measurement.distance_m)
        else:
            self._reset_measurement_history()

        if self._close_streak >= self.consecutive_ticks_required:
            return self._advance("close_threshold")

        if (
            self.enable_range_trend
            and self._has_been_close_to_gate
            and self._range_increase_streak >= self.range_trend_ticks_required
        ):
            return self._advance("range_increasing_after_close")

        return OnboardGateUpdate(switched=False)

    def _update_range_trend(self, distance_m: float) -> None:
        if self._previous_distance_m is not None:
            increased = distance_m > self._previous_distance_m + self.range_trend_min_delta_m
            if increased and self._has_been_close_to_gate:
                self._range_increase_streak += 1
            else:
                self._range_increase_streak = 0
        self._previous_distance_m = distance_m

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
        self._range_increase_streak = 0
        self._has_been_close_to_gate = False
        self._previous_distance_m = None
