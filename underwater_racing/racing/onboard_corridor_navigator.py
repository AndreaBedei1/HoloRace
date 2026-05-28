"""Onboard gate corridor navigation state driven by guidance measurements."""

from __future__ import annotations

from dataclasses import dataclass, field

from underwater_racing.racing.beacon import BeaconMeasurement, GuidanceTarget
from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.gate_corridor import GateCorridor, build_gate_corridor
from underwater_racing.racing.track import RaceTrack

APPROACH = "APPROACH"
TRANSIT = "TRANSIT"
EXIT = "EXIT"


@dataclass(frozen=True)
class OnboardNavigationUpdate:
    phase_changed: bool = False
    from_phase: str | None = None
    to_phase: str | None = None
    switched: bool = False
    completed_gate_id: int | None = None
    from_gate_id: int | None = None
    to_gate_id: int | None = None
    reason: str = ""


@dataclass
class OnboardCorridorNavigator:
    track: RaceTrack
    approach_distance_m: float = 2.0
    exit_distance_m: float = 2.0
    target_reached_threshold_m: float = 0.6
    active_index: int = 0
    phase: str = APPROACH
    completed_gate_ids: list[int] = field(default_factory=list)
    _corridors: list[GateCorridor] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.target_reached_threshold_m <= 0.0:
            raise ValueError("target_reached_threshold_m must be positive")
        self._corridors = [
            build_gate_corridor(
                gate,
                approach_distance_m=self.approach_distance_m,
                exit_distance_m=self.exit_distance_m,
            )
            for gate in self.track.gates
        ]

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

    @property
    def active_corridor(self) -> GateCorridor | None:
        if self.is_finished:
            return None
        return self._corridors[self.active_index]

    @property
    def active_target(self) -> GuidanceTarget | None:
        corridor = self.active_corridor
        gate = self.active_gate
        if corridor is None or gate is None:
            return None

        if self.phase == APPROACH:
            return GuidanceTarget(
                gate_id=gate.id,
                target_name="approach",
                position=corridor.approach_point,
                beacon_id=gate.beacon_id,
                beacon_position=gate.beacon_position,
            )

        return GuidanceTarget(
            gate_id=gate.id,
            target_name="exit",
            position=corridor.exit_point,
            beacon_id=gate.beacon_id,
            beacon_position=gate.beacon_position,
        )

    @property
    def keep_forward_near_target(self) -> bool:
        return self.phase in {TRANSIT, EXIT}

    def update_from_measurement(self, measurement: BeaconMeasurement) -> OnboardNavigationUpdate:
        gate = self.active_gate
        if gate is None or measurement.active_gate_id != gate.id:
            return OnboardNavigationUpdate()

        if self.phase == APPROACH:
            if measurement.distance_m < self.target_reached_threshold_m:
                self.phase = TRANSIT
                return OnboardNavigationUpdate(
                    phase_changed=True,
                    from_phase=APPROACH,
                    to_phase=TRANSIT,
                    reason="approach_reached",
                )
            return OnboardNavigationUpdate()

        if measurement.distance_m < self.target_reached_threshold_m:
            from_phase = self.phase
            self.phase = EXIT
            return self._advance(
                reason="exit_reached",
                phase_changed=from_phase != EXIT,
                from_phase=from_phase,
                to_phase=EXIT,
            )

        return OnboardNavigationUpdate()

    def _advance(
        self,
        reason: str,
        phase_changed: bool = False,
        from_phase: str | None = None,
        to_phase: str | None = None,
    ) -> OnboardNavigationUpdate:
        gate = self.active_gate
        if gate is None:
            return OnboardNavigationUpdate()

        completed_gate_id = gate.id
        self.completed_gate_ids.append(completed_gate_id)
        self.active_index += 1
        next_gate_id = self.active_gate_id
        self.phase = APPROACH

        return OnboardNavigationUpdate(
            phase_changed=phase_changed,
            from_phase=from_phase,
            to_phase=to_phase,
            switched=True,
            completed_gate_id=completed_gate_id,
            from_gate_id=completed_gate_id,
            to_gate_id=next_gate_id,
            reason=reason,
        )
