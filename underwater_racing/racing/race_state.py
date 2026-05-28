"""Race progress state for ordered gates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from underwater_racing.geometry.gate_geometry import CrossingResult
from underwater_racing.racing.gate import RaceGate
from underwater_racing.racing.track import RaceTrack


@dataclass
class RaceState:
    track: RaceTrack
    active_index: int = 0
    completed_gate_ids: List[int] = field(default_factory=list)

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

    def update_from_crossing(self, crossing: CrossingResult) -> bool:
        """Advance to the next gate if the active gate was crossed correctly."""
        gate = self.active_gate
        if gate is None or crossing.gate_id != gate.id:
            return False
        if not (crossing.crossed and crossing.inside_opening):
            return False

        self.completed_gate_ids.append(gate.id)
        self.active_index += 1
        return True
