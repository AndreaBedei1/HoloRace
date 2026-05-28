"""Ordered race track model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List

from underwater_racing.racing.gate import RaceGate


@dataclass(frozen=True)
class RaceTrack:
    gates: List[RaceGate]

    def __post_init__(self) -> None:
        if not self.gates:
            raise ValueError("RaceTrack requires at least one gate")
        ids = [gate.id for gate in self.gates]
        if len(set(ids)) != len(ids):
            raise ValueError("Gate ids must be unique")
        beacon_ids = [gate.beacon_id for gate in self.gates]
        if len(set(beacon_ids)) != len(beacon_ids):
            raise ValueError("Beacon ids must be unique")

    @classmethod
    def from_gates(cls, gates: Iterable[RaceGate]) -> "RaceTrack":
        return cls(gates=list(gates))

    def first_gate(self) -> RaceGate:
        return self.gates[0]

    def gate_after(self, gate_id: int) -> RaceGate | None:
        for index, gate in enumerate(self.gates):
            if gate.id == gate_id and index + 1 < len(self.gates):
                return self.gates[index + 1]
        return None
