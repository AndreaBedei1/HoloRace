"""Small registry for named race tracks."""

from __future__ import annotations

from collections.abc import Callable

from underwater_racing.config.tracks import single_gate, straight_line, zig_zag
from underwater_racing.racing.track import RaceTrack

TrackBuilder = Callable[[], RaceTrack]

TRACK_BUILDERS: dict[str, TrackBuilder] = {
    "single": single_gate.build_track,
    "straight": straight_line.build_track,
    "zigzag": zig_zag.build_track,
}


def available_tracks() -> list[str]:
    return sorted(TRACK_BUILDERS)


def build_track(name: str) -> RaceTrack:
    key = name.lower()
    try:
        return TRACK_BUILDERS[key]()
    except KeyError as exc:
        choices = ", ".join(available_tracks())
        raise ValueError(f"Unknown track '{name}'. Choose one of: {choices}") from exc
