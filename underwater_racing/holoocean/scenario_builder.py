"""Build minimal HoloOcean scenario dictionaries for racing demos."""

from __future__ import annotations

from typing import Any, Dict, List

from underwater_racing.config.default_vehicle import DEFAULT_WORLD, WORLD_FALLBACKS


def choose_world(preferred: str | None = None) -> str:
    """Choose a stable underwater world name without requiring HoloOcean at import time."""
    if preferred:
        return preferred
    return DEFAULT_WORLD if DEFAULT_WORLD in WORLD_FALLBACKS else WORLD_FALLBACKS[0]


def build_scenario(
    vehicle_config: Dict[str, Any],
    world: str | None = None,
    name: str = "UnderwaterRacingSingleGate",
    package_name: str = "Ocean",
    main_agent: str | None = None,
    ticks_per_sec: int = 30,
    frames_per_sec: bool | int = True,
) -> Dict[str, Any]:
    selected_world = choose_world(world)
    agent_name = main_agent or vehicle_config["agent_name"]
    return {
        "name": name,
        "package_name": package_name,
        "world": selected_world,
        "main_agent": agent_name,
        "ticks_per_sec": ticks_per_sec,
        "frames_per_sec": frames_per_sec,
        "agents": [vehicle_config],
    }


def available_default_worlds() -> List[str]:
    return list(WORLD_FALLBACKS)
