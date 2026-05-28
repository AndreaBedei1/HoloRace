"""Run the first underwater racing single-gate HoloOcean demo."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from underwater_racing.config.default_track import build_default_track
from underwater_racing.config.default_vehicle import (
    MAX_DEMO_DURATION_S,
    ROVER_CONFIG_SOURCE,
    ROVER_NAME,
    ROVER_START_LOCATION,
    ROVER_START_ROTATION,
    TICKS_PER_SEC,
)
from underwater_racing.control.simple_gate_follower import RoverCommand, SimpleGateFollower
from underwater_racing.geometry.gate_geometry import CrossingDetector
from underwater_racing.holoocean.prop_spawner import spawn_beacon_marker, spawn_gate
from underwater_racing.holoocean.scenario_builder import build_scenario, choose_world
from underwater_racing.holoocean.state_parsing import has_collision, parse_vehicle_pose
from underwater_racing.holoocean.vehicle_loader import BlueROVThrusterAdapter, build_bluerov_config
from underwater_racing.logging_utils.race_logger import RaceLogger
from underwater_racing.racing.beacon import VirtualGateBeacon
from underwater_racing.racing.race_state import RaceState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the HoloOcean single-gate racing demo.")
    parser.add_argument("--world", default=None, help="HoloOcean world name. Defaults to OpenWater.")
    parser.add_argument("--headless", action="store_true", help="Run without rendering the viewport.")
    parser.add_argument(
        "--max-duration",
        type=float,
        default=MAX_DEMO_DURATION_S,
        help="Maximum demo duration in simulated seconds.",
    )
    parser.add_argument(
        "--ticks-per-sec",
        type=int,
        default=TICKS_PER_SEC,
        help="HoloOcean simulation ticks per second.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        import holoocean
    except ImportError:
        print(
            "Could not import holoocean in this Python environment. "
            "Install/activate HoloOcean, then rerun: python scripts/run_single_gate_demo.py"
        )
        return 1

    track = build_default_track()
    race_state = RaceState(track)
    selected_world = choose_world(args.world)
    vehicle_config = build_bluerov_config()
    scenario_cfg = build_scenario(
        vehicle_config=vehicle_config,
        world=selected_world,
        ticks_per_sec=args.ticks_per_sec,
        frames_per_sec=True,
    )

    controller = SimpleGateFollower()
    adapter = BlueROVThrusterAdapter()
    beacons = {gate.id: VirtualGateBeacon(gate) for gate in track.gates}

    gate_passed = False
    final_distance = None
    min_distance = float("inf")
    collision_count = 0
    elapsed_time = 0.0

    with RaceLogger() as logger:
        print(f"Output directory: {logger.output_dir}")
        print(f"Selected world: {selected_world}")
        print(f"Rover config source: {ROVER_CONFIG_SOURCE}")

        with holoocean.make(
            scenario_cfg=scenario_cfg,
            show_viewport=not args.headless,
            ticks_per_sec=args.ticks_per_sec,
            frames_per_sec=True,
            start_world=True,
        ) as env:
            for gate in track.gates:
                spawn_gate(env, gate)
                spawn_beacon_marker(env, gate)

            active_gate = race_state.active_gate
            if active_gate is None:
                raise RuntimeError("Track has no active gate")

            detector = CrossingDetector(active_gate, previous_position=ROVER_START_LOCATION)
            action = adapter.zero_action()
            position = list(ROVER_START_LOCATION)
            yaw_deg = float(ROVER_START_ROTATION[2])
            next_print_time = 0.0
            max_steps = int(args.max_duration * args.ticks_per_sec)

            for step in range(max_steps):
                elapsed_time = step / float(args.ticks_per_sec)
                state = env.step(action)

                pose = parse_vehicle_pose(state, agent_name=ROVER_NAME)
                if pose is not None:
                    position = pose.position
                    yaw_deg = pose.yaw_deg

                active_gate = race_state.active_gate
                if active_gate is None:
                    break

                measurement = beacons[active_gate.id].get_measurement(position, yaw_deg)
                final_distance = measurement.distance_m
                min_distance = min(min_distance, measurement.distance_m)

                crossing = detector.update(position)
                if crossing.plane_crossed:
                    logger.log_event(
                        elapsed_time,
                        "gate_plane_crossed",
                        crossing.gate_id,
                        (
                            f"inside={crossing.inside_opening}, "
                            f"lateral={crossing.lateral_error_m:.3f}, "
                            f"vertical={crossing.vertical_error_m:.3f}"
                        ),
                    )

                if race_state.update_from_crossing(crossing):
                    gate_passed = True
                    logger.log_event(elapsed_time, "gate_passed", crossing.gate_id)
                    if race_state.active_gate is None:
                        command = RoverCommand()
                        action = adapter.zero_action()
                    else:
                        detector.reset(race_state.active_gate, position)
                        measurement = beacons[race_state.active_gate.id].get_measurement(position, yaw_deg)
                        command = controller.compute_command(measurement)
                        action = adapter.to_action(command)
                else:
                    command = controller.compute_command(measurement)
                    action = adapter.to_action(command)

                if has_collision(state, agent_name=ROVER_NAME):
                    collision_count += 1
                    logger.log_event(elapsed_time, "collision", race_state.active_gate_id)

                logger.log_trajectory(
                    time=f"{elapsed_time:.3f}",
                    x=f"{position[0]:.4f}",
                    y=f"{position[1]:.4f}",
                    z=f"{position[2]:.4f}",
                    active_gate_id=measurement.active_gate_id,
                    distance_to_gate=f"{measurement.distance_m:.4f}",
                    bearing_error=f"{measurement.bearing_error_deg:.4f}",
                    vertical_error=f"{measurement.vertical_error_m:.4f}",
                    command_surge=f"{command.surge:.4f}",
                    command_sway=f"{command.sway:.4f}",
                    command_heave=f"{command.heave:.4f}",
                    command_yaw=f"{command.yaw:.4f}",
                )

                if elapsed_time >= next_print_time:
                    print(
                        "t={:.1f}s pos=({:.2f}, {:.2f}, {:.2f}) gate={} "
                        "dist={:.2f}m bearing={:.1f}deg vertical={:.2f}m passed={} collisions={}".format(
                            elapsed_time,
                            position[0],
                            position[1],
                            position[2],
                            measurement.active_gate_id,
                            measurement.distance_m,
                            measurement.bearing_error_deg,
                            measurement.vertical_error_m,
                            gate_passed,
                            collision_count,
                        )
                    )
                    next_print_time += 1.0

                if gate_passed and race_state.is_finished:
                    break

        summary = {
            "gate_passed": gate_passed,
            "final_distance_to_gate": final_distance,
            "min_distance_to_gate": None if min_distance == float("inf") else min_distance,
            "elapsed_time": elapsed_time,
            "collision_count": collision_count,
            "selected_world": selected_world,
            "rover_config_source": ROVER_CONFIG_SOURCE,
        }
        logger.write_summary(summary)
        print(f"Summary written to: {logger.output_dir / 'summary.json'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
