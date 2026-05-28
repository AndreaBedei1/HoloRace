"""Reusable HoloOcean runner for configured race tracks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from underwater_racing.config.default_vehicle import (
    MAX_DEMO_DURATION_S,
    ROVER_CONFIG_SOURCE,
    ROVER_NAME,
    ROVER_START_LOCATION,
    ROVER_START_ROTATION,
    TICKS_PER_SEC,
)
from underwater_racing.config.track_registry import build_track
from underwater_racing.control.simple_gate_follower import RoverCommand, SimpleGateFollower
from underwater_racing.geometry.gate_geometry import CrossingDetector, CrossingResult
from underwater_racing.holoocean.prop_spawner import spawn_beacon_marker, spawn_gate
from underwater_racing.holoocean.scenario_builder import build_scenario, choose_world
from underwater_racing.holoocean.state_parsing import has_collision, parse_vehicle_pose
from underwater_racing.holoocean.vehicle_loader import BlueROVThrusterAdapter, build_bluerov_config
from underwater_racing.logging_utils.race_logger import RaceLogger
from underwater_racing.racing.beacon import BeaconMeasurement, GuidanceTarget, VirtualGuidanceProvider
from underwater_racing.racing.onboard_corridor_navigator import (
    OnboardCorridorNavigator,
    OnboardNavigationUpdate,
)
from underwater_racing.racing.race_state import RaceState

POST_FINISH_SURGE = 0.25


@dataclass(frozen=True)
class TrackDemoConfig:
    track_name: str = "single"
    world: str | None = None
    headless: bool = False
    max_duration_s: float = MAX_DEMO_DURATION_S
    ticks_per_sec: int = TICKS_PER_SEC
    output_root: str | Path | None = None
    post_finish_duration_s: float = 4.0
    axis_aligned_visual_gates: bool = False
    yaw_sign: float = 1.0


def run_track_demo(config: TrackDemoConfig) -> int:
    if config.post_finish_duration_s < 0.0:
        raise ValueError("post_finish_duration_s cannot be negative")

    try:
        import holoocean
    except ImportError:
        print(
            "Could not import holoocean in this Python environment. "
            "Install/activate HoloOcean, then rerun the demo script."
        )
        return 1

    track = build_track(config.track_name)
    referee_state = RaceState(track)
    onboard_navigator = OnboardCorridorNavigator(track)
    selected_world = choose_world(config.world)
    vehicle_config = build_bluerov_config()
    scenario_cfg = build_scenario(
        vehicle_config=vehicle_config,
        world=selected_world,
        name=f"UnderwaterRacing{config.track_name.title()}Track",
        ticks_per_sec=config.ticks_per_sec,
        frames_per_sec=True,
    )

    controller = SimpleGateFollower()
    adapter = BlueROVThrusterAdapter(yaw_sign=config.yaw_sign)
    guidance_provider = VirtualGuidanceProvider()
    output_root = config.output_root or f"outputs/{config.track_name}_track_demo"

    collision_count = 0
    elapsed_time = 0.0
    finish_time_s = None
    stopped_after_finish_clearance = False
    rotate_visual_boxes = _should_rotate_visual_boxes(config)

    with RaceLogger(root=output_root) as logger:
        print(f"Output directory: {logger.output_dir}")
        print(f"Selected world: {selected_world}")
        print(f"Track: {config.track_name} ({len(track.gates)} gates)")
        print(f"Rover config source: {ROVER_CONFIG_SOURCE}")
        if not rotate_visual_boxes:
            print("Visual gate boxes are spawned axis-aligned for compact frames.")

        with holoocean.make(
            scenario_cfg=scenario_cfg,
            show_viewport=not config.headless,
            ticks_per_sec=config.ticks_per_sec,
            frames_per_sec=True,
            start_world=True,
        ) as env:
            for gate in track.gates:
                spawn_gate(env, gate, rotate_visual_boxes=rotate_visual_boxes)
                spawn_beacon_marker(env, gate)

            referee_gate = referee_state.active_gate
            if referee_gate is None:
                raise RuntimeError("Track has no active gate")

            referee_detector = CrossingDetector(
                referee_gate,
                previous_position=ROVER_START_LOCATION,
            )
            action = adapter.zero_action()
            position = list(ROVER_START_LOCATION)
            yaw_deg = float(ROVER_START_ROTATION[2])
            next_print_time = 0.0
            max_steps = int(config.max_duration_s * config.ticks_per_sec)

            for step in range(max_steps):
                elapsed_time = step / float(config.ticks_per_sec)
                state = env.step(action)

                pose = parse_vehicle_pose(state, agent_name=ROVER_NAME)
                if pose is not None:
                    position = pose.position
                    yaw_deg = pose.yaw_deg

                referee_detector = _update_referee(
                    detector=referee_detector,
                    referee_state=referee_state,
                    position=position,
                    logger=logger,
                    time_s=elapsed_time,
                )

                target = onboard_navigator.active_target
                measurement = _measure_onboard_target(
                    onboard_navigator,
                    guidance_provider,
                    position,
                    yaw_deg,
                )
                if measurement is not None:
                    update = onboard_navigator.update_from_measurement(measurement)
                    if update.phase_changed:
                        logger.log_event(
                            elapsed_time,
                            "onboard_phase_changed",
                            update.completed_gate_id or onboard_navigator.active_gate_id,
                            (
                                f"from={update.from_phase}, to={update.to_phase}, "
                                f"reason={update.reason}"
                            ),
                        )
                    if update.switched:
                        _log_onboard_switch(logger, elapsed_time, update)
                    if update.phase_changed or update.switched:
                        target = onboard_navigator.active_target
                        measurement = _measure_onboard_target(
                            onboard_navigator,
                            guidance_provider,
                            position,
                            yaw_deg,
                        )

                if onboard_navigator.is_finished and referee_state.is_finished and finish_time_s is None:
                    finish_time_s = elapsed_time

                if finish_time_s is not None:
                    command = RoverCommand(surge=POST_FINISH_SURGE)
                    action = adapter.to_action(command)
                elif measurement is None and onboard_navigator.is_finished:
                    command = RoverCommand(surge=POST_FINISH_SURGE)
                    action = adapter.to_action(command)
                elif measurement is None:
                    command = RoverCommand()
                    action = adapter.zero_action()
                else:
                    command = controller.compute_command(
                        measurement,
                        keep_forward_near_target=onboard_navigator.keep_forward_near_target,
                    )
                    action = adapter.to_action(command)

                if has_collision(state, agent_name=ROVER_NAME):
                    collision_count += 1
                    logger.log_event(
                        elapsed_time,
                        "collision",
                        onboard_navigator.active_gate_id,
                        f"referee_active_gate_id={_format_optional(referee_state.active_gate_id)}",
                    )

                _log_trajectory(
                    logger=logger,
                    time_s=elapsed_time,
                    position=position,
                    yaw_deg=yaw_deg,
                    onboard_gate_id=onboard_navigator.active_gate_id,
                    onboard_phase=None if onboard_navigator.is_finished else onboard_navigator.phase,
                    target=target,
                    referee_gate_id=referee_state.active_gate_id,
                    measurement=measurement,
                    command=command,
                )

                if elapsed_time >= next_print_time:
                    _print_status(
                        elapsed_time=elapsed_time,
                        position=position,
                        onboard_navigator=onboard_navigator,
                        referee_state=referee_state,
                        measurement=measurement,
                        collision_count=collision_count,
                        total_gates=len(track.gates),
                    )
                    next_print_time += 1.0

                if (
                    finish_time_s is not None
                    and elapsed_time - finish_time_s >= config.post_finish_duration_s
                ):
                    stopped_after_finish_clearance = True
                    break

        summary = {
            "track_name": config.track_name,
            "total_gates": len(track.gates),
            "onboard_completed_gates": len(onboard_navigator.completed_gate_ids),
            "referee_passed_gates": len(referee_state.completed_gate_ids),
            "race_finished_onboard": onboard_navigator.is_finished,
            "race_finished_referee": referee_state.is_finished,
            "finish_time": finish_time_s,
            "post_finish_duration_s": config.post_finish_duration_s,
            "stopped_after_finish_clearance": stopped_after_finish_clearance,
            "elapsed_time": elapsed_time,
            "collision_count": collision_count,
            "selected_world": selected_world,
            "yaw_sign": config.yaw_sign,
            "rover_config_source": ROVER_CONFIG_SOURCE,
        }
        logger.write_summary(summary)
        print(f"Summary written to: {logger.output_dir / 'summary.json'}")

    return 0


def _should_rotate_visual_boxes(config: TrackDemoConfig) -> bool:
    if config.axis_aligned_visual_gates:
        return False
    return not config.track_name.lower().startswith("zigzag")


def _update_referee(
    detector: CrossingDetector,
    referee_state: RaceState,
    position: list[float],
    logger: RaceLogger,
    time_s: float,
) -> CrossingDetector:
    if referee_state.active_gate is None:
        return detector

    crossing = detector.update(position)
    if not crossing.plane_crossed:
        return detector

    if referee_state.update_from_crossing(crossing):
        logger.log_event(time_s, "referee_gate_passed", crossing.gate_id)
        if referee_state.active_gate is not None:
            detector.reset(referee_state.active_gate, position)
        return detector

    logger.log_event(
        time_s,
        "referee_gate_missed",
        crossing.gate_id,
        _crossing_details(crossing),
    )
    return detector


def _measure_onboard_target(
    onboard_navigator: OnboardCorridorNavigator,
    guidance_provider: VirtualGuidanceProvider,
    position: list[float],
    yaw_deg: float,
) -> BeaconMeasurement | None:
    target = onboard_navigator.active_target
    if target is None:
        return None
    return guidance_provider.get_measurement(position, yaw_deg, target)


def _log_onboard_switch(
    logger: RaceLogger,
    time_s: float,
    switch: OnboardNavigationUpdate,
) -> None:
    logger.log_event(
        time_s,
        "onboard_switch",
        switch.completed_gate_id,
        f"to_gate_id={_format_optional(switch.to_gate_id)}, reason={switch.reason}",
    )


def _log_trajectory(
    logger: RaceLogger,
    time_s: float,
    position: list[float],
    yaw_deg: float,
    onboard_gate_id: int | None,
    onboard_phase: str | None,
    target: GuidanceTarget | None,
    referee_gate_id: int | None,
    measurement: BeaconMeasurement | None,
    command: RoverCommand,
) -> None:
    logger.log_trajectory(
        time=f"{time_s:.3f}",
        x=f"{position[0]:.4f}",
        y=f"{position[1]:.4f}",
        z=f"{position[2]:.4f}",
        yaw_deg=f"{yaw_deg:.4f}",
        onboard_gate_id=_format_optional(onboard_gate_id),
        onboard_active_gate_id=_format_optional(onboard_gate_id),
        onboard_phase="" if onboard_phase is None else onboard_phase,
        referee_active_gate_id=_format_optional(referee_gate_id),
        target_name="" if target is None else target.target_name,
        target_x="" if target is None else f"{target.position[0]:.4f}",
        target_y="" if target is None else f"{target.position[1]:.4f}",
        target_z="" if target is None else f"{target.position[2]:.4f}",
        distance_to_gate="" if measurement is None else f"{measurement.distance_m:.4f}",
        bearing_error="" if measurement is None else f"{measurement.bearing_error_deg:.4f}",
        vertical_error="" if measurement is None else f"{measurement.vertical_error_m:.4f}",
        command_surge=f"{command.surge:.4f}",
        command_sway=f"{command.sway:.4f}",
        command_heave=f"{command.heave:.4f}",
        command_yaw=f"{command.yaw:.4f}",
    )


def _print_status(
    elapsed_time: float,
    position: list[float],
    onboard_navigator: OnboardCorridorNavigator,
    referee_state: RaceState,
    measurement: BeaconMeasurement | None,
    collision_count: int,
    total_gates: int,
) -> None:
    if measurement is None:
        guidance = "dist=n/a bearing=n/a vertical=n/a"
    else:
        guidance = (
            f"dist={measurement.distance_m:.2f}m "
            f"bearing={measurement.bearing_error_deg:.1f}deg "
            f"vertical={measurement.vertical_error_m:.2f}m"
        )

    print(
        "t={:.1f}s pos=({:.2f}, {:.2f}, {:.2f}) onboard_gate={} phase={} "
        "target={} referee_gate={} {} onboard_done={}/{} referee_done={}/{} collisions={}".format(
            elapsed_time,
            position[0],
            position[1],
            position[2],
            _format_optional(onboard_navigator.active_gate_id),
            "" if onboard_navigator.is_finished else onboard_navigator.phase,
            "" if measurement is None else measurement.target_name,
            _format_optional(referee_state.active_gate_id),
            guidance,
            len(onboard_navigator.completed_gate_ids),
            total_gates,
            len(referee_state.completed_gate_ids),
            total_gates,
            collision_count,
        )
    )


def _crossing_details(crossing: CrossingResult) -> str:
    return (
        f"inside={crossing.inside_opening}, direction_ok={crossing.direction_ok}, "
        f"lateral={crossing.lateral_error_m:.3f}, vertical={crossing.vertical_error_m:.3f}"
    )


def _format_optional(value: int | None) -> str:
    return "" if value is None else str(value)
