"""Run a configured underwater racing HoloOcean track demo."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from underwater_racing.config.default_vehicle import MAX_DEMO_DURATION_S, TICKS_PER_SEC
from underwater_racing.config.track_registry import available_tracks
from underwater_racing.holoocean.track_demo_runner import TrackDemoConfig, run_track_demo


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a HoloOcean racing track demo.")
    parser.add_argument(
        "--track",
        choices=available_tracks(),
        default="single",
        help="Track layout to run.",
    )
    parser.add_argument("--world", default="OpenWater", help="HoloOcean world name.")
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
    parser.add_argument(
        "--post-finish-duration",
        type=float,
        default=4.0,
        help="Seconds to keep moving after onboard and referee finish.",
    )
    parser.add_argument(
        "--axis-aligned-visual-gates",
        action="store_true",
        help="Spawn gate frames without yaw rotation while keeping referee yaw unchanged.",
    )
    parser.add_argument(
        "--invert-yaw",
        action="store_true",
        help="Invert yaw thruster contribution for testing BlueROV yaw sign.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return run_track_demo(
        TrackDemoConfig(
            track_name=args.track,
            world=args.world,
            headless=args.headless,
            max_duration_s=args.max_duration,
            ticks_per_sec=args.ticks_per_sec,
            post_finish_duration_s=args.post_finish_duration,
            axis_aligned_visual_gates=args.axis_aligned_visual_gates,
            yaw_sign=-1.0 if args.invert_yaw else 1.0,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
