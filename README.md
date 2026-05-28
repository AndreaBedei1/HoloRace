# Underwater Racing for HoloOcean

This repository adds a small modular Python layer for building underwater drone racing tracks in HoloOcean. A BlueROV2-compatible vehicle starts in front of square gates with exact 1.5 m x 1.5 m openings, receives guidance from virtual gate beacons, and attempts to fly through the ordered track.

The code intentionally keeps HoloOcean-specific details thin. Racing logic uses gates, tracks, beacon measurements, onboard gate selection, referee state, and abstract `surge/sway/heave/yaw` commands. HoloOcean adapters convert those objects to scenario dictionaries, primitive box props, parsed pose state, and the BlueROV2 8-thruster action vector.

## Structure

```text
underwater_racing/
  config/            track registry, track layouts, vehicle, and runtime values
    tracks/          single, straight, and zigzag track definitions
  geometry/          transforms, gate box geometry, crossing detection
  holoocean/         scenario, prop spawning, pose parsing, thruster adapter
  racing/            gates, tracks, virtual beacons, onboard selector, race state
  control/           simple gate follower controller
  logging_utils/     CSV and JSON race logger
scripts/
  run_single_gate_demo.py
  run_track_demo.py
tests/
  test_gate_geometry.py
  test_crossing_detection.py
  test_beacon_guidance.py
  test_track_configs.py
  test_onboard_gate_selector.py
```

## Run the Demo

Activate an environment with HoloOcean and the Ocean package installed, then run:

```bash
python scripts/run_single_gate_demo.py
```

The generic track runner supports the single-gate, straight-line, and zigzag layouts:

```bash
python scripts/run_track_demo.py --track single --headless
python scripts/run_track_demo.py --track straight --headless
python scripts/run_track_demo.py --track zigzag --headless
python scripts/run_track_demo.py --track zigzag_smooth --headless
```

Useful options:

```bash
python scripts/run_track_demo.py --track straight --world OpenWater
python scripts/run_track_demo.py --track zigzag --max-duration 60
python scripts/run_track_demo.py --track zigzag_smooth --max-duration 120 --invert-yaw
python scripts/run_track_demo.py --track single --ticks-per-sec 30
python scripts/run_track_demo.py --track zigzag --post-finish-duration 4
python scripts/run_track_demo.py --track zigzag --axis-aligned-visual-gates
```

The generic script writes:

```text
outputs/<track>_track_demo/YYYYMMDD_HHMMSS/
  trajectory.csv
  race_events.csv
  summary.json
```

The compatibility single-gate script still writes:

```text
outputs/single_gate_demo/YYYYMMDD_HHMMSS/
  trajectory.csv
  race_events.csv
  summary.json
```

## Gate Representation

Each `RaceGate` has an exact internal opening of `1.5 m x 1.5 m`. The frame is procedural and uses four HoloOcean box props:

- top bar
- bottom bar
- left bar
- right bar

The default frame thickness is `0.25 m`, so the default outer square size is `2.0 m`. The box depth is `0.30 m`. The gate plane normal points along the gate local `+X` axis, and `yaw_deg` rotates the gate around the world `Z` axis.

## Virtual Beacon

Each gate owns one `VirtualGateBeacon` with `beacon_id == gate.id`. The marker position is above the gate:

```text
gate.center + [0, 0, opening_size / 2 + frame_thickness + beacon_clearance]
```

The beacon does not target its own physical marker position. Its measurement targets a configurable navigation point inside the gate opening. By default this point is `0.25 m` below `gate.center` so the simple controller aims slightly lower through the opening. The measurement includes distance, world direction, yaw/bearing error, vertical error, elevation error, active gate id, and beacon id. This keeps the interface replaceable by real `AcousticBeaconSensor` messages later.

The virtual beacon is a sensor emulator for the demo. The rover's onboard mission state sees only `BeaconMeasurement` values, so a real acoustic beacon measurement source can replace it later without changing the gate selector.

## Onboard Selection vs Referee

The rover has two separate state machines:

- `OnboardCorridorNavigator` is the rover's internal mission state. It follows each gate's approach and exit corridor using only `BeaconMeasurement` values for onboard target points.
- `RaceState` is the referee state. It uses parsed HoloOcean pose with `CrossingDetector` to score true geometric gate crossings through the active gate opening.

PoseSensor / simulator ground truth is used only for referee scoring, crossing detection, logging, debugging, and the final summary. It does not choose the controller target. The controller follows the target selected by `OnboardCorridorNavigator`, and the referee independently records whether that choice actually produced valid crossings.

The logs include both concepts:

- `trajectory.csv`: position, yaw, onboard active gate, onboard phase, target point, referee active gate, beacon guidance errors, and abstract command values
- `race_events.csv`: `onboard_phase_changed`, `onboard_switch`, `referee_gate_passed`, `referee_gate_missed`, and `collision` events
- `summary.json`: selected track/world, onboard completion, referee completion, passed gate counts, finish time, post-finish clearance, elapsed time, and collisions

For the first stable zigzag demo, visual gate boxes are spawned axis-aligned by default so each frame remains compact in HoloOcean. The referee still uses each gate's configured `yaw_deg` for crossing geometry.

## Existing Rover Compatibility

`HolooceanLibrary/src/lib/rover.py` was inspected and the demo keeps the same basic BlueROV2 scenario schema:

- `agent_type`: `BlueROV2`
- `control_scheme`: `0`
- initial `location` and `rotation`
- sensor dictionaries using HoloOcean sensor types

Only the minimal sensors needed for this demo are configured: pose, velocity, IMU, depth, and collision. The old keyboard thruster mapping is preserved behind `BlueROVThrusterAdapter`, so racing code works with abstract commands.

## Tests

Run the pure-Python tests with:

```bash
python -m unittest discover -s tests
```

These tests verify the exact gate opening geometry, virtual beacon target behavior, and geometric crossing detection. They also verify the configured track layouts, unique gate/beacon ids, onboard beacon-only switching, and the independence between onboard state and referee state.

## Remaining Work

- Tune the simple controller in the actual HoloOcean runtime.
- Replace the virtual beacon with real `AcousticBeaconSensor` messages.
- Add per-gate visual styles.
- Improve collision interpretation once the exact CollisionSensor payload is confirmed.
- Add camera/sonar visual diagnostics only where needed.

HoloOcean references used for this implementation:

- Scenario dictionaries and `holoocean.make`: https://byu-holoocean.github.io/holoocean-docs/v2.0.0/usage/scenarios.html
- Primitive prop spawning with `spawn_prop`: https://byu-holoocean.github.io/holoocean-docs/v2.0.1/holoocean/environments.html
- Acoustic beacon sensor concept: https://byu-holoocean.github.io/holoocean-docs/v2.2.1/sensors/sensors/acoustic-beacon-sensor.html
