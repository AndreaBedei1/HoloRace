# Underwater Racing for HoloOcean

This repository adds a small modular Python layer for building underwater drone racing tracks in HoloOcean. The first target is a single square gate demo: a BlueROV2-compatible vehicle starts in front of a 1.5 m x 1.5 m opening, receives guidance from a virtual beacon above the gate, and attempts to fly through the center.

The code intentionally keeps HoloOcean-specific details thin. Racing logic uses gates, tracks, beacon measurements, and abstract `surge/sway/heave/yaw` commands. HoloOcean adapters convert those objects to scenario dictionaries, primitive box props, parsed pose state, and the BlueROV2 8-thruster action vector.

## Structure

```text
underwater_racing/
  config/            default track, vehicle, and runtime values
  geometry/          transforms, gate box geometry, crossing detection
  holoocean/         scenario, prop spawning, pose parsing, thruster adapter
  racing/            gates, tracks, virtual beacons, race state, scoring exports
  control/           simple gate follower controller
  logging_utils/     CSV and JSON race logger
scripts/
  run_single_gate_demo.py
tests/
  test_gate_geometry.py
  test_crossing_detection.py
  test_beacon_guidance.py
```

## Run the Demo

Activate an environment with HoloOcean and the Ocean package installed, then run:

```bash
python scripts/run_single_gate_demo.py
```

Useful options:

```bash
python scripts/run_single_gate_demo.py --headless
python scripts/run_single_gate_demo.py --world SimpleUnderwater
python scripts/run_single_gate_demo.py --max-duration 60
```

The script writes:

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

The beacon does not target its own physical marker position. Its measurement targets `gate.center`, the center of the empty opening. The measurement includes distance, world direction, yaw/bearing error, vertical error, elevation error, active gate id, and beacon id. This keeps the interface replaceable by real `AcousticBeaconSensor` messages later.

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

These tests verify the exact gate opening geometry, virtual beacon target behavior, and geometric crossing detection.

## Remaining Work

- Tune the simple controller in the actual HoloOcean runtime.
- Replace the virtual beacon with real `AcousticBeaconSensor` messages.
- Add multiple-gate demo tracks and per-gate visual styles.
- Improve collision interpretation once the exact CollisionSensor payload is confirmed.
- Add camera/sonar visual diagnostics only where needed.

HoloOcean references used for this implementation:

- Scenario dictionaries and `holoocean.make`: https://byu-holoocean.github.io/holoocean-docs/v2.0.0/usage/scenarios.html
- Primitive prop spawning with `spawn_prop`: https://byu-holoocean.github.io/holoocean-docs/v2.0.1/holoocean/environments.html
- Acoustic beacon sensor concept: https://byu-holoocean.github.io/holoocean-docs/v2.2.1/sensors/sensors/acoustic-beacon-sensor.html
