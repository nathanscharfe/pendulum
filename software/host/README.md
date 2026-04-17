# Host Software

Initial Python host-side project for communicating with the three-Arduino pendulum hardware setup.

GitHub issue: #7, `Set up initial host software project`

## Purpose

The host software starts one serial interface object for each Arduino:

- Arduino #1: linear actuator controller
- Arduino #2: optical limit sensor reader
- Arduino #3: AS5600 magnetic encoder reader

Each interface owns its serial port. Reader threads continuously collect serial data in the background and expose the latest parsed sample to the main script.

## Setup

Install Python dependencies:

```powershell
python -m pip install -r software\host\requirements.txt
```

## Run

Run with whichever Arduino ports are currently connected:

```powershell
python -m software.host.main --actuator-port COM3 --limits-port COM4 --encoder-port COM5
```

Current bench COM-port assignment:

- Arduino #1 actuator controller: `COM6`
- Arduino #2 limit sensor reader: `COM10`
- Arduino #3 AS5600 encoder reader: `COM8`

Current bench command:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8
```

Live bench smoke test:

- Date: 2026-04-17
- Command: `python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8`
- Result: host successfully read actuator status, limit sensor state, and encoder angle data from all three Arduinos.
- Shutdown: `Ctrl+C` stopped the host monitor and sent a stop command to the actuator interface.

For an initial read-only test with only one Arduino connected, provide only that port:

```powershell
python -m software.host.main --encoder-port COM5
```

Optional flags:

- `--duration 10`: run for 10 seconds and exit.
- `--zero-encoder-on-start`: send `z` to the encoder Arduino after opening the port.
- `--poll-period 0.5`: print snapshots every 0.5 seconds.

## Safety Note

The first host scaffold only reads status streams and exposes command helpers. It is not yet a closed-loop controller. Before hardware control tests, the main loop should enforce limit-sensor stops before issuing actuator motion commands.
