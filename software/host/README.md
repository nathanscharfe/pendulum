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
- `--steps-per-mm 10.638`: override the actuator position scale.
- `--travel-mm 600`: override the assumed actuator travel.

## Basic Motion Commands

The host includes early motion commands with host-side limit-sensor checks. Current assumptions:

- Negative actuator motion moves toward the left limit.
- Positive actuator motion moves toward the right limit.
- The provisional actuator calibration is `5000 steps / 470 mm`, or about `10.638 steps/mm`.
- The total travel estimate of `600 mm` is provisional. For calibration, prefer raw step moves/counts and then update `--steps-per-mm`.
- Arduino #1 receives raw step and step-rate commands. Millimeter conversion is handled on the host side.

Home left:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 home --side left --speed-mm-s 25
```

Move to an absolute position after homing:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 move-mm 300 --speed-mm-s 25
```

Move a relative number of raw steps:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 move-steps 500 --steps-per-second 100
```

Move to an absolute raw step position:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 goto-steps 2500 --steps-per-second 100
```

Run the interactive travel calibration routine:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 calibrate-travel
```

Default calibration routine:

- Homes left.
- Moves to the 100-step start position.
- Prompts for the tape measure reading at the start position.
- For each speed, `100`, `200`, and `300` steps/s:
  - returns to the 100-step start position
  - moves right by `6000`, `5000`, `4000`, `3000`, `2000`, and `1000` steps
  - prompts for the tape measure reading after each move
- Writes a CSV file under `hardware/linear actuator calibration/`.

Calibration follow-up:

- GitHub issue: #21, `Run actuator travel calibration and fit step-to-mm conversion`
- An initial calibration attempt exposed a stale actuator-status issue after homing. The host now waits for Arduino #1 to confirm `position_steps = 0` after left homing before continuing.
- Empty CSV files from the interrupted attempts were not kept.

Important: opening the actuator Arduino serial port resets the Arduino, so `position_steps` is not preserved across separate Python invocations. For homing, moving, and checking position in one continuous session, use the interactive shell:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 shell
```

Example shell session:

```text
motion> home left 10
motion> status
motion> move 50 10
motion> steps 500 100
motion> goto 2500 100
motion> status
motion> quit
```

Run a signed speed command for a fixed duration:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 speed 20 --duration 2
```

Initial live motion smoke test:

- Date: 2026-04-17
- Command: `python -m software.host.main --actuator-port COM6 --limits-port COM10 speed 5 --duration 1`
- Result: command worked as expected with host-side limit monitoring active.
- Direction check confirmed that positive speed moves away from the left limit and negative speed moves toward the left limit.
- Left-limit protection check: `python -m software.host.main --actuator-port COM6 --limits-port COM10 speed -5 --duration 30` correctly refused motion because the left limit was already active.
- Left homing check: `python -m software.host.main --actuator-port COM6 --limits-port COM10 home --side left --speed-mm-s 10` correctly set the actuator position estimate to zero while already at the left limit.
- Absolute move check: `python -m software.host.main --actuator-port COM6 --limits-port COM10 move-mm 50 --speed-mm-s 10` moved off the left limit and stopped as expected.
- Interactive shell workflow check: `home left 10`, `status`, `move 50 10`, `status`, `quit` worked in one continuous serial session. The final status reported `position_steps = 532` and `position_mm = 50.008`.
- After updating the host to disable the driver after completed `home` and `move` commands, the same shell workflow ended with `enabled = False`, `motion_mode = 0`, `position_steps = 532`, and `position_mm = 50.008`.
- Raw step shell check: `steps 100 100` increased `position_steps` from `0` to `100`, and `steps -100 100` returned `position_steps` to `0`. The displayed `position_mm` came from the provisional host-side step-to-mm conversion.

Stop immediately:

```powershell
python -m software.host.main --actuator-port COM6 stop
```

## Safety Note

The host motion commands are still bench-test utilities, not the final closed-loop controller. The host refuses to start motion into an already-active limit and stops motion if the limit in the active travel direction becomes active. Finite moves wait until the actuator firmware reports completion; use `Ctrl+C` in the shell or the `stop` command if a move needs to be interrupted.
