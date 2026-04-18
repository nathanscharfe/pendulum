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
- `--steps-per-mm 10.652`: override the actuator position scale.
- `--travel-mm 600`: override the assumed actuator travel.

## Basic Motion Commands

The host includes early motion commands with host-side limit-sensor checks. Current assumptions:

- Negative actuator motion moves toward the left limit.
- Positive actuator motion moves toward the right limit.
- The measured actuator travel calibration is `10.652 steps/mm`, or about `0.093879 mm/step`.
- The total travel estimate is about `600 mm` with the current limit sensor placement. Limit sensor physics can still make the observed limit-to-limit distance vary slightly by run.
- Arduino #1 receives raw step and step-rate commands. Millimeter conversion is handled on the host side.

Home left:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 home --side left --speed-mm-s 25
```

Move to an absolute position after homing:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 move-mm 300 --speed-mm-s 25
```

Move to the configured midpoint after homing:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 middle --speed-mm-s 25
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
- Prompts for the speed to run in steps/s.
- For each move length, `6000`, `5000`, `4000`, `3000`, `2000`, and `1000` steps:
  - returns to the 100-step start position
  - prompts for the tape measure reading at the 100-step start position
  - moves right by the requested step count
  - prompts for the tape measure reading after the move
- After a speed is complete, asks whether to run another speed or quit and save.
- Writes a CSV file under `hardware/linear actuator calibration/`.

Optional queued speeds can be supplied up front:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 calibrate-travel --speeds 100 200 300
```

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
motion> middle 10
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

Capture a free-swing pendulum encoder experiment:

```powershell
python -m software.host.main --encoder-port COM8 capture-encoder
```

The capture command waits for you to press Enter, zeros the current downward resting angle, records encoder data while you manually displace and release the pendulum, then stops when you press Enter again. Each run writes a timestamped CSV under `hardware/magnetic encoder/captures/`.

To keep the existing encoder zero instead:

```powershell
python -m software.host.main --encoder-port COM8 capture-encoder --no-zero-on-start
```

Run a first-pass downward pendulum damping experiment:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-down
```

Run a first-pass upright pendulum balancing experiment:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-up
```

The command first homes left, then moves to the configured midpoint. Once the actuator is centered, let the pendulum hang downward and motionless, then press Enter to zero the encoder and start a conservative LQR-style damping loop. Press Enter again to stop the controller and write the log. `Ctrl+C`, active limit sensors, or exceeding the angle cutoff also stop the actuator and save the data.

The command uses the downward-equilibrium LQR gain from `lqr_downward_first_pass.m`:

```text
K = [0.3162, 0.8139, -0.3088, -0.3479]
```

The upright command uses the upright-equilibrium first-pass LQR gain from `lqr_first_pass.m`:

```text
K = [-1.0, -2.0104, -29.1450, -6.5238]
```

Because the upright first-pass LQR wants substantially more authority than the downward case, the upright command defaults to a larger acceleration and speed clamp:

- Control period: `0.01 s`
- Speed clamp: `+/-500 mm/s`
- Acceleration clamp: `+/-4.0 m/s^2`
- Control trigger: immediate arm after Enter and encoder zeroing

It estimates the full state `[x, x_dot, theta, theta_dot]`, computes `u = -KX`, and integrates that acceleration command into the actuator step-rate interface. The command includes filtering, theta slew-rate limiting, and deadbands so a motionless pendulum does not make the actuator chatter.

Default controller behavior:

- Control period: `0.05 s`
- Speed clamp: `+/-150 mm/s`
- Acceleration clamp: `+/-0.5 m/s^2`
- Angle cutoff: `+/-0.35 rad`
- LQR cart-position gain: `0.3162`
- LQR cart-velocity gain: `0.8139`
- LQR theta gain: `-0.3088`
- LQR theta-dot gain: `-0.3479`
- Estimator: `alpha-beta`
- Alpha-beta angle gain: `0.35`
- Alpha-beta angular-rate gain: `0.04`
- Alpha-beta residual clamp: `0.01 rad`
- Armed alpha-beta angle gain: `0.40`
- Armed alpha-beta angular-rate gain: `0.06`
- Armed alpha-beta residual clamp: `0.025 rad`
- Theta deadband: `0.003 rad`
- Theta-dot deadband: `0.10 rad/s`
- Control trigger: `|theta| > 0.045 rad` or `|theta_dot| > 0.30 rad/s` for `2` consecutive samples
- Settle disarm: `|theta| < 0.03 rad` and `|theta_dot| < 0.15 rad/s` for `25` consecutive samples
- Theta slew-rate limit: `3.0 rad/s`
- Acceleration deadband: `0.005 m/s^2`
- Velocity leak: `0 1/s`
- Centering gain: `0 1/s`

If `--max-accel-m-s2` is set below the default acceleration deadband, the program automatically lowers the effective acceleration deadband to half of the acceleration clamp so commands are not silently zeroed.

Setup defaults:

- Home left speed: `50 mm/s`
- Move-to-middle speed: `50 mm/s`

To skip the setup moves for a repeat test:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-down --no-home --no-middle
```

Each run writes a timestamped control CSV under `hardware/control experiments/downward/` or `hardware/control experiments/upright/`, plus sidecar raw sample logs for encoder, actuator, and limit-sensor serial data. Analysis notebooks live under `hardware/control experiments/analysis/`.

The hardware and model sign conventions are adjustable independently:

- `--invert-actuator-command`: flips the sign between the LQR acceleration command and physical actuator motion.
- `--invert-cart-position`: flips the sign of cart position and cart velocity before applying the LQR gain.
- `--invert-encoder-angle`: flips the sign of theta and theta-dot before applying the LQR gain.
- `--invert-control`: compatibility alias for `--invert-actuator-command`.

If a small perturbation gets worse instead of damping out, stop immediately and try the most likely sign change first. The latest bench runs suggest testing the actuator command and cart-position signs together:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-down --invert-actuator-command --invert-cart-position
```

If the motion is too aggressive for a cautious retest, override the clamps with `--max-speed-mm-s` and `--max-accel-m-s2`.

The default `alpha-beta` estimator is intended to reject the several-degree encoder jumps seen when the pendulum is motionless while still estimating theta-dot with less lag than differentiating a heavily filtered angle. The controller also holds the actuator still until the estimated angle or angular rate exceeds a trigger threshold for several consecutive samples. After a perturbation damps back near zero, it disarms and zeros the speed command so the cart stops chasing encoder jitter. To fall back to the original filter path, use `--estimator legacy`. A more responsive test without making the controller react to every encoder count jump is:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-down --invert-actuator-command --invert-cart-position --period-s 0.02
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
