# Pendulum Optimal Control Demonstration

This project is a working optimal control demonstration for a pendulum mounted to a linear actuator. The repository now includes hardware, modeling, firmware, host-side control software, and experimental results for both downward stabilization and first-pass real-hardware upright balancing with an LQR-based controller.

## Upright Balance Video

<video src="docs/media/upright_balance_run_2026-04-18.mp4" controls muted playsinline></video>

If your Markdown viewer does not render embedded video, open:

- `docs/media/upright_balance_run_2026-04-18.mp4`

This clip shows the current best upright run, where the pendulum balances for several seconds before eventually losing control.

## Run The Upright Control Test

From the repository root, run:

```powershell
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-up
```

What to expect:

1. The cart homes left and then moves to the middle.
2. A staging prompt appears so you can operate the latch servo on Arduino #2:
   - `p` holds the pendulum with the servo at `80 deg`
   - `g` releases the servo to `0 deg`
   - `start` zeroes the encoder, sends `g`, and begins automatic control
3. Use the staging loop until the pendulum is aligned the way you want.
4. Type `start` to begin automatic control.
5. Keep clear and be ready to stop the run if needed.

Important notes:

- The angle at the moment you type `start` becomes the controller's zero reference, so careful manual alignment still matters.
- The current upright setup uses a latch servo on Arduino #2 (`D7`) so the pendulum can be staged before release.
- The current best upright runs used no cart-centering term, so the plain `control-up` command above is the baseline test.
- The measured no-stall actuator ceiling is about `315 mm/s`, and the current upright default speed clamp matches that measured limit.
- Output logs are written under `hardware/control experiments/upright/`.
- Analysis notebook: `hardware/control experiments/analysis/analyze_upright_control.ipynb`

## Project Goals

- Model the pendulum and linear actuator system.
- Identify useful system parameters from measurement and calibration data.
- Design an LQR controller for the linearized pendulum dynamics.
- Implement the control loop using a software interface such as LabVIEW, Python, MATLAB, or another suitable tool.
- Use an Arduino as the interface between the control software and the physical hardware.
- Compare real hardware behavior against the model and controller predictions.

## Hardware

Planned setup:

- Pendulum
- Linear actuator
- Arduino #1 for linear actuator stepper control
- Arduino #2 for optical actuator limit sensors
- Arduino #3 for AS5600 magnetic encoder angle sensing
- Position and/or angle sensors
- Computer running LabVIEW

Current hardware-related files:

- `docs/images/assembled_pendulum_bench_setup_2026-04-17.jpg`
- `hardware/linear actuator calibration/calibration_procedure.md`
- `hardware/linear actuator calibration/linear actuator speed calibration.xlsx`
- `hardware/linear actuator calibration/actuator_speed_fit.ipynb`
- `hardware/linear actuator calibration/host_travel_calibration_20260417_194546.csv`
- `hardware/linear actuator calibration/host_travel_calibration_results.md`
- `hardware/linear actuator calibration/calibrate_linear_actuator/calibrate_linear_actuator.ino`
- `hardware/linear actuator control/linear_actuator_controller/linear_actuator_controller.ino`
- `hardware/limit sensors/limit_sensor_reader/limit_sensor_reader.ino`
- `hardware/limit sensors/servo_angle_test/servo_angle_test.ino`
- `hardware/magnetic encoder/as5600_example/README.md`
- `hardware/magnetic encoder/pendulum_angle_reader/pendulum_angle_reader.ino`
- `hardware/cad/stl/arduino mount.stl`
- `hardware/cad/stl/bearing mount.stl`
- `hardware/cad/stl/encoder mount.stl`
- `hardware/cad/stl/pendulum hinge.stl`
- `hardware/cad/stl/weight attachment.stl`
- `hardware/cad/stl/weight attachment - threaded.stl`
- `hardware/cad/reference/linear actuator stage holes.jpg`

Current modeling and control files:

- `modeling/pendulum_actuator_derivation/README.md`
- `modeling/pendulum_actuator_derivation/parameters.md`
- `modeling/pendulum_actuator_derivation/inverted pendulum derivation.pdf`
- `modeling/pendulum_actuator_derivation/lqr_first_pass.m`
- `modeling/pendulum_actuator_derivation/lqr_first_pass_results.md`
- `modeling/pendulum_actuator_derivation/LQR_closed_loop_response.fig`
- `modeling/pendulum_actuator_derivation/LQR_closed_loop_response.png`
- `modeling/pendulum_actuator_derivation/torque_diagram.svg`

Current host software files:

- `software/host/README.md`
- `software/host/requirements.txt`
- `software/host/main.py`
- `software/host/serial_worker.py`
- `software/host/arduino_actuator.py`
- `software/host/arduino_limits.py`
- `software/host/arduino_encoder.py`
- `software/host/motion_control.py`
- `software/host/travel_calibration.py`

The linear actuator calibration notebook fits the measured actuator data and gives a command equation for converting desired speed into the required delay:

```text
delay = 58399.75215056 / (speed + 0.00022846) - 3.82116746
```

The host-side travel calibration uses the measured raw-step travel scale:

```text
steps_per_mm = 10.652
position_mm = position_steps / 10.652
target_steps = round(target_mm * 10.652)
```

3D-printable solid model exports should go in:

- `hardware/cad/stl/`

Editable CAD source files, if used, should go in:

- `hardware/cad/source/`

CAD reference photos and measurements should go in:

- `hardware/cad/reference/`

## Software

Planned software stack:

- LabVIEW, Python, MATLAB, or another suitable tool for the main control interface and experiment workflow
- Arduino firmware split across three boards: actuator motion, limit sensors, and magnetic encoder sensing
- MATLAB, Python, or similar tools for modeling, simulation, and LQR design as needed

## Control Approach

The initial controller will be based on Linear Quadratic Regulator (LQR) design. The expected workflow is:

1. Derive or identify the system dynamics.
2. Linearize the model around the operating point of interest.
3. Choose state and input cost matrices.
4. Compute the LQR gain.
5. Test the controller in simulation.
6. Implement the controller through LabVIEW and Arduino.
7. Tune and validate on the physical system.

## Repository Structure

This repository is still being organized. A likely structure is:

```text
.
+-- hardware/
|   +-- cad/
|   |   +-- reference/  # CAD reference photos and measurements
|   |   +-- source/  # Editable CAD files
|   |   `-- stl/     # 3D-printable STL exports
|   +-- linear actuator calibration/
|   |   +-- actuator_speed_fit.ipynb
|   |   +-- calibration_procedure.md
|   |   +-- linear actuator speed calibration.xlsx
|   |   `-- calibrate_linear_actuator/
|   |       `-- calibrate_linear_actuator.ino
|   +-- linear actuator control/
|   |   `-- linear_actuator_controller/
|   |       +-- README.md
|   |       `-- linear_actuator_controller.ino
|   +-- limit sensors/
|   |   `-- limit_sensor_reader/
|   |       +-- README.md
|   |       `-- limit_sensor_reader.ino
|   `-- magnetic encoder/
|       +-- as5600_example/
|       |   `-- README.md
|       `-- pendulum_angle_reader/
|           +-- README.md
|           `-- pendulum_angle_reader.ino
+-- software/        # Control interface code, such as LabVIEW, Python, or MATLAB
+-- modeling/        # Models, simulations, and controller design scripts
`-- README.md
```

## Current Status

The project has moved well beyond initial setup. The linear actuator speed calibration data has been collected, and a Python notebook now fits the calibration data to produce a desired-speed-to-delay command equation. The host-side raw-step travel calibration has been collected and gives a default scale of `10.652 steps/mm`. The Arduino sketch used for collecting the actuator speed calibration data is stored with the calibration files. Initial 3D-printable STL exports for the printed hardware have been added under `hardware/cad/stl/`, and the pendulum has been assembled with the printed parts. Both optical actuator limit sensors have been tested with Arduino inputs, and the Arduino #1 actuator controller firmware has been tested. The limit sensor reader firmware and magnetic encoder reader firmware are complete for the current three-Arduino setup, and Arduino #2 now also supports a simple latch-servo interface for upright staging. The actuator limit sensor wire harness has been built, and the magnetic encoder Arduino mount has been created. A first-pass point-mass inverted pendulum model has been derived, and MATLAB LQR simulations have been run for both inverted and downward equilibria. The Python host software now supports homing, absolute moves, encoder capture, downward control experiments, upright control experiments, experiment logging, analysis notebooks, and a servo-assisted upright staging loop. On hardware, the system now demonstrates repeatable downward recovery and first-pass upright balancing for several seconds on successful runs. Bench testing also established that the current actuator hardware can reliably reach about `315 mm/s` without stalling, and the upright host defaults now reflect that measured limit.

Current bench setup photo:

- `docs/images/assembled_pendulum_bench_setup_2026-04-17.jpg`

Project notes are tracked in:

- `docs/project_log.md`

## Next Steps

- Improve upright balance duration and reduce cart drift during inverted balancing runs.
- Make upright start-up more repeatable, including angle-zeroing workflow and mechanical alignment.
- Refine the encoder mounting and magnet alignment to reduce transient angle estimation errors.
- Revisit the upright model and LQR tuning after any pendulum geometry changes.
- Add better documentation for the experiment workflow, analysis notebooks, and best-known controller settings.
- Add editable CAD source files if they are available.
