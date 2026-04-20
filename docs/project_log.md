# Project Log

## 2026-04-19

Validated the pendulum-1 center-of-mass length with a small-angle free-swing period test and documented the result.

Completed:

- Added a dedicated host-side period-test capture command:
  - `python -m software.host.main --encoder-port COM8 period-test`
  - `software/host/main.py`
  - `software/host/encoder_capture.py`
- Collected a pendulum-1 period-test capture:
  - `hardware/magnetic encoder/captures/pendulum 1_period_test_20260419_093442.csv`
- Added a period-test analysis notebook:
  - `hardware/magnetic encoder/analysis/analyze_pendulum_1_period_test.ipynb`
- Updated the host, modeling, and top-level documentation to record the measurement and its agreement with the first-pass MATLAB LQR parameter.

Notes:

- The pendulum-1 free-swing period test gave an average measured period of about `1.42 s`.
- Under the simple-pendulum assumption, that implies an effective pendulum length of about `0.500 m`.
- The first-pass MATLAB LQR work used `l = 0.510 m`, so the period test agrees well with the original tape-measure estimate and supports keeping that parameter for now.
- The analysis notebook was corrected to estimate the full period from same-sign extrema after an earlier half-period mistake temporarily produced an incorrect `0.125 m` estimate.

Next:

- If the pendulum geometry changes, repeat the same period-test workflow and update the model parameter if needed.
- Keep using `l = 0.510 m` as the current first-pass modeling value unless later hardware changes justify revising it.

## 2026-04-18

Added host-side magnetic encoder capture tooling and collected initial bench data.

Completed:

- Added an interactive encoder capture command:
  - `python -m software.host.main --encoder-port COM8 capture-encoder`
  - `software/host/encoder_capture.py`
- Collected AS5600 free-swing and manual held-angle captures:
  - `hardware/magnetic encoder/captures/encoder_capture_20260418_112254.csv`
  - `hardware/magnetic encoder/captures/encoder_capture_20260418_113008.csv`
  - `hardware/magnetic encoder/captures/encoder_capture_20260418_113607.csv`
  - `hardware/magnetic encoder/captures/encoder_capture_results.md`
- Added and ran a downward-equilibrium LQR simulation for early bench damping tests:
  - `modeling/pendulum_actuator_derivation/scripts/lqr_downward_first_pass.m`
  - `modeling/pendulum_actuator_derivation/results/downward_first_pass/lqr_downward_first_pass_results.md`
  - `modeling/pendulum_actuator_derivation/results/downward_first_pass/LQR_downward_closed_loop_response.fig`
  - `modeling/pendulum_actuator_derivation/results/downward_first_pass/LQR_downward_closed_loop_response.png`

Notes:

- The encoder capture workflow recorded about `100 Hz` data and showed good return-to-zero behavior near the downward resting position.
- Larger manual angle checks showed asymmetric response. A roughly `45 deg` side-to-side test reported about `-47 deg` on one side and about `+89 deg` on the other.
- A likely mechanical cause is that the magnet is currently sitting on the pendulum axle bolt head and may not be flat or centered relative to the AS5600.
- The AS5600 magnetic diagnostics still reported weak-field behavior even while angle data was smooth enough for qualitative testing.
- The downward LQR gain was `K = [0.3162, 0.8139, -0.3088, -0.3479]`, with peak simulated commanded acceleration `0.112 m/s^2` and peak cart displacement `0.027 m`.

Next:

- Design a more dimensionally stable magnet/sensor alignment method for the AS5600.
- Repeat encoder held-angle validation after improving magnet alignment.
- Continue toward the fixed-rate hardware control loop and state-estimation software tasks.

## 2026-04-17

Continued hardware bring-up for the sensing electronics.

Completed:

- Created the mount for the magnetic encoder Arduino:
  - `hardware/cad/stl/arduino mount.stl`
- Built the wire harness for the actuator limit sensors.
- Marked the optical limit sensor reader firmware complete for the current setup.
- Marked the AS5600 magnetic encoder reader firmware complete for the current setup.
- Added first-pass pendulum-actuator modeling notes:
  - `modeling/pendulum_actuator_derivation/README.md`
  - `modeling/pendulum_actuator_derivation/docs/parameters.md`
  - `modeling/pendulum_actuator_derivation/docs/inverted pendulum derivation.pdf`
  - `modeling/pendulum_actuator_derivation/docs/torque_diagram.svg`
- Added and ran an initial MATLAB LQR simulation:
  - `modeling/pendulum_actuator_derivation/scripts/lqr_first_pass.m`
  - `modeling/pendulum_actuator_derivation/results/upright_first_pass/lqr_first_pass_results.md`
  - `modeling/pendulum_actuator_derivation/results/upright_first_pass/LQR_closed_loop_response.fig`
  - `modeling/pendulum_actuator_derivation/results/upright_first_pass/LQR_closed_loop_response.png`
- Added an initial Python host software scaffold:
  - `software/host/README.md`
  - `software/host/requirements.txt`
  - `software/host/main.py`
  - `software/host/serial_worker.py`
  - `software/host/arduino_actuator.py`
  - `software/host/arduino_limits.py`
  - `software/host/arduino_encoder.py`
- Added basic host-side actuator motion commands with limit-sensor checks:
  - `software/host/motion_control.py`
- Organized the updated current bench setup photo:
  - `docs/images/assembled_pendulum_bench_setup_2026-04-17.jpg`

Notes:

- The current sensing firmware remains split across Arduino #2 for the optical limit sensors and Arduino #3 for the AS5600 magnetic encoder.
- The first-pass model treats the pendulum as a point mass with \(l = 0.510\ \text{m}\), measured from the pivot to the rough center of mass of the bob.
- The initial LQR simulation used \(Q = \operatorname{diag}(1,\ 0.1,\ 50,\ 1)\) and \(R = 1\). The computed gain was \(K = [-1.0000,\ -2.0104,\ -29.1450,\ -6.5238]\).
- The initial closed-loop simulation had peak commanded acceleration \(2.543\ \text{m/s}^2\) and peak cart displacement \(0.228\ \text{m}\). With approximately \(0.60\ \text{m}\) total actuator travel, the displacement is within the nominal centered travel range but leaves limited margin.
- The initial host software uses one background serial worker per Arduino interface. The actuator interface includes command helpers, while the limit sensor and encoder interfaces continuously parse the latest streamed sample.
- The Python host monitor was tested with the current bench COM-port assignment: actuator on `COM6`, limit sensors on `COM10`, and encoder on `COM8`. It successfully read live status/data from all three Arduinos, and `Ctrl+C` shut down the monitor as expected.
- The host motion layer uses the measured actuator travel calibration `10.652 steps/mm`, or about `0.093879 mm/step`, and the total travel estimate remains about `600 mm`. Negative motion is assumed to travel toward the left limit, and positive motion toward the right limit.
- A cautious live host motion smoke test worked as expected using `python -m software.host.main --actuator-port COM6 --limits-port COM10 speed 5 --duration 1`.
- Direction signs were confirmed. Positive host speed moves away from the left limit, and negative host speed moves toward the left limit. A negative speed command was correctly refused while the left limit was already active.
- Left homing while already at the left limit correctly set the actuator position estimate to zero, and a subsequent `move-mm 50 --speed-mm-s 10` command moved off the left limit and stopped as expected.
- The interactive host motion shell was tested with `home left 10`, `status`, `move 50 10`, `status`, and `quit` in one continuous serial session. The final status reported `position_steps = 532` and `position_mm = 50.008`.
- After updating the host to disable the actuator driver after completed `home` and `move` commands, the shell workflow was repeated and ended with `enabled = False`, `motion_mode = 0`, `position_steps = 532`, and `position_mm = 50.008`.
- After a right-limit test reported `position_mm` very close to the provisional `600 mm` travel estimate, the host and actuator firmware were adjusted so Arduino #1 only handles raw step and step-rate commands while millimeter conversion remains on the host side. The host now includes a raw `move-steps` command and `steps` shell command for calibration work.
- Arduino #1 raw step commands were verified through the Arduino Serial Monitor after re-uploading the updated firmware. The Python host raw-step shell command was also verified: `steps 100 100` increased `position_steps` from `0` to `100`, and `steps -100 100` returned it to `0`.
- The updated current bench setup photo shows the full actuator rail with the assembled pendulum carriage, limit-sensor hardware, encoder mount, wire harness, and three-Arduino electronics layout.
- Added an interactive host travel calibration routine:
  - `software/host/travel_calibration.py`
- An initial travel-calibration attempt exposed a stale actuator-status issue after homing. The host now waits for Arduino #1 to report `position_steps = 0` after left homing before continuing to the first calibration move. Empty CSV files from the interrupted attempts were not kept.
- Updated the host travel calibration routine so each speed is selected interactively, the 100-step tape reading is confirmed before every move, and completed data is saved after each requested speed set.
- Collected host travel calibration data:
  - `hardware/linear actuator calibration/host_travel_calibration_20260417_194546.csv`
  - `hardware/linear actuator calibration/host_travel_calibration_results.md`
- Averaged the completed calibration rows to set the host default actuator scale to `10.652 steps/mm`.

Next:

- Verify clearances, alignment, range of motion, and mechanical stiffness on the assembled pendulum.
- Add editable CAD source files if available.
- Verify the encoder signal through the installed magnet/shaft geometry.
- Refine physical parameter estimates for the pendulum and actuator.
- Compare first-pass LQR acceleration commands against actuator limits.
- Tune and validate the LQR controller in simulation.
- Bench-test host-side homing and absolute-position moves.

## 2026-04-16

Added initial printable hardware assets and CAD reference material.

Completed:

- Added 3D-printable STL exports under `hardware/cad/stl/`:
  - `hardware/cad/stl/bearing mount.stl`
  - `hardware/cad/stl/encoder mount.stl`
  - `hardware/cad/stl/pendulum hinge.stl`
  - `hardware/cad/stl/weight attachment.stl`
  - `hardware/cad/stl/weight attachment - threaded.stl`
- Added a linear actuator stage hole-pattern reference photo:
  - `hardware/cad/reference/linear actuator stage holes.jpg`
- Updated the README with the current CAD file list and the `hardware/cad/reference/` convention.
- Resolved the local documentation next-step item to add initial CAD/STL files for the printed hardware structure.
- Added a placeholder folder for AS5600 magnetic encoder Arduino example code:
  - `hardware/magnetic encoder/as5600_example/`
- Added AS5600 pendulum angle reader firmware:
  - `hardware/magnetic encoder/pendulum_angle_reader/pendulum_angle_reader.ino`
- Added initial optical limit sensor reader firmware:
  - `hardware/limit sensors/limit_sensor_reader/limit_sensor_reader.ino`
- Added Arduino #1 linear actuator controller firmware:
  - `hardware/linear actuator control/linear_actuator_controller/linear_actuator_controller.ino`
- Organized the current bench setup photo:
  - `docs/images/assembled_pendulum_bench_setup_2026-04-16.jpg`
- Tested the AS5600 magnetic encoder example with Arduino. The encoder connects over I2C and returns smooth angle readings.
- Assembled the pendulum using the 3D printed parts made today.
- Tested both optical actuator limit sensors with Arduino. Brown is wired to `5V`, blue to `GND`, black to the Arduino input, and white is disconnected for the current setup.
- Tested the Arduino #1 linear actuator controller firmware successfully.

Notes:

- Current hardware I/O is split across three Arduinos: Arduino #1 controls actuator motion, Arduino #2 reads the optical limit sensors, and Arduino #3 reads the AS5600 magnetic encoder.
- The new STL files are printable exports. Editable CAD source files still need to be added if they are available.
- The AS5600 example sketch is stored under `hardware/magnetic encoder/as5600_example/AS5600_demo/`.
- The pendulum angle reader streams CSV angle data at `115200` baud and supports Serial commands for zeroing and wrap-tracking reset.
- The limit sensor reader uses brown to Arduino `5V`, blue to `GND`, black as the Arduino input signal, and white disconnected. The two tested limit sensor inputs are Arduino `D2` and `D3`.
- The linear actuator controller uses `D10` for step, `D11` for direction, and `D12` for active-low enable. It supports serial commands for enable, stop, relative moves, signed step rate, and signed calibrated actuator speed.
- The current bench setup photo shows the assembled actuator and pendulum hardware with the in-progress three-Arduino layout.
- During AS5600 testing, the status bits reported `magnet_detected = 0` and `magnet_too_weak = 1` even while angle readings were smooth. The observed diagnostic values included `AGC = 255` and magnitude around `279`, so the magnet-strength flags should be treated as diagnostics until the mechanical magnet setup is finalized.
- VS Code GitHub CLI commands need to be run outside the sandboxed shell to use the Windows keyring credentials.
- GitHub issue #10, `Assemble pendulum with 3D printed parts`, is closed.

Next:

- Verify clearances, alignment, range of motion, and mechanical stiffness on the assembled pendulum.
- Add editable CAD source files if available.
- Verify the encoder signal through the installed magnet/shaft geometry.

## 2026-04-14

Initial project organization and calibration setup.

Completed:

- Created the initial project README.
- Added a `hardware/` folder for hardware-related files.
- Organized linear actuator calibration files under `hardware/linear actuator calibration/`.
- Added the linear actuator speed calibration spreadsheet.
- Added the Arduino sketch used to collect linear actuator calibration data:
  - `hardware/linear actuator calibration/calibrate_linear_actuator/calibrate_linear_actuator.ino`
- Added a Python/Jupyter notebook to extract calibration data and fit the actuator command equation:
  - `hardware/linear actuator calibration/actuator_speed_fit.ipynb`
- Documented the linear actuator calibration procedure:
  - `hardware/linear actuator calibration/calibration_procedure.md`
- Fit the actuator speed-to-delay command equation:

```text
delay = 58399.75215056 / (speed + 0.00022846) - 3.82116746
```

- Updated the README with the calibration files, command equation, and planned hardware/CAD organization.
- Installed and authenticated GitHub CLI.
- Created GitHub milestones:
  - Project Setup
  - Linear Actuator Calibration
  - Hardware Bring-Up
  - Pendulum Sensing
  - System Modeling and LQR
  - Software-Hardware Integration
  - Final Demo and Documentation
- Committed and pushed the initial project setup and calibration files.

Notes:

- 3D-printable STL files should go in `hardware/cad/stl/`.
- Editable CAD source files should go in `hardware/cad/source/`.
- CAD reference photos and measurements should go in `hardware/cad/reference/`.
- PowerShell currently prints a profile script warning because script execution is disabled, but Git and GitHub CLI are working.

Next:

- Create initial GitHub issues under the new milestones.
- Document the current physical hardware setup and wiring.
- Verify the actuator speed-to-delay equation on hardware.
- Decide on the pendulum angle sensing approach.

## 2026-04-18

Downward control tuning reached a usable first-pass baseline on hardware.

Completed:

- Added a host-side `control-down` experiment for the downward pendulum case.
- Added a downward-equilibrium LQR MATLAB script and documented the first-pass gain.
- Added host-side sign handling for actuator command, cart position, and encoder angle conventions.
- Added an `alpha-beta` theta/theta-dot estimator with separate pre-arm and armed tuning.
- Added trigger/disarm logic so the cart stays quiet when the pendulum is still, engages on real perturbations, and returns to zero command after the motion settles.
- Tuned the current baseline around repeated bench tests so the cart can recover from multiple manual perturbations without running away or chattering continuously.

Notes:

- Current best-so-far downward control runs showed the cart returning near center and re-engaging after repeated perturbations.
- Residual oscillation remains after control disarms, but this is now a secondary tuning issue rather than a basic sign or startup-instability problem.
- The current baseline command is:

```text
python -m software.host.main --actuator-port COM6 --limits-port COM10 --encoder-port COM8 control-down --invert-actuator-command --invert-cart-position --period-s 0.02
```

Next:

- Preserve this downward-control baseline in git before making larger changes.
- Revisit low-amplitude residual oscillation and tail-phase overreaction with smaller, targeted adjustments.
- Use the downward case as the host/control baseline before moving on to more ambitious inverted-pendulum work.

## 2026-04-18

Extended the upright-control workflow with documentation, analysis overlays, servo-assisted staging, and a measured actuator speed limit.

Completed:

- Added an upright analysis notebook and overlay plots against the current best run:
  - `hardware/control experiments/analysis/analyze_upright_control.ipynb`
- Updated top-level documentation to feature the upright demo video and current upright-control workflow:
  - `README.md`
  - `docs/media/upright_balance_run_2026-04-18.mp4`
- Updated the host and firmware for Arduino #2 to support a simple latch servo:
  - `hardware/limit sensors/limit_sensor_reader/limit_sensor_reader.ino`
  - `hardware/limit sensors/servo_angle_test/servo_angle_test.ino`
  - `software/host/arduino_limits.py`
- Updated `control-up` to use a staging loop after homing/middle:
  - `p` to hold the pendulum with the servo
  - `g` to release the servo
  - `start` to zero the encoder, release the servo, and begin automatic control
- Added a staging-loop diagnostic that reports time-to-fall past `10 deg` after each servo release.
- Bench-tested the actuator speed ceiling and reduced the upright host default speed clamp to the measured no-stall limit of `315 mm/s`.
- Hardened the serial reader so a malformed encoder line no longer kills the host-side reader thread.

Notes:

- The best upright run still remains the earlier manually staged run at about `6.75 s`; the servo-assisted workflow improved repeatability of the start condition but has not yet improved the best achieved balance duration.
- Recent poor upright runs appear more consistent with startup/release transients or physical actuator limitations than with a sign error or a dramatic observer failure.
- Current upright control remains most sensitive to initial condition, release repeatability, and real actuator authority.

Next:

- Compare servo-assisted starts directly against manual starts to separate release-transient effects from controller limitations.
- Consider mechanical changes that slow the upright dynamics or reduce release disturbance, including pendulum length and latch geometry.
- Use the new staging-loop `g` timing metric to quantify release repeatability before further controller changes.

## 2026-04-19

Moved the host-side pendulum control workflow away from fixed MATLAB-exported gains and toward Python-side LQR synthesis tied directly to the pendulum length and requested cost weights.

Completed:

- Added Python LQR synthesis inside the host controller using the same linearized model structure as the MATLAB first-pass scripts:
  - `software/host/pendulum_control.py`
  - `software/host/requirements.txt`
- Exposed on-demand LQR tuning from the CLI for both `control-down` and `control-up`:
  - `--lqr-mode {python,manual}`
  - `--pendulum-length-m`
  - `--q-x`
  - `--q-x-dot`
  - `--q-theta`
  - `--q-omega`
  - `--r-input`
  - `software/host/main.py`
- Kept compatibility with manual gain entry so hardware runs can still use explicit `K` values when desired.
- Increased the upright default speed clamp from `315 mm/s` to `325 mm/s` for further bench testing:
  - `software/host/main.py`
  - `software/host/pendulum_control.py`
  - `software/host/README.md`
- Added a startup guard that waits for a fresh encoder snapshot after arming so the control loop does not launch on stale pre-zero data:
  - `software/host/pendulum_control.py`
- Updated control CSV logging so every run records the full controller configuration as `config_*` columns:
  - LQR mode
  - pendulum length
  - `Q`, `R`
  - resolved gains
  - estimator settings
  - deadbands, clamps, trigger/disarm settings
  - sign conventions and setup flags
- Updated the upright analysis notebook so it prints the active log path and displays the `config_*` summary clearly near the top:
  - `hardware/control experiments/analysis/analyze_upright_control.ipynb`
- Updated project-facing docs to describe the new Python tuning workflow and self-describing control logs:
  - `README.md`
  - `software/host/README.md`

Notes:

- The host now defaults to computing `K` directly from the pendulum length and chosen `Q`, `R`, which removes the need to round-trip through MATLAB for routine gain changes.
- The default upright and downward Python synthesis settings were chosen to reproduce the earlier first-pass MATLAB gains closely, so the tuning baseline remains continuous with prior work.
- The new `config_*` fields make later notebook analysis much easier because the run settings now travel with the data.

Next:

- Re-test upright control once the encoder and magnet hardware rework is installed.
- Use the logged `config_*` fields to compare successful and failed runs without relying on terminal notes.
- Decide whether to add a hard startup refusal when encoder magnetic diagnostics indicate that the magnet is missing or too weak.
