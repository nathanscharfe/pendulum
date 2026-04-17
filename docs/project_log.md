# Project Log

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
  - `modeling/pendulum_actuator_derivation/parameters.md`
  - `modeling/pendulum_actuator_derivation/inverted pendulum derivation.pdf`
  - `modeling/pendulum_actuator_derivation/torque_diagram.svg`
- Added and ran an initial MATLAB LQR simulation:
  - `modeling/pendulum_actuator_derivation/lqr_first_pass.m`
  - `modeling/pendulum_actuator_derivation/lqr_first_pass_results.md`
  - `modeling/pendulum_actuator_derivation/LQR_closed_loop_response.fig`
  - `modeling/pendulum_actuator_derivation/LQR_closed_loop_response.png`
- Added an initial Python host software scaffold:
  - `software/host/README.md`
  - `software/host/requirements.txt`
  - `software/host/main.py`
  - `software/host/serial_worker.py`
  - `software/host/arduino_actuator.py`
  - `software/host/arduino_limits.py`
  - `software/host/arduino_encoder.py`

Notes:

- The current sensing firmware remains split across Arduino #2 for the optical limit sensors and Arduino #3 for the AS5600 magnetic encoder.
- The first-pass model treats the pendulum as a point mass with \(l = 0.510\ \text{m}\), measured from the pivot to the rough center of mass of the bob.
- The initial LQR simulation used \(Q = \operatorname{diag}(1,\ 0.1,\ 50,\ 1)\) and \(R = 1\). The computed gain was \(K = [-1.0000,\ -2.0104,\ -29.1450,\ -6.5238]\).
- The initial closed-loop simulation had peak commanded acceleration \(2.543\ \text{m/s}^2\) and peak cart displacement \(0.228\ \text{m}\). With approximately \(0.60\ \text{m}\) total actuator travel, the displacement is within the nominal centered travel range but leaves limited margin.
- The initial host software uses one background serial worker per Arduino interface. The actuator interface includes command helpers, while the limit sensor and encoder interfaces continuously parse the latest streamed sample.
- The Python host monitor was tested with the current bench COM-port assignment: actuator on `COM6`, limit sensors on `COM10`, and encoder on `COM8`. It successfully read live status/data from all three Arduinos, and `Ctrl+C` shut down the monitor as expected.

Next:

- Verify clearances, alignment, range of motion, and mechanical stiffness on the assembled pendulum.
- Add editable CAD source files if available.
- Verify the encoder signal through the installed magnet/shaft geometry.
- Refine physical parameter estimates for the pendulum and actuator.
- Compare first-pass LQR acceleration commands against actuator limits.
- Tune and validate the LQR controller in simulation.
- Define the full software-Arduino control-loop protocol.

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
