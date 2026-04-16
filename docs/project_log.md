# Project Log

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
- Tested the AS5600 magnetic encoder example with Arduino. The encoder connects over I2C and returns smooth angle readings.
- Assembled the pendulum using the 3D printed parts made today.

Notes:

- The new STL files are printable exports. Editable CAD source files still need to be added if they are available.
- The AS5600 example sketch is stored under `hardware/magnetic encoder/as5600_example/AS5600_demo/`.
- The pendulum angle reader streams CSV angle data at `115200` baud and supports Serial commands for zeroing and wrap-tracking reset.
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
