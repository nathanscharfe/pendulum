# Project Log

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
- PowerShell currently prints a profile script warning because script execution is disabled, but Git and GitHub CLI are working.

Next:

- Create initial GitHub issues under the new milestones.
- Document the current physical hardware setup and wiring.
- Verify the actuator speed-to-delay equation on hardware.
- Decide on the pendulum angle sensing approach.
