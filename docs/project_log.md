# Project Log

## 2026-04-16

Added initial printable hardware assets and CAD reference material.

Completed:

- Added 3D-printable STL exports under `hardware/cad/stl/`:
  - `hardware/cad/stl/bearing mount.stl`
  - `hardware/cad/stl/encoder mount.stl`
  - `hardware/cad/stl/pendulum hinge.stl`
  - `hardware/cad/stl/weight attachment.stl`
- Added a linear actuator stage hole-pattern reference photo:
  - `hardware/cad/reference/linear actuator stage holes.jpg`
- Updated the README with the current CAD file list and the `hardware/cad/reference/` convention.
- Resolved the local documentation next-step item to add initial CAD/STL files for the printed hardware structure.

Notes:

- The new STL files are printable exports. Editable CAD source files still need to be added if they are available.
- GitHub issue review could not be completed from this shell because `gh issue list` returned `HTTP 401: Requires authentication`.

Next:

- Print and test-fit the bearing mount, encoder mount, pendulum hinge, and weight attachment.
- Add editable CAD source files if available.
- Re-authenticate GitHub CLI if issue management is needed from this machine.

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
