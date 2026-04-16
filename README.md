# Pendulum Optimal Control Demonstration

This project is a starting point for an optimal control demonstration using a pendulum mounted to a linear actuator. The first control target is an LQR controller, with the long-term goal of stabilizing the pendulum in the inverted position if the hardware and sensing setup allow it.

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
- Arduino
- Position and/or angle sensors
- Computer running LabVIEW

Current hardware-related files:

- `hardware/linear actuator calibration/calibration_procedure.md`
- `hardware/linear actuator calibration/linear actuator speed calibration.xlsx`
- `hardware/linear actuator calibration/actuator_speed_fit.ipynb`
- `hardware/linear actuator calibration/calibrate_linear_actuator/calibrate_linear_actuator.ino`
- `hardware/limit sensors/limit_sensor_reader/limit_sensor_reader.ino`
- `hardware/magnetic encoder/as5600_example/README.md`
- `hardware/magnetic encoder/pendulum_angle_reader/pendulum_angle_reader.ino`
- `hardware/cad/stl/bearing mount.stl`
- `hardware/cad/stl/encoder mount.stl`
- `hardware/cad/stl/pendulum hinge.stl`
- `hardware/cad/stl/weight attachment.stl`
- `hardware/cad/stl/weight attachment - threaded.stl`
- `hardware/cad/reference/linear actuator stage holes.jpg`

The linear actuator calibration notebook fits the measured actuator data and gives a command equation for converting desired speed into the required delay:

```text
delay = 58399.75215056 / (speed + 0.00022846) - 3.82116746
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
- Arduino firmware for hardware I/O
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

Early project setup. The linear actuator speed calibration data has been collected, and a Python notebook now fits the calibration data to produce a desired-speed-to-delay command equation. The Arduino sketch used for collecting the actuator calibration data is stored with the calibration files. Initial 3D-printable STL exports for the printed hardware have been added under `hardware/cad/stl/`, and the pendulum has been assembled with the printed parts. Both optical actuator limit sensors have been tested with Arduino inputs.

Project notes are tracked in:

- `docs/project_log.md`

## Next Steps

- Document the physical setup and available sensors.
- Decide what states will be measured or estimated.
- Add editable CAD source files if they are available.
- Verify clearances, alignment, range of motion, and mechanical stiffness on the assembled pendulum.
- Verify the encoder signal through the installed magnet/shaft geometry.
- Build a robust wire harness for the actuator limit sensors.
- Create an initial pendulum model.
- Design and simulate the first LQR controller.
- Define the software-Arduino communication protocol.
