# Linear Actuator Calibration Procedure

This procedure describes how the linear actuator speed calibration data was collected.

## Purpose

The goal of this calibration is to estimate the relationship between the commanded step delay and the resulting actuator speed. The fitted relationship is used to convert a desired actuator speed into the delay value needed by the Arduino stepper motor control code.

## Hardware Setup

The calibration setup used:

- Linear actuator
- Microstep driver
- Arduino
- External power source for the actuator/driver
- Basic Arduino stepper motor demo code for initial motion testing
- Calibration Arduino sketch:
  - `calibrate_linear_actuator/calibrate_linear_actuator.ino`

## Procedure

1. Set up the linear actuator, microstep driver, Arduino, and power source.
2. Use a basic Arduino stepper motor demo file to command the actuator to move `5000` steps.
3. Benchmark the physical distance traveled during the `5000` step move.
4. Use the calibration Arduino file to run repeated `5000` step moves at different delay values.
5. Record the number of milliseconds required for each `5000` step run.
6. Record the measured travel distance, command delay values, elapsed times, and calculated speeds in the Excel calibration file:
   - `linear actuator speed calibration.xlsx`
7. Use the calibration notebook to fit the measured data:
   - `actuator_speed_fit.ipynb`

## Calibration Data

The Excel file stores the measured data used for the fit. The key columns are:

- `speed`: calculated actuator speed for the run
- `delay`: command delay used by the Arduino stepper control code
- `ms`: measured elapsed time for the `5000` step move

The calibration sheet also records:

- Travel distance: `470 mm`
- Commanded move: `5000 steps`

## Fitted Command Equation

The notebook fits the calibration data and provides this command equation:

```text
delay = 58399.75215056 / (speed + 0.00022846) - 3.82116746
```

Use this equation to convert a desired actuator speed into the delay value sent to the Arduino.

## Notes

- The equation should only be trusted over the speed and delay range covered by the calibration data.
- If the actuator, driver settings, power supply, load, belt drive, or microstepping settings change, the calibration should be repeated.
- The calibration data was collected using `5000` step moves, so future verification tests should start with the same move length before changing the test setup.
