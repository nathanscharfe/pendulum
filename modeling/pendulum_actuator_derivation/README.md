# Pendulum-Actuator Model Derivation

This folder tracks the derivation for the initial state-space model of the pendulum mounted on the linear actuator.

GitHub issue: #20, `Derive initial pendulum-actuator state-space model`

## Goals

- Define the coordinate system and sign conventions.
- Choose the state vector and control input.
- Derive the equations of motion.
- Linearize the model around the selected operating point.
- Put the result in state-space form for LQR design.

## Working Assumptions

- The actuator carriage/base motion is horizontal.
- The pendulum is modeled as a rigid body.
- The first model may neglect bearing friction, actuator compliance, and sensor delay unless those effects are needed later.
- The actuator command can be represented initially by a base acceleration or an equivalent commanded motion input.

## Notes

Add derivation notes, sketches, equations, and model parameters here as they are developed.

Current parameter notes:

- `parameters.md`

Current simulation notes:

- `lqr_first_pass.m` - upright-equilibrium LQR first pass
- `lqr_downward_first_pass.m` - downward-equilibrium LQR first pass for early bench damping tests
- `lqr_first_pass_results.md`
- `lqr_downward_first_pass_results.md`

Recent parameter-validation notes:

- `hardware/magnetic encoder/captures/pendulum 1_period_test_20260419_093442.csv`
- `hardware/magnetic encoder/analysis/analyze_pendulum_1_period_test.ipynb`

The pendulum-1 free-swing period test gave an average period of about `1.42 s`, which corresponds to an effective simple-pendulum length of about `0.50 m`. That result is close to the `0.510 m` length already used in the first-pass MATLAB LQR scripts, so the original center-of-mass estimate remains a reasonable modeling value for now.
