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

## Layout

This folder is organized into:

- `docs/` - shared derivation notes, parameter notes, and reference figures
- `scripts/` - MATLAB analysis scripts
- `results/` - generated figures, CSV summaries, and per-analysis notes

Shared modeling notes:

- `docs/parameters.md`
- `docs/inverted pendulum derivation.pdf`
- `docs/torque_diagram.svg`

MATLAB scripts:

- `scripts/lqr_first_pass.m` - upright-equilibrium LQR first pass
- `scripts/lqr_downward_first_pass.m` - downward-equilibrium LQR first pass for early bench damping tests
- `scripts/lqr_upright_length_qr_sweep.m` - upright-equilibrium sweep across pendulum length and several Q/R choices

Saved results:

- `results/upright_first_pass/lqr_first_pass_results.md`
- `results/upright_first_pass/LQR_closed_loop_response.fig`
- `results/upright_first_pass/LQR_closed_loop_response.png`
- `results/downward_first_pass/lqr_downward_first_pass_results.md`
- `results/downward_first_pass/LQR_downward_closed_loop_response.fig`
- `results/downward_first_pass/LQR_downward_closed_loop_response.png`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_summary.csv`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_theta.fig`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_theta.png`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_x.fig`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_x.png`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_tradeoffs.fig`
- `results/upright_length_qr_sweep/lqr_upright_length_qr_sweep_tradeoffs.png`

Recent parameter-validation notes:

- `hardware/magnetic encoder/captures/pendulum 1_period_test_20260419_093442.csv`
- `hardware/magnetic encoder/analysis/analyze_pendulum_1_period_test.ipynb`

The pendulum-1 free-swing period test gave an average period of about `1.42 s`, which corresponds to an effective simple-pendulum length of about `0.50 m`. That result is close to the `0.510 m` length already used in the first-pass MATLAB LQR scripts, so the original center-of-mass estimate remains a reasonable modeling value for now.
