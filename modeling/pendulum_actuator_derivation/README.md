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

- `lqr_first_pass.m`
- `lqr_first_pass_results.md`
