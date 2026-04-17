# First-Pass LQR Simulation Results

GitHub issue: #20, `Derive initial pendulum-actuator state-space model`

Simulation script:

- `lqr_first_pass.m`

Saved response figure:

- `LQR_closed_loop_response.fig`
- `LQR_closed_loop_response.png`

## Model

The simulation used the simplified point-mass model with actuator acceleration input:

\[
u = \ddot{x}
\]

State vector:

\[
X =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
\]

Measured pendulum length:

\[
l = 0.510\ \text{m}
\]

## LQR Weights

\[
Q = \operatorname{diag}(1,\ 0.1,\ 50,\ 1)
\]

\[
R = 1
\]

## MATLAB Output

LQR gain:

\[
K =
\begin{bmatrix}
-1.0000 & -2.0104 & -29.1450 & -6.5238
\end{bmatrix}
\]

Closed-loop eigenvalues:

\[
\lambda(A-BK) =
\begin{bmatrix}
-4.7376 + 1.1311i \\
-4.7376 - 1.1311i \\
-0.6531 + 0.6199i \\
-0.6531 - 0.6199i
\end{bmatrix}
\]

Simulation summary:

- Peak commanded acceleration: \(2.543\ \text{m/s}^2\)
- Peak cart displacement: \(0.228\ \text{m}\)
- Approximate total actuator travel: \(0.60\ \text{m}\)
- Approximate one-sided travel from center: \(0.30\ \text{m}\)

## Notes

- All closed-loop eigenvalues have negative real parts, so the linearized closed-loop system is stable for this first-pass controller.
- The angle response settles back toward upright from the tested initial perturbation.
- The peak cart displacement is about \(0.228\ \text{m}\). With approximately \(0.30\ \text{m}\) one-sided travel from actuator center, this first simulation stays inside the nominal travel range but leaves limited margin.
- The peak commanded acceleration is about \(2.543\ \text{m/s}^2\). This should be compared against what the actuator can actually produce with the stepper driver and current command interface.
