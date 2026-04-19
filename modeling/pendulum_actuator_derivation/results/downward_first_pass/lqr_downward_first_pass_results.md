# First-Pass Downward LQR Simulation Results

Simulation script:

- `../../scripts/lqr_downward_first_pass.m`

Saved response figure:

- `LQR_downward_closed_loop_response.fig`
- `LQR_downward_closed_loop_response.png`

## Model

The simulation used the same simplified point-mass model as the upright first pass, with actuator acceleration input:

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

For this downward-equilibrium controller, \(\theta = 0\) is the hanging-down position.

Measured pendulum length:

\[
l = 0.510\ \text{m}
\]

## Linearized Downward Model

Compared with the upright model, the gravity stiffness term changes sign because gravity is restoring about the downward equilibrium:

\[
A_{4,3} = -\frac{g}{l}
\]

The input matrix keeps the same base-acceleration sign convention:

\[
B_4 = -\frac{1}{l}
\]

## LQR Weights

\[
Q = \operatorname{diag}(1,\ 0.1,\ 5,\ 1)
\]

\[
R = 10
\]

These weights are deliberately gentler than the upright first-pass controller because the downward pendulum is already passively stable.

## MATLAB Output

Downward LQR gain:

\[
K =
\begin{bmatrix}
0.3162 & 0.8139 & -0.3088 & -0.3479
\end{bmatrix}
\]

Closed-loop eigenvalues:

\[
\lambda(A-BK) =
\begin{bmatrix}
-0.3478 + 4.3777i \\
-0.3478 - 4.3777i \\
-0.4002 + 0.3940i \\
-0.4002 - 0.3940i
\end{bmatrix}
\]

Simulation summary:

- Peak commanded acceleration: \(0.112\ \text{m/s}^2\)
- Peak cart displacement: \(0.027\ \text{m}\)

## Notes

- The response is lightly damped, which is expected for a conservative controller around the naturally stable downward equilibrium.
- The angle oscillates and decays toward the downward equilibrium over the simulated `12 s` window.
- The required cart displacement and acceleration are much smaller than the upright first-pass simulation.
- This controller is useful for early bench damping tests, but it is not the final inverted-pendulum LQR controller.
