# Pendulum-Actuator Model Parameters

GitHub issue: #20, `Derive initial pendulum-actuator state-space model`

## Initial Point-Mass Model

For the first LQR model, treat the pendulum as a point mass located at the rough center of mass of the bob.

Measured pendulum length:

\[
l = 510\ \text{mm} = 0.510\ \text{m}
\]

Measurement definition:

- \(l\) is measured from the pendulum pivot to the rough center of mass of the bob.
- The current pendulum uses an available dowel, so this length is an initial hardware value rather than an optimized design choice.
- Later design iterations may change the pendulum length if simulations or hardware tests show that a longer or shorter pendulum would work better.

Period-test cross-check:

- `hardware/magnetic encoder/captures/pendulum 1_period_test_20260419_093442.csv`
- `hardware/magnetic encoder/analysis/analyze_pendulum_1_period_test.ipynb`
- Average measured free-swing period: about `1.42 s`
- Inferred simple-pendulum effective length: about `0.500 m = 500 mm`

This period-based estimate is within about `10 mm` of the original `510 mm` tape-measure estimate, so the first-pass LQR model parameter remains consistent with the current hardware measurement.

Gravity:

\[
g = 9.81\ \text{m/s}^2
\]

Actuator travel:

\[
x_{\text{travel,total}} \approx 0.60\ \text{m}
\]

If the actuator starts near the center of travel, the approximate one-sided travel available is:

\[
x_{\text{travel,one-side}} \approx 0.30\ \text{m}
\]

## Initial Linearized Model Coefficients

Using the simplified point-mass model with actuator acceleration input \(u = \ddot{x}\):

\[
A_{4,3} = \frac{g}{l}
\]

\[
B_4 = -\frac{1}{l}
\]

With \(l = 0.510\ \text{m}\):

\[
\frac{g}{l} = \frac{9.81}{0.510} \approx 19.235\ \text{s}^{-2}
\]

\[
-\frac{1}{l} = -\frac{1}{0.510} \approx -1.961\ \text{m}^{-1}
\]

These coefficients are for the first-pass MATLAB/LQR simulation. A later compound-pendulum model may replace \(l\) with measured mass, center-of-mass distance, and moment of inertia terms.
