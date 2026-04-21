# Pendulum Angle Reader Firmware

Arduino firmware for reading the pendulum angle from an AS5600 magnetic encoder.

Current bench hardware is an Arduino #3 connected to the AS5600 breakout over I2C.

## Requirements

- Arduino with I2C support
- AS5600 magnetic encoder breakout
- Rob Tillaart `AS5600` Arduino library

## Wiring

Current bench wiring from the AS5600 breakout to the Arduino:

- `VCC` to `5V`
- `GND` to `GND`
- `DIR` to `GND`
- `SCL` to `SCL`
- `SDA` to `SDA`

## Serial Output

The sketch streams CSV at `115200` baud:

```text
time_ms,connected,magnet_detected,magnet_too_weak,magnet_too_strong,agc,magnitude,raw_count,unwrapped_count,theta_deg,theta_rad,omega_rad_s
```

Ideal magnet-health flags are:

```text
connected = 1
magnet_detected = 1
magnet_too_weak = 0
magnet_too_strong = 0
```

During initial bring-up, the angle reading was smooth even though the magnetic field diagnostics reported `magnet_detected = 0` and `magnet_too_weak = 1`. For now, use the magnet fields as diagnostics rather than hard validity gates. Confirm that the angle is stable at rest, changes smoothly during rotation, wraps cleanly across `0/4095`, and returns to the same value at the same physical position.

Follow-up host captures on 2026-04-18 confirmed stable near-zero behavior, but larger manual angle checks were asymmetric. A roughly `45 deg` side-to-side check reported about `-47 deg` on one side and about `+89 deg` on the other. The likely cause is mechanical alignment: the magnet is currently sitting on the head of the pendulum axle bolt and may not be flat or centered relative to the AS5600. Before relying on the encoder for closed-loop control over a wider range, design a more stable magnet/sensor alignment method and repeat the held-angle validation.

## Serial Commands

- `z`: zero the current encoder position
- `r`: reset wrap tracking without changing the zero
- `h`: print the CSV header

## Notes

- The AS5600 raw angle is `0` to `4095` counts per revolution.
- The firmware unwraps rotations across the `0/4095` boundary.
- `theta_deg` and `theta_rad` are reported relative to the current zero.
- `omega_rad_s` is a simple finite-difference estimate from the unwrapped angle.
- `agc` and `magnitude` are included so magnet field strength can be checked during bring-up.
