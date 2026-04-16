# Pendulum Angle Reader Firmware

Arduino firmware for reading the pendulum angle from an AS5600 magnetic encoder.

## Requirements

- Arduino with I2C support
- AS5600 magnetic encoder breakout
- Rob Tillaart `AS5600` Arduino library

## Wiring

Typical Arduino Uno or Nano wiring:

- `VCC` to `3.3V` or `5V`, depending on the breakout board
- `GND` to `GND`
- `SDA` to `A4`
- `SCL` to `A5`

For an Arduino Mega:

- `SDA` to `20`
- `SCL` to `21`

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
