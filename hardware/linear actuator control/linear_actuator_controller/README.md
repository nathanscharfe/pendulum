# Linear Actuator Controller Firmware

Arduino #1 firmware for commanding the linear actuator stepper driver.

## Wiring

- Driver `STEP` to Arduino `D10`
- Driver `DIR` to Arduino `D11`
- Driver `ENA` to Arduino `D12`
- Arduino `GND` to driver signal ground

The driver enable pin is treated as active-low:

```cpp
const byte ENABLE_ACTIVE_LEVEL = LOW;
const byte ENABLE_INACTIVE_LEVEL = HIGH;
```

## Serial Settings

- Baud: `115200`
- Commands end with a newline
- Commands can be separated with spaces or commas

## Commands

```text
h                         print help
?                         print status
e 1                       enable driver
e 0                       disable driver
x                         stop motion immediately
p <steps>                 set current position estimate
m <steps> <delay_us>      relative move by step count
r <steps_per_second>      continuous signed step-rate command
```

Examples:

```text
e 1
m 5000 100
m -5000 100
r 1000
r -1000
x
e 0
```

## Status Output

The firmware prints a status line every `500 ms`:

```text
status,time_ms,enabled,motion_mode,position_steps,remaining_move_steps,direction,step_delay_us
```

`motion_mode` values:

- `0`: idle
- `1`: relative move
- `2`: continuous motion

## Host-Side Unit Conversion

Arduino #1 accepts raw step commands and step-rate commands only. Physical-unit conversion, such as millimeters to steps or millimeters per second to steps per second, is handled by the host software.

## Safety Notes

- Arduino #1 does not read the limit sensors directly.
- The host software should monitor Arduino #2 limit sensor data and command Arduino #1 to stop before driving into an end stop.
- Use `x` for immediate software stop.
- Confirm direction signs with slow moves before running continuous commands.
