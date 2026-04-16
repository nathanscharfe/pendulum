# Linear Actuator Limit Sensor Reader

Arduino firmware for the second Arduino that monitors the optical limit sensors at each end of the linear actuator.

## Sensor Wiring

Wire both tested sensors this way:

Left/end A sensor:

- `brown` to Arduino `5V`
- `blue` to Arduino `GND`
- `black` to Arduino `D2`
- `white` disconnected

Right/end B sensor:

- `brown` to Arduino `5V`
- `blue` to Arduino `GND`
- `black` to Arduino `D3`
- `white` disconnected

With the default sketch settings, `left_limit` and `right_limit` should change between `0` and `1` when each sensor is unblocked and blocked.

## Sketch Pin Settings

- Left/end A black signal: `D2`
- Right/end B black signal: `D3`
- Left/end A white complement signal: disabled by default
- Right/end B white complement signal: disabled by default

If the white complement wires are used, enable them by assigning Arduino input pins:

```cpp
const byte LEFT_LIMIT_COMPLEMENT_PIN = 4;
const byte RIGHT_LIMIT_COMPLEMENT_PIN = 5;
```

## Input Polarity

The sketch defaults to:

```cpp
const byte LIMIT_INPUT_MODE = INPUT_PULLUP;
const byte LIMIT_ACTIVE_LEVEL = LOW;
```

That matches the tested sensors: the black signal wire uses the Arduino internal pullup, and a blocked beam reads active.

If the readings are backwards, change:

```cpp
const byte LIMIT_ACTIVE_LEVEL = HIGH;
```

## Serial Output

The sketch streams CSV at `115200` baud:

```text
time_ms,left_limit,right_limit,left_complement,right_complement,left_pair_ok,right_pair_ok,any_limit
```

It also prints event lines when a main limit state changes:

```text
# event,time_ms,left_limit,active
# event,time_ms,right_limit,inactive
```

## Serial Commands

- `h`: print the CSV header
- `s`: print the current state immediately

## Bring-Up Checklist

- Trigger each end by hand and confirm the matching CSV field changes.
- Confirm the complement signal is opposite the main signal if the white wire is used.
