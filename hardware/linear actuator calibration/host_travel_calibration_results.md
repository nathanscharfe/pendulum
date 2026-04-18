# Host Travel Calibration Results

Date: 2026-04-17

Raw data:

- `host_travel_calibration_20260417_194546.csv`

The host travel calibration returned to the 100-step reference position before each move. The tape reading at the 100-step reference was `599 mm` for every row. Positive actuator motion moved away from the left limit and decreased the tape reading, so the measured travel was calculated as:

```text
travel_mm = start_tape_reading_mm - tape_reading_mm
```

## Measurements

| Speed (steps/s) | Commanded steps | Start reading (mm) | End reading (mm) | Travel (mm) | Steps/mm |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 400 | 6000 | 599.0 | 35.5 | 563.5 | 10.6477 |
| 400 | 5000 | 599.0 | 129.5 | 469.5 | 10.6496 |
| 400 | 4000 | 599.0 | 223.5 | 375.5 | 10.6525 |
| 400 | 3000 | 599.0 | 317.5 | 281.5 | 10.6572 |
| 400 | 2000 | 599.0 | 411.5 | 187.5 | 10.6667 |
| 400 | 1000 | 599.0 | 505.0 | 94.0 | 10.6383 |

## Average Scale

Average measured scale:

```text
steps_per_mm = 10.651997690950955
mm_per_step = 0.09387916666666667
```

Use the rounded host calibration value:

```text
steps_per_mm = 10.652
position_mm = position_steps / 10.652
target_steps = round(target_mm * 10.652)
```

This is very close to the earlier provisional calibration of `5000 steps / 470 mm = 10.638 steps/mm`, but the new value is based on the repeated raw-step travel measurements collected by the host calibration routine.
