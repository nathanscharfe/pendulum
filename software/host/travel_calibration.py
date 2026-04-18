from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from time import sleep

from .arduino_actuator import ActuatorController
from .arduino_limits import LimitSensorReader
from .motion_control import MotionController, MotionSafetyError


@dataclass(frozen=True)
class CalibrationPoint:
    speed_steps_s: float
    commanded_relative_steps: int
    start_steps: int
    actual_start_steps: int
    target_steps: int
    actual_position_steps: int
    start_tape_reading_mm: str
    tape_reading_mm: str
    completed: bool
    note: str


DEFAULT_SPEEDS_STEPS_S = (100.0, 200.0, 300.0)
DEFAULT_RELATIVE_STEPS = (6000, 5000, 4000, 3000, 2000, 1000)


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware") / "linear actuator calibration" / f"host_travel_calibration_{timestamp}.csv"


def run_travel_calibration(
    actuator: ActuatorController,
    limits: LimitSensorReader,
    motion: MotionController,
    output_path: Path | None = None,
    start_steps: int = 100,
    speeds_steps_s: tuple[float, ...] | None = None,
    relative_steps: tuple[int, ...] = DEFAULT_RELATIVE_STEPS,
    home_speed_mm_s: float = 10.0,
    reposition_speed_steps_s: float = 300.0,
) -> Path:
    output_path = output_path or default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    points: list[CalibrationPoint] = []

    print("Travel calibration routine")
    print(f"Output file: {output_path}")
    print("Keep the tape measure fixed. Enter tape readings in millimeters.")
    print("You will choose one speed at a time. After each speed, choose whether to continue.")
    print("Press Ctrl+C to stop; completed points will still be written.")
    input("Press Enter to home left and begin...")

    try:
        print("Homing left...")
        motion.home(side="left", speed_mm_s=home_speed_mm_s)

        queued_speeds = list(speeds_steps_s or ())
        while True:
            if queued_speeds:
                speed = queued_speeds.pop(0)
                print(f"\nUsing queued speed: {speed:g} steps/s")
            else:
                speed = prompt_positive_float("\nSpeed for this calibration run [steps/s]")

            print(f"\n=== Speed: {speed:g} steps/s ===")
            for move_steps in relative_steps:
                target_steps = start_steps + move_steps
                print(f"\nReturning to {start_steps} steps...")
                motion.move_to_steps(start_steps, reposition_speed_steps_s)
                sleep(0.25)

                actual_start_steps = current_position_steps(actuator)
                start_reading = prompt_tape_reading(
                    f"Confirm tape reading at {start_steps}-step start position before +{move_steps} steps [mm]"
                )

                print(f"Moving +{move_steps} steps to target {target_steps} at {speed:g} steps/s...")
                completed = True
                note = ""
                try:
                    motion.move_to_steps(target_steps, speed)
                except MotionSafetyError as exc:
                    completed = False
                    note = str(exc)
                    print(f"Move stopped by safety check: {note}")

                actual_steps = current_position_steps(actuator)
                tape_reading = prompt_tape_reading(
                    f"Tape reading after +{move_steps} steps at {speed:g} steps/s [mm]"
                )

                points.append(
                    CalibrationPoint(
                        speed_steps_s=speed,
                        commanded_relative_steps=move_steps,
                        start_steps=start_steps,
                        actual_start_steps=actual_start_steps,
                        target_steps=target_steps,
                        actual_position_steps=actual_steps,
                        start_tape_reading_mm=start_reading,
                        tape_reading_mm=tape_reading,
                        completed=completed,
                        note=note,
                    )
                )

            if not prompt_yes_no("\nRun another speed before saving? [y/N]", default=False):
                break

    except KeyboardInterrupt:
        print("\nCalibration interrupted. Writing completed points.")
        actuator.stop_motion()
    finally:
        write_points(output_path, points)
        actuator.disable()

    print(f"Wrote {len(points)} calibration rows to {output_path}")
    return output_path


def prompt_tape_reading(prompt: str) -> str:
    while True:
        value = input(f"{prompt}: ").strip()
        if value:
            return value
        print("Please enter a tape reading. Use a number if possible, or a short note if needed.")


def prompt_positive_float(prompt: str) -> float:
    while True:
        value = input(f"{prompt}: ").strip()
        try:
            number = float(value)
        except ValueError:
            print("Please enter a numeric speed.")
            continue

        if number > 0:
            return number

        print("Speed must be greater than zero.")


def prompt_yes_no(prompt: str, default: bool) -> bool:
    while True:
        value = input(f"{prompt}: ").strip().lower()
        if not value:
            return default
        if value in {"y", "yes"}:
            return True
        if value in {"n", "no"}:
            return False
        print("Please enter y or n.")


def current_position_steps(actuator: ActuatorController) -> int:
    actuator.request_status()
    sleep(0.6)
    latest = actuator.get_latest()
    if latest is None:
        raise RuntimeError("No actuator status available while recording calibration point")
    return int(latest.data["position_steps"])


def write_points(output_path: Path, points: list[CalibrationPoint]) -> None:
    fieldnames = [
        "speed_steps_s",
        "commanded_relative_steps",
        "start_steps",
        "actual_start_steps",
        "target_steps",
        "actual_position_steps",
        "start_tape_reading_mm",
        "tape_reading_mm",
        "completed",
        "note",
    ]

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for point in points:
            writer.writerow(
                {
                    "speed_steps_s": point.speed_steps_s,
                    "commanded_relative_steps": point.commanded_relative_steps,
                    "start_steps": point.start_steps,
                    "actual_start_steps": point.actual_start_steps,
                    "target_steps": point.target_steps,
                    "actual_position_steps": point.actual_position_steps,
                    "start_tape_reading_mm": point.start_tape_reading_mm,
                    "tape_reading_mm": point.tape_reading_mm,
                    "completed": point.completed,
                    "note": point.note,
                }
            )
