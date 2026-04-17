from __future__ import annotations

from typing import Any

from .serial_worker import SerialWorker, parse_bool_int


class ActuatorController(SerialWorker):
    """Serial interface for Arduino #1 linear actuator controller."""

    def request_status(self) -> None:
        self.write_command("?")

    def enable(self) -> None:
        self.write_command("e 1")

    def disable(self) -> None:
        self.write_command("e 0")

    def stop_motion(self) -> None:
        self.write_command("x")

    def set_position_steps(self, steps: int) -> None:
        self.write_command(f"p {steps}")

    def move_steps(self, steps: int, delay_us: int) -> None:
        self.write_command(f"m {steps} {delay_us}")

    def set_step_rate(self, steps_per_second: float) -> None:
        self.write_command(f"r {steps_per_second:.6g}")

    def parse_line(self, line: str) -> dict[str, Any] | None:
        if line == "status,time_ms,enabled,motion_mode,position_steps,remaining_move_steps,direction,step_delay_us":
            return None

        parts = line.split(",")
        if len(parts) != 8 or parts[0] != "status":
            return None

        return {
            "time_ms": int(parts[1]),
            "enabled": parse_bool_int(parts[2]),
            "motion_mode": int(parts[3]),
            "position_steps": int(parts[4]),
            "remaining_move_steps": int(parts[5]),
            "direction": int(parts[6]),
            "step_delay_us": int(parts[7]),
        }
