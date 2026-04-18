from __future__ import annotations

from typing import Any

from .serial_worker import SerialWorker, parse_bool_int


class LimitSensorReader(SerialWorker):
    """Serial interface for Arduino #2 optical limit sensor reader."""

    HEADER = "time_ms,left_limit,right_limit,left_complement,right_complement,left_pair_ok,right_pair_ok,any_limit,servo_angle_deg"

    def request_header(self) -> None:
        self.write_command("h")

    def request_state(self) -> None:
        self.write_command("s")

    def release_servo(self) -> None:
        self.write_command("g")

    def hold_servo(self) -> None:
        self.write_command("p")

    def parse_line(self, line: str) -> dict[str, Any] | None:
        if line == self.HEADER:
            return None

        parts = line.split(",")

        if len(parts) == 4 and parts[0] == "# event":
            return {
                "event": parts[2],
                "event_active": parts[3] == "active",
                "time_ms": int(parts[1]),
            }

        if len(parts) not in {8, 9}:
            return None

        data = {
            "time_ms": int(parts[0]),
            "left_limit": parse_bool_int(parts[1]),
            "right_limit": parse_bool_int(parts[2]),
            "left_complement": parse_bool_int(parts[3]),
            "right_complement": parse_bool_int(parts[4]),
            "left_pair_ok": parse_bool_int(parts[5]),
            "right_pair_ok": parse_bool_int(parts[6]),
            "any_limit": parse_bool_int(parts[7]),
        }
        if len(parts) == 9:
            data["servo_angle_deg"] = int(parts[8])
        return data
