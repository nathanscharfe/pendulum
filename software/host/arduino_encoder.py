from __future__ import annotations

from typing import Any

from .serial_worker import SerialWorker, parse_bool_int


class EncoderReader(SerialWorker):
    """Serial interface for Arduino #3 AS5600 pendulum angle reader."""

    HEADER = (
        "time_ms,connected,magnet_detected,magnet_too_weak,magnet_too_strong,"
        "agc,magnitude,raw_count,filtered_count,unwrapped_count,theta_deg,theta_rad,omega_rad_s"
    )

    def request_header(self) -> None:
        self.write_command("h")

    def zero_current_position(self) -> None:
        self.write_command("z")

    def reset_tracking(self) -> None:
        self.write_command("r")

    def parse_line(self, line: str) -> dict[str, Any] | None:
        if line == self.HEADER:
            return None

        parts = line.split(",")
        if len(parts) not in (12, 13):
            return None

        if len(parts) == 12:
            return {
                "time_ms": int(parts[0]),
                "connected": parse_bool_int(parts[1]),
                "magnet_detected": parse_bool_int(parts[2]),
                "magnet_too_weak": parse_bool_int(parts[3]),
                "magnet_too_strong": parse_bool_int(parts[4]),
                "agc": int(parts[5]),
                "magnitude": int(parts[6]),
                "raw_count": int(parts[7]),
                "filtered_count": int(parts[7]),
                "unwrapped_count": int(parts[8]),
                "theta_deg": float(parts[9]),
                "theta_rad": float(parts[10]),
                "omega_rad_s": float(parts[11]),
            }

        return {
            "time_ms": int(parts[0]),
            "connected": parse_bool_int(parts[1]),
            "magnet_detected": parse_bool_int(parts[2]),
            "magnet_too_weak": parse_bool_int(parts[3]),
            "magnet_too_strong": parse_bool_int(parts[4]),
            "agc": int(parts[5]),
            "magnitude": int(parts[6]),
            "raw_count": int(parts[7]),
            "filtered_count": int(parts[8]),
            "unwrapped_count": int(parts[9]),
            "theta_deg": float(parts[10]),
            "theta_rad": float(parts[11]),
            "omega_rad_s": float(parts[12]),
        }
