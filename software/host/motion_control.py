from __future__ import annotations

from dataclasses import dataclass
from time import sleep

from .arduino_actuator import ActuatorController
from .arduino_limits import LimitSensorReader
from .serial_worker import SerialSnapshot


@dataclass(frozen=True)
class MotionConfig:
    steps_per_mm: float = 5000.0 / 470.0
    travel_mm: float = 600.0
    default_speed_mm_s: float = 25.0
    command_period_s: float = 0.05

    @property
    def travel_steps(self) -> int:
        return round(self.travel_mm * self.steps_per_mm)


class MotionSafetyError(RuntimeError):
    pass


class MotionController:
    """Host-side actuator motion helper with limit-sensor safety checks."""

    def __init__(self, actuator: ActuatorController, limits: LimitSensorReader, config: MotionConfig | None = None) -> None:
        self.actuator = actuator
        self.limits = limits
        self.config = config or MotionConfig()

    def home(self, side: str = "left", speed_mm_s: float | None = None) -> None:
        side = self._normalize_side(side)
        signed_speed = abs(speed_mm_s or self.config.default_speed_mm_s)
        if side == "left":
            signed_speed = -signed_speed
        signed_step_rate = self.mm_s_to_steps_s(signed_speed)
        target_limit = "left_limit" if side == "left" else "right_limit"

        latest_limits = self._require_limits()
        if latest_limits.data.get(target_limit):
            self._stop_and_zero_for_home(side)
            return

        self._ensure_safe_direction(signed_step_rate, latest_limits)
        self.actuator.enable()
        self.actuator.set_step_rate(signed_step_rate)

        try:
            while True:
                latest_limits = self._require_limits()
                if latest_limits.data.get(target_limit):
                    self._stop_and_zero_for_home(side)
                    self.actuator.disable()
                    return

                self._stop_if_wrong_limit_for_direction(signed_step_rate, latest_limits)
                sleep(self.config.command_period_s)
        finally:
            self.actuator.stop_motion()

    def move_to_mm(self, target_mm: float, speed_mm_s: float | None = None) -> None:
        if target_mm < 0.0 or target_mm > self.config.travel_mm:
            raise ValueError(f"target_mm must be in [0, {self.config.travel_mm}]")

        status = self._require_actuator_status()
        current_steps = int(status.data["position_steps"])
        target_steps = self.mm_to_steps(target_mm)
        delta_steps = target_steps - current_steps

        speed = abs(speed_mm_s or self.config.default_speed_mm_s)
        step_rate = abs(self.mm_s_to_steps_s(speed))
        self.move_relative_steps(delta_steps, step_rate)

    def move_to_steps(self, target_steps: int, steps_per_second: float) -> None:
        status = self._require_actuator_status()
        current_steps = int(status.data["position_steps"])
        delta_steps = target_steps - current_steps
        self.move_relative_steps(delta_steps, abs(steps_per_second))

    def move_relative_steps(self, steps: int, steps_per_second: float) -> None:
        if steps == 0:
            return

        if steps_per_second <= 0.0:
            raise ValueError("steps_per_second must be positive")

        delay_us = self.delay_us_from_step_rate(steps_per_second)
        direction_rate = steps_per_second if steps > 0 else -steps_per_second
        latest_limits = self._require_limits()
        self._ensure_safe_direction(direction_rate, latest_limits)

        self.actuator.enable()
        self.actuator.move_steps(steps, delay_us)
        # The actuator firmware only streams status every 500 ms. Wait for a
        # fresh status line before allowing an old idle snapshot to end the move.
        sleep(0.75)

        try:
            while True:
                latest_limits = self._require_limits()
                self._stop_if_wrong_limit_for_direction(direction_rate, latest_limits)

                status = self._require_actuator_status()
                if int(status.data["motion_mode"]) == 0:
                    self.actuator.disable()
                    return

                sleep(self.config.command_period_s)
        finally:
            self.actuator.stop_motion()

    def run_speed(self, speed_mm_s: float, duration_s: float | None = None) -> None:
        latest_limits = self._require_limits()
        step_rate = self.mm_s_to_steps_s(speed_mm_s)
        self._ensure_safe_direction(step_rate, latest_limits)

        self.actuator.enable()
        self.actuator.set_step_rate(step_rate)

        start = monotonic()
        try:
            while True:
                latest_limits = self._require_limits()
                self._stop_if_wrong_limit_for_direction(step_rate, latest_limits)

                if duration_s is not None and (monotonic() - start) >= duration_s:
                    return

                sleep(self.config.command_period_s)
        finally:
            self.actuator.stop_motion()

    def stop(self) -> None:
        self.actuator.stop_motion()

    def mm_to_steps(self, value_mm: float) -> int:
        return round(value_mm * self.config.steps_per_mm)

    def steps_to_mm(self, value_steps: int) -> float:
        return value_steps / self.config.steps_per_mm

    def mm_s_to_steps_s(self, value_mm_s: float) -> float:
        return value_mm_s * self.config.steps_per_mm

    def delay_us_from_speed(self, speed_mm_s: float) -> int:
        return self.delay_us_from_step_rate(abs(self.mm_s_to_steps_s(speed_mm_s)))

    def delay_us_from_step_rate(self, steps_per_second: float) -> int:
        if steps_per_second <= 0.0:
            raise ValueError("steps_per_second must be positive")

        delay_us = 500000.0 / steps_per_second
        return max(10, min(10000, round(delay_us)))

    def _stop_and_zero_for_home(self, side: str) -> None:
        self.actuator.stop_motion()
        if side == "left":
            self.actuator.set_position_steps(0)
            self._wait_for_position_steps(0)

    def _wait_for_position_steps(self, expected_steps: int) -> None:
        while True:
            self.actuator.request_status()
            sleep(self.config.command_period_s)
            latest = self.actuator.get_latest()
            if latest is not None and int(latest.data["position_steps"]) == expected_steps:
                return

    def _ensure_safe_direction(self, speed_mm_s: float, latest_limits: SerialSnapshot) -> None:
        if speed_mm_s < 0 and latest_limits.data.get("left_limit"):
            self.actuator.stop_motion()
            raise MotionSafetyError("Refusing negative/left motion while left limit is active")

        if speed_mm_s > 0 and latest_limits.data.get("right_limit"):
            self.actuator.stop_motion()
            raise MotionSafetyError("Refusing positive/right motion while right limit is active")

    def _stop_if_wrong_limit_for_direction(self, speed_mm_s: float, latest_limits: SerialSnapshot) -> None:
        try:
            self._ensure_safe_direction(speed_mm_s, latest_limits)
        except MotionSafetyError:
            self.actuator.stop_motion()
            raise

    def _require_limits(self) -> SerialSnapshot:
        latest = self.limits.get_latest()
        if latest is None:
            self.limits.request_state()
            sleep(self.config.command_period_s)
            latest = self.limits.get_latest()

        if latest is None:
            raise MotionSafetyError("No limit sensor data available")

        return latest

    def _require_actuator_status(self) -> SerialSnapshot:
        latest = self.actuator.get_latest()
        if latest is None:
            self.actuator.request_status()
            sleep(self.config.command_period_s)
            latest = self.actuator.get_latest()

        if latest is None:
            raise MotionSafetyError("No actuator status available")

        return latest

    def _normalize_side(self, side: str) -> str:
        normalized = side.lower().strip()
        if normalized not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")
        return normalized
