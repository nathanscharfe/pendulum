from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from threading import Event, Lock, Thread
from time import monotonic, sleep
from tkinter import BOTH, DISABLED, LEFT, NORMAL, RIGHT, X, Y, Button, Canvas, DoubleVar, Frame, Label, Scale, Tk

from .arduino_actuator import ActuatorController
from .arduino_encoder import EncoderReader
from .arduino_limits import LimitSensorReader
from .motion_control import MotionConfig, MotionController
from .pendulum_control import (
    UprightControlConfig,
    apply_deadband,
    clamp,
    compute_lqr_acceleration,
    effective_acceleration_deadband,
    latest_limit_state,
    release_upright_servo_for_setup,
    resolve_lqr_config,
    update_alpha_beta_estimate,
    update_settle_sample_count,
    update_trigger_sample_count,
    wait_for_new_snapshot,
)


DEFAULT_UI_ACTUATOR_PORT = "COM6"
DEFAULT_UI_LIMITS_PORT = "COM10"
DEFAULT_UI_ENCODER_PORT = "COM8"


@dataclass(frozen=True)
class UprightControlUiConfig:
    poll_period_ms: int = 20
    history_window_s: float = 12.0
    canvas_width: int = 860
    canvas_height: int = 420
    padding_px: int = 42
    setup_settle_s: float = 2.0
    left_axis_min_deg: float = -15.0
    left_axis_max_deg: float = 15.0
    plot_speed_margin_mm_s: float = 10.0


@dataclass(frozen=True)
class LqrWeights:
    q_x: float
    q_x_dot: float
    q_theta: float
    q_theta_dot: float
    r_input: float


@dataclass(frozen=True)
class DeadbandSettings:
    theta_deadband_rad: float
    theta_dot_deadband_rad_s: float
    acceleration_deadband_m_s2: float
    control_trigger_theta_rad: float
    control_trigger_theta_dot_rad_s: float
    control_trigger_samples: int
    settle_theta_rad: float
    settle_theta_dot_rad_s: float
    settle_samples: int


@dataclass(frozen=True)
class ControllerTelemetry:
    timestamp_s: float
    filtered_theta_deg: float
    filtered_theta_dot_deg_s: float
    command_speed_mm_s: float
    control_armed: bool


class UprightControlRuntime:
    def __init__(
        self,
        actuator_port: str,
        limits_port: str,
        encoder_port: str,
        *,
        baudrate: int,
        motion_config: MotionConfig,
        control_config: UprightControlConfig,
    ) -> None:
        self.motion_config = motion_config
        self.base_control_config = control_config
        self.actuator = ActuatorController(actuator_port, baudrate)
        self.limits = LimitSensorReader(limits_port, baudrate)
        self.encoder = EncoderReader(encoder_port, baudrate)
        self.motion = MotionController(self.actuator, self.limits, motion_config)

        self._state_lock = Lock()
        self._weights_lock = Lock()
        self._telemetry_lock = Lock()
        self._stop_control_event = Event()
        self._shutdown_event = Event()
        self._setup_thread: Thread | None = None
        self._control_thread: Thread | None = None

        self._weights = LqrWeights(
            q_x=control_config.lqr_q_x,
            q_x_dot=control_config.lqr_q_x_dot,
            q_theta=control_config.lqr_q_theta,
            q_theta_dot=control_config.lqr_q_theta_dot,
            r_input=control_config.lqr_r_input,
        )
        self._deadbands = DeadbandSettings(
            theta_deadband_rad=control_config.theta_deadband_rad,
            theta_dot_deadband_rad_s=control_config.theta_dot_deadband_rad_s,
            acceleration_deadband_m_s2=control_config.acceleration_deadband_m_s2,
            control_trigger_theta_rad=control_config.control_trigger_theta_rad,
            control_trigger_theta_dot_rad_s=control_config.control_trigger_theta_dot_rad_s,
            control_trigger_samples=control_config.control_trigger_samples,
            settle_theta_rad=control_config.settle_theta_rad,
            settle_theta_dot_rad_s=control_config.settle_theta_dot_rad_s,
            settle_samples=control_config.settle_samples,
        )
        self._current_gain_text = self._format_gain_text(self._resolve_active_config())
        self._telemetry_history: deque[ControllerTelemetry] = deque(maxlen=5000)
        self._latest_command_speed_mm_s = 0.0
        self._latest_filtered_theta_deg = 0.0
        self._status_message = "Starting serial setup..."
        self._setup_complete = False
        self._control_running = False
        self._last_stop_reason = ""

    def start(self) -> None:
        self._setup_thread = Thread(target=self._run_setup, name="upright-ui-setup", daemon=True)
        self._setup_thread.start()

    def shutdown(self) -> None:
        self._shutdown_event.set()
        self.stop_control()
        if self._setup_thread is not None and self._setup_thread.is_alive():
            self._setup_thread.join(timeout=2.0)
        for worker in (self.encoder, self.limits, self.actuator):
            worker.stop()

    def start_control(self) -> None:
        with self._state_lock:
            if not self._setup_complete or self._control_running:
                return
            self._control_running = True
            self._last_stop_reason = ""
            self._status_message = "Zeroing encoder and starting automatic control..."
        self._stop_control_event.clear()
        self._control_thread = Thread(target=self._run_control_loop, name="upright-ui-control", daemon=True)
        self._control_thread.start()

    def stop_control(self) -> None:
        self._stop_control_event.set()
        if self._control_thread is not None and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        try:
            self.actuator.stop_motion()
            self.actuator.disable()
        except RuntimeError:
            pass
        with self._state_lock:
            self._control_running = False

    def update_weights(self, *, q_x: float, q_x_dot: float, q_theta: float, q_theta_dot: float, r_input: float) -> None:
        weights = LqrWeights(
            q_x=max(q_x, 1e-6),
            q_x_dot=max(q_x_dot, 1e-6),
            q_theta=max(q_theta, 1e-6),
            q_theta_dot=max(q_theta_dot, 1e-6),
            r_input=max(r_input, 1e-6),
        )
        with self._weights_lock:
            self._weights = weights
            self._current_gain_text = self._format_gain_text(
                self._resolve_active_config_from_values(weights, self._deadbands)
            )

    def update_deadbands(
        self,
        *,
        theta_deadband_rad: float,
        theta_dot_deadband_rad_s: float,
        acceleration_deadband_m_s2: float,
        control_trigger_theta_rad: float,
        control_trigger_theta_dot_rad_s: float,
        control_trigger_samples: int,
        settle_theta_rad: float,
        settle_theta_dot_rad_s: float,
        settle_samples: int,
    ) -> None:
        settings = DeadbandSettings(
            theta_deadband_rad=max(theta_deadband_rad, 0.0),
            theta_dot_deadband_rad_s=max(theta_dot_deadband_rad_s, 0.0),
            acceleration_deadband_m_s2=max(acceleration_deadband_m_s2, 0.0),
            control_trigger_theta_rad=max(control_trigger_theta_rad, 0.0),
            control_trigger_theta_dot_rad_s=max(control_trigger_theta_dot_rad_s, 0.0),
            control_trigger_samples=max(control_trigger_samples, 0),
            settle_theta_rad=max(settle_theta_rad, 0.0),
            settle_theta_dot_rad_s=max(settle_theta_dot_rad_s, 0.0),
            settle_samples=max(settle_samples, 1),
        )
        with self._weights_lock:
            self._deadbands = settings
            self._current_gain_text = self._format_gain_text(self._resolve_active_config_from_values(self._weights, settings))

    def get_setup_complete(self) -> bool:
        with self._state_lock:
            return self._setup_complete

    def get_control_running(self) -> bool:
        with self._state_lock:
            return self._control_running

    def get_status_message(self) -> str:
        with self._state_lock:
            return self._status_message

    def get_gain_text(self) -> str:
        with self._weights_lock:
            return self._current_gain_text

    def get_weight_values(self) -> LqrWeights:
        with self._weights_lock:
            return self._weights

    def get_deadband_values(self) -> DeadbandSettings:
        with self._weights_lock:
            return self._deadbands

    def get_latest_command_speed_mm_s(self) -> float:
        with self._telemetry_lock:
            return self._latest_command_speed_mm_s

    def get_plot_history(self, window_s: float) -> list[ControllerTelemetry]:
        latest_encoder = self.encoder.get_latest()
        now_s = monotonic()
        history: list[ControllerTelemetry]
        with self._telemetry_lock:
            history = list(self._telemetry_history)
            latest_speed_mm_s = self._latest_command_speed_mm_s

        if latest_encoder is not None:
            current_sample = ControllerTelemetry(
                timestamp_s=latest_encoder.timestamp_s,
                filtered_theta_deg=float(latest_encoder.data["theta_deg"]),
                filtered_theta_dot_deg_s=float(latest_encoder.data.get("omega_rad_s", 0.0)) * (180.0 / 3.141592653589793),
                command_speed_mm_s=latest_speed_mm_s,
                control_armed=self.get_control_running(),
            )
            if not history or current_sample.timestamp_s > history[-1].timestamp_s:
                history.append(current_sample)

        if not history:
            return []

        newest_time_s = history[-1].timestamp_s
        oldest_allowed_s = newest_time_s - window_s
        trimmed = [sample for sample in history if sample.timestamp_s >= oldest_allowed_s]
        if trimmed:
            return trimmed

        # Fallback for any odd clock edge-case.
        return [sample for sample in history if newest_time_s - sample.timestamp_s <= window_s + max(0.0, now_s - newest_time_s)]

    def _run_setup(self) -> None:
        try:
            self._set_status("Opening Arduino serial ports...")
            self.actuator.start()
            self.limits.start()
            self.encoder.start()
            sleep(2.0)

            self.actuator.request_status()
            self.limits.request_state()
            release_upright_servo_for_setup(self.limits)

            self._set_status("Homing actuator left...")
            self.motion.home(side="left", speed_mm_s=self.base_control_config.home_speed_mm_s)

            self._set_status("Moving cart to midpoint...")
            self.motion.move_to_middle(speed_mm_s=self.base_control_config.middle_speed_mm_s)

            self._set_status("Holding latch and waiting for start.")
            self.limits.hold_servo()
            sleep(0.2)

            with self._state_lock:
                self._setup_complete = True
                self._status_message = "Setup complete. Place the pendulum upright and press Start Automatic Control."
        except Exception as exc:
            with self._state_lock:
                self._setup_complete = False
                self._status_message = f"Setup failed: {exc}"

    def _run_control_loop(self) -> None:
        stop_reason = "stopped"
        try:
            if self.base_control_config.zero_encoder_on_start:
                self.encoder.zero_current_position()
                sleep(0.1)

            self.encoder.clear_snapshots()
            self.limits.clear_snapshots()
            self.actuator.clear_snapshots()

            arm_reference_snapshot = self.encoder.get_latest()
            arm_reference_timestamp_s = arm_reference_snapshot.timestamp_s if arm_reference_snapshot is not None else 0.0

            self.limits.release_servo()
            sleep(0.05)

            self.actuator.enable()
            fresh_encoder_snapshot = wait_for_new_snapshot(
                self.encoder,
                newer_than_timestamp_s=arm_reference_timestamp_s,
                timeout_s=max(1.0, 5.0 * self.base_control_config.period_s),
            )
            if fresh_encoder_snapshot is None:
                stop_reason = "no fresh encoder sample after start"
                return

            next_loop_s = monotonic()
            last_command_speed_mm_s: float | None = None
            command_speed_mm_s = 0.0
            filtered_theta_rad: float | None = None
            previous_filtered_theta_rad: float | None = None
            filtered_theta_dot_rad_s: float | None = None
            theta_residual_rad = 0.0
            control_armed = False
            trigger_sample_count = 0
            settle_sample_count = 0
            last_encoder_timestamp_s = fresh_encoder_snapshot.timestamp_s
            active_signature: tuple[float, float, float, float, float, float, float, float, float, float, float, float, int, int] | None = None
            active_config = self._resolve_active_config()

            self._set_status("Automatic control running.")

            while not self._shutdown_event.is_set() and not self._stop_control_event.is_set():
                now_s = monotonic()
                if now_s < next_loop_s:
                    sleep(min(0.002, next_loop_s - now_s))
                    continue
                next_loop_s += active_config.period_s

                current_signature = self._weight_signature()
                if current_signature != active_signature:
                    active_config = self._resolve_active_config()
                    active_signature = current_signature
                    self._set_status("Automatic control running. LQR gains updated from UI.")

                encoder_snapshot = self.encoder.get_latest()
                actuator_snapshot = self.actuator.get_latest()
                limits_snapshot = latest_limit_state(self.limits)

                if encoder_snapshot is None:
                    stop_reason = "no_encoder_data"
                    break

                if encoder_snapshot.timestamp_s <= last_encoder_timestamp_s:
                    sleep(0.001)
                    continue
                last_encoder_timestamp_s = encoder_snapshot.timestamp_s

                if actuator_snapshot is None:
                    self.actuator.request_status()

                if limits_snapshot is None:
                    self.limits.request_state()
                    stop_reason = "no_limit_data"
                    break

                left_limit = bool(limits_snapshot.data.get("left_limit", False))
                right_limit = bool(limits_snapshot.data.get("right_limit", False))
                any_limit = bool(limits_snapshot.data.get("any_limit", left_limit or right_limit))
                if any_limit:
                    stop_reason = "limit_active"
                    break

                raw_theta_rad = float(encoder_snapshot.data["theta_rad"])
                if abs(raw_theta_rad) > active_config.max_abs_theta_rad:
                    stop_reason = "angle_cutoff"
                    break

                dt_s = active_config.period_s
                theta_rad = active_config.encoder_angle_sign * raw_theta_rad
                if active_config.estimator == "alpha-beta":
                    theta_alpha = active_config.armed_theta_alpha if control_armed else active_config.theta_alpha
                    theta_beta = active_config.armed_theta_beta if control_armed else active_config.theta_beta
                    max_theta_residual_rad = (
                        active_config.armed_max_theta_residual_rad
                        if control_armed
                        else active_config.max_theta_residual_rad
                    )
                    filtered_theta_rad, filtered_theta_dot_rad_s, theta_residual_rad = update_alpha_beta_estimate(
                        theta_estimate_rad=filtered_theta_rad,
                        theta_dot_estimate_rad_s=filtered_theta_dot_rad_s,
                        measured_theta_rad=theta_rad,
                        dt_s=dt_s,
                        alpha=theta_alpha,
                        beta=theta_beta,
                        max_residual_rad=max_theta_residual_rad,
                    )
                else:
                    limited_theta_rad = theta_rad
                    previous_filtered_theta_rad = filtered_theta_rad
                    filtered_theta_rad = theta_rad if filtered_theta_rad is None else theta_rad
                    theta_dot_rad_s = (
                        0.0 if previous_filtered_theta_rad is None else (filtered_theta_rad - previous_filtered_theta_rad) / dt_s
                    )
                    filtered_theta_dot_rad_s = theta_dot_rad_s
                    theta_residual_rad = theta_rad - limited_theta_rad

                control_theta_rad = apply_deadband(filtered_theta_rad, active_config.theta_deadband_rad)
                control_theta_dot_rad_s = apply_deadband(
                    filtered_theta_dot_rad_s,
                    active_config.theta_dot_deadband_rad_s,
                )

                if not control_armed:
                    trigger_sample_count = update_trigger_sample_count(
                        trigger_sample_count=trigger_sample_count,
                        theta_rad=filtered_theta_rad,
                        theta_dot_rad_s=filtered_theta_dot_rad_s,
                        config=active_config,
                    )
                    control_armed = trigger_sample_count >= active_config.control_trigger_samples
                    if control_armed:
                        settle_sample_count = 0
                else:
                    settle_sample_count = update_settle_sample_count(
                        settle_sample_count=settle_sample_count,
                        raw_theta_rad=theta_rad,
                        theta_rad=filtered_theta_rad,
                        theta_dot_rad_s=filtered_theta_dot_rad_s,
                        theta_residual_rad=theta_residual_rad,
                        command_speed_mm_s=command_speed_mm_s,
                        config=active_config,
                    )
                    if settle_sample_count >= active_config.settle_samples:
                        control_armed = False
                        trigger_sample_count = 0
                        settle_sample_count = 0
                        command_speed_mm_s = 0.0

                position_steps = int(actuator_snapshot.data["position_steps"]) if actuator_snapshot is not None else 0
                position_mm = position_steps / self.motion_config.steps_per_mm
                position_m = position_mm / 1000.0
                center_m = (self.motion_config.travel_mm / 1000.0) / 2.0
                control_x_m = active_config.cart_position_sign * (position_m - center_m)
                control_x_dot_m_s = active_config.cart_position_sign * command_speed_mm_s / 1000.0
                center_error_mm = position_mm - (self.motion_config.travel_mm / 2.0)

                if control_armed:
                    command_acceleration_m_s2 = compute_lqr_acceleration(
                        x_m=control_x_m,
                        x_dot_m_s=control_x_dot_m_s,
                        theta_rad=control_theta_rad,
                        theta_dot_rad_s=control_theta_dot_rad_s,
                        config=active_config,
                    )
                else:
                    command_acceleration_m_s2 = 0.0
                if abs(command_acceleration_m_s2) < effective_acceleration_deadband(active_config):
                    command_acceleration_m_s2 = 0.0

                leak = max(0.0, 1.0 - active_config.velocity_leak_per_s * dt_s)
                if control_armed:
                    command_speed_mm_s = command_speed_mm_s * leak + command_acceleration_m_s2 * dt_s * 1000.0
                    command_speed_mm_s -= active_config.center_gain_per_s * center_error_mm
                else:
                    command_speed_mm_s = 0.0
                command_speed_mm_s = clamp(command_speed_mm_s, -active_config.max_speed_mm_s, active_config.max_speed_mm_s)

                if (
                    command_acceleration_m_s2 == 0.0
                    and control_x_m == 0.0
                    and control_x_dot_m_s == 0.0
                    and control_theta_rad == 0.0
                    and control_theta_dot_rad_s == 0.0
                ):
                    command_speed_mm_s = 0.0

                if command_speed_mm_s < 0.0 and left_limit:
                    stop_reason = "left_limit_command"
                    break
                if command_speed_mm_s > 0.0 and right_limit:
                    stop_reason = "right_limit_command"
                    break

                if last_command_speed_mm_s is None or abs(command_speed_mm_s - last_command_speed_mm_s) >= 0.05:
                    self.actuator.set_step_rate(command_speed_mm_s * self.motion_config.steps_per_mm)
                    last_command_speed_mm_s = command_speed_mm_s

                self._append_telemetry(
                    timestamp_s=encoder_snapshot.timestamp_s,
                    filtered_theta_deg=filtered_theta_rad * (180.0 / 3.141592653589793),
                    filtered_theta_dot_deg_s=filtered_theta_dot_rad_s * (180.0 / 3.141592653589793),
                    command_speed_mm_s=command_speed_mm_s,
                    control_armed=control_armed,
                )
        except Exception as exc:
            stop_reason = f"error: {exc}"
        finally:
            try:
                self.actuator.stop_motion()
                self.actuator.disable()
            except RuntimeError:
                pass
            with self._state_lock:
                self._control_running = False
                self._last_stop_reason = stop_reason
                if self._setup_complete and not self._shutdown_event.is_set():
                    self._status_message = (
                        "Automatic control stopped. "
                        f"Reason: {stop_reason}. Press Start Automatic Control to run again."
                    )
            with self._telemetry_lock:
                self._latest_command_speed_mm_s = 0.0

    def _append_telemetry(
        self,
        *,
        timestamp_s: float,
        filtered_theta_deg: float,
        filtered_theta_dot_deg_s: float,
        command_speed_mm_s: float,
        control_armed: bool,
    ) -> None:
        with self._telemetry_lock:
            self._latest_command_speed_mm_s = command_speed_mm_s
            self._latest_filtered_theta_deg = filtered_theta_deg
            self._telemetry_history.append(
                ControllerTelemetry(
                    timestamp_s=timestamp_s,
                    filtered_theta_deg=filtered_theta_deg,
                    filtered_theta_dot_deg_s=filtered_theta_dot_deg_s,
                    command_speed_mm_s=command_speed_mm_s,
                    control_armed=control_armed,
                )
            )

    def _set_status(self, message: str) -> None:
        with self._state_lock:
            self._status_message = message

    def _weight_signature(self) -> tuple[float, float, float, float, float, float, float, float, float, float, float, float, int, int]:
        weights = self.get_weight_values()
        deadbands = self.get_deadband_values()
        return (
            weights.q_x,
            weights.q_x_dot,
            weights.q_theta,
            weights.q_theta_dot,
            weights.r_input,
            deadbands.theta_deadband_rad,
            deadbands.theta_dot_deadband_rad_s,
            deadbands.acceleration_deadband_m_s2,
            deadbands.control_trigger_theta_rad,
            deadbands.control_trigger_theta_dot_rad_s,
            deadbands.settle_theta_rad,
            deadbands.settle_theta_dot_rad_s,
            deadbands.control_trigger_samples,
            deadbands.settle_samples,
        )

    def _resolve_active_config(self) -> UprightControlConfig:
        weights = self.get_weight_values()
        deadbands = self.get_deadband_values()
        return self._resolve_active_config_from_values(weights, deadbands)

    def _resolve_active_config_from_values(
        self,
        weights: LqrWeights,
        deadbands: DeadbandSettings,
    ) -> UprightControlConfig:
        updated_config = replace(
            self.base_control_config,
            lqr_q_x=weights.q_x,
            lqr_q_x_dot=weights.q_x_dot,
            lqr_q_theta=weights.q_theta,
            lqr_q_theta_dot=weights.q_theta_dot,
            lqr_r_input=weights.r_input,
            theta_deadband_rad=deadbands.theta_deadband_rad,
            theta_dot_deadband_rad_s=deadbands.theta_dot_deadband_rad_s,
            acceleration_deadband_m_s2=deadbands.acceleration_deadband_m_s2,
            control_trigger_theta_rad=deadbands.control_trigger_theta_rad,
            control_trigger_theta_dot_rad_s=deadbands.control_trigger_theta_dot_rad_s,
            control_trigger_samples=deadbands.control_trigger_samples,
            settle_theta_rad=deadbands.settle_theta_rad,
            settle_theta_dot_rad_s=deadbands.settle_theta_dot_rad_s,
            settle_samples=deadbands.settle_samples,
        )
        return resolve_lqr_config(updated_config, equilibrium="upright")

    def _format_gain_text(self, config: UprightControlConfig) -> str:
        return (
            "K = ["
            f"{config.lqr_x_gain:+.3f}, "
            f"{config.lqr_x_dot_gain:+.3f}, "
            f"{config.lqr_theta_gain:+.3f}, "
            f"{config.lqr_theta_dot_gain:+.3f}]"
        )


class UprightControlUiApp:
    def __init__(self, runtime: UprightControlRuntime, config: UprightControlUiConfig) -> None:
        self.runtime = runtime
        self.config = config

        self.root = Tk()
        self.root.title("Inverted Pendulum Automatic Control")
        self.root.geometry("1280x820")
        self.root.minsize(980, 680)
        self.root.protocol("WM_DELETE_WINDOW", self._handle_close)

        main = Frame(self.root)
        main.pack(fill=BOTH, expand=True, padx=12, pady=12)

        left_panel = Frame(main)
        left_panel.pack(side=LEFT, fill=BOTH, expand=True)

        right_panel = Frame(main)
        right_panel.pack(side=RIGHT, fill=Y, padx=(12, 0))

        controls = Frame(left_panel)
        controls.pack(fill=X, pady=(0, 8))

        self.start_button = Button(
            controls,
            text="Start Automatic Control",
            width=24,
            state=DISABLED,
            command=self.runtime.start_control,
        )
        self.start_button.pack(side=LEFT)
        Button(controls, text="Zero Encoder", width=14, command=self._zero_encoder).pack(side=LEFT, padx=(8, 0))
        Button(controls, text="Stop And Exit", width=16, command=self._handle_close).pack(side=RIGHT)

        self.status_label = Label(left_panel, anchor="w", justify="left", font=("Segoe UI", 10))
        self.status_label.pack(fill=X, pady=(0, 6))

        self.gain_label = Label(left_panel, anchor="w", justify="left", font=("Consolas", 10))
        self.gain_label.pack(fill=X, pady=(0, 10))

        self.canvas = Canvas(
            left_panel,
            width=self.config.canvas_width,
            height=self.config.canvas_height,
            background="white",
            highlightthickness=1,
            highlightbackground="#c7c7c7",
        )
        self.canvas.pack(fill=BOTH, expand=True)

        self.theta_label = Label(left_panel, anchor="w", font=("Segoe UI", 11, "bold"))
        self.theta_label.pack(fill=X, pady=(8, 0))
        self.speed_label = Label(left_panel, anchor="w", font=("Segoe UI", 11))
        self.speed_label.pack(fill=X, pady=(4, 0))

        self.q_x_var = DoubleVar(value=self.runtime.get_weight_values().q_x)
        self.q_x_dot_var = DoubleVar(value=self.runtime.get_weight_values().q_x_dot)
        self.q_theta_var = DoubleVar(value=self.runtime.get_weight_values().q_theta)
        self.q_theta_dot_var = DoubleVar(value=self.runtime.get_weight_values().q_theta_dot)
        self.r_input_var = DoubleVar(value=self.runtime.get_weight_values().r_input)
        deadbands = self.runtime.get_deadband_values()
        self.theta_deadband_var = DoubleVar(value=deadbands.theta_deadband_rad)
        self.theta_dot_deadband_var = DoubleVar(value=deadbands.theta_dot_deadband_rad_s * (180.0 / 3.141592653589793))
        self.accel_deadband_var = DoubleVar(value=deadbands.acceleration_deadband_m_s2)
        self.trigger_theta_var = DoubleVar(value=deadbands.control_trigger_theta_rad)
        self.trigger_theta_dot_var = DoubleVar(value=deadbands.control_trigger_theta_dot_rad_s * (180.0 / 3.141592653589793))
        self.trigger_samples_var = DoubleVar(value=float(deadbands.control_trigger_samples))
        self.settle_theta_var = DoubleVar(value=deadbands.settle_theta_rad * (180.0 / 3.141592653589793))
        self.settle_theta_dot_var = DoubleVar(value=deadbands.settle_theta_dot_rad_s * (180.0 / 3.141592653589793))
        self.settle_samples_var = DoubleVar(value=float(deadbands.settle_samples))

        self._build_slider(right_panel, "Qx", self.q_x_var, 0.1, 20.0, 0.1)
        self._build_slider(right_panel, "Qx_dot", self.q_x_dot_var, 0.01, 5.0, 0.01)
        self._build_slider(right_panel, "Qtheta", self.q_theta_var, 1.0, 200.0, 1.0)
        self._build_slider(right_panel, "Qomega", self.q_theta_dot_var, 0.1, 20.0, 0.1)
        self._build_slider(right_panel, "R", self.r_input_var, 0.1, 20.0, 0.1)
        self._build_slider(right_panel, "theta deadband", self.theta_deadband_var, 0.0, 0.03, 0.0005)
        self._build_slider(right_panel, "omega deadband (deg/s)", self.theta_dot_deadband_var, 0.0, 20.0, 0.5)
        self._build_slider(right_panel, "accel deadband", self.accel_deadband_var, 0.0, 0.05, 0.001)
        self._build_slider(right_panel, "trigger theta", self.trigger_theta_var, 0.0, 0.05, 0.001)
        self._build_slider(right_panel, "trigger omega (deg/s)", self.trigger_theta_dot_var, 0.0, 20.0, 0.5)
        self._build_slider(right_panel, "trigger samples", self.trigger_samples_var, 0.0, 10.0, 1.0)
        self._build_slider(right_panel, "settle theta (deg)", self.settle_theta_var, 0.0, 15.0, 0.5)
        self._build_slider(right_panel, "settle omega (deg/s)", self.settle_theta_dot_var, 0.0, 60.0, 1.0)
        self._build_slider(right_panel, "settle samples", self.settle_samples_var, 1.0, 60.0, 1.0)

    def run(self) -> None:
        self.runtime.start()
        self._schedule_update()
        self.root.mainloop()

    def _schedule_update(self) -> None:
        self._update_ui()
        self.root.after(self.config.poll_period_ms, self._schedule_update)

    def _update_ui(self) -> None:
        setup_complete = self.runtime.get_setup_complete()
        control_running = self.runtime.get_control_running()
        status_message = self.runtime.get_status_message()
        self.status_label.configure(text=status_message)
        self.gain_label.configure(text=self.runtime.get_gain_text())
        deadbands = self.runtime.get_deadband_values()

        if setup_complete and not control_running:
            self.start_button.configure(state=NORMAL)
        else:
            self.start_button.configure(state=DISABLED)

        history = self.runtime.get_plot_history(self.config.history_window_s)
        self._draw_plot(
            history,
            settle_theta_deg=deadbands.settle_theta_rad * (180.0 / 3.141592653589793),
            settle_omega_deg_s=deadbands.settle_theta_dot_rad_s * (180.0 / 3.141592653589793),
        )

        if history:
            latest = history[-1]
            self.theta_label.configure(text=f"Filtered angle: {latest.filtered_theta_deg:+.3f} deg")
            self.speed_label.configure(text=f"Commanded speed: {latest.command_speed_mm_s:+.2f} mm/s")
        else:
            self.theta_label.configure(text="Filtered angle: waiting for encoder data")
            self.speed_label.configure(text="Commanded speed: waiting for control data")

    def _draw_plot(
        self,
        history: list[ControllerTelemetry],
        *,
        settle_theta_deg: float,
        settle_omega_deg_s: float,
    ) -> None:
        width = max(self.canvas.winfo_width(), self.config.canvas_width)
        height = max(self.canvas.winfo_height(), self.config.canvas_height)
        left = self.config.padding_px
        top = self.config.padding_px
        right = width - self.config.padding_px
        bottom = height - self.config.padding_px

        self.canvas.delete("all")
        self.canvas.create_rectangle(left, top, right, bottom, outline="#d0d0d0")

        if not history:
            self.canvas.create_text(
                width / 2,
                height / 2,
                text="Waiting for setup and live data...",
                fill="#666666",
                font=("Segoe UI", 12),
            )
            return

        times_s = [sample.timestamp_s for sample in history]
        newest_time_s = times_s[-1]
        oldest_time_s = times_s[0]
        time_span_s = max(newest_time_s - oldest_time_s, 1e-6)

        theta_min = self.config.left_axis_min_deg
        theta_max = self.config.left_axis_max_deg
        theta_span = max(theta_max - theta_min, 1e-6)

        speed_values = [sample.command_speed_mm_s for sample in history]
        speed_min = min(speed_values)
        speed_max = max(speed_values)
        if speed_min == speed_max:
            speed_min -= 5.0
            speed_max += 5.0
        speed_min -= self.config.plot_speed_margin_mm_s
        speed_max += self.config.plot_speed_margin_mm_s
        speed_span = max(speed_max - speed_min, 1e-6)

        if settle_theta_deg > 0.0:
            band_min = -settle_theta_deg
            band_max = settle_theta_deg
            visible_band_min = max(theta_min, band_min)
            visible_band_max = min(theta_max, band_max)
            if visible_band_min <= visible_band_max:
                band_top_y = bottom - ((visible_band_max - theta_min) / theta_span) * (bottom - top)
                band_bottom_y = bottom - ((visible_band_min - theta_min) / theta_span) * (bottom - top)
                self.canvas.create_rectangle(
                    left,
                    band_top_y,
                    right,
                    band_bottom_y,
                    fill="#0b6efd",
                    stipple="gray12",
                    outline="",
                )
                self.canvas.create_line(left, band_top_y, right, band_top_y, fill="#0b6efd", dash=(5, 3))
                self.canvas.create_line(left, band_bottom_y, right, band_bottom_y, fill="#0b6efd", dash=(5, 3))

        theta_zero_y = None
        if theta_min <= 0.0 <= theta_max:
            theta_zero_y = bottom - ((0.0 - theta_min) / theta_span) * (bottom - top)
            self.canvas.create_line(left, theta_zero_y, right, theta_zero_y, fill="#d9e7ff", dash=(4, 3))

        speed_zero_y = None
        if speed_min <= 0.0 <= speed_max:
            speed_zero_y = bottom - ((0.0 - speed_min) / speed_span) * (bottom - top)
            self.canvas.create_line(left, speed_zero_y, right, speed_zero_y, fill="#ffe1d6", dash=(2, 4))

        if settle_omega_deg_s > 0.0:
            omega_band_min = -settle_omega_deg_s
            omega_band_max = settle_omega_deg_s
            visible_omega_band_min = max(theta_min, omega_band_min)
            visible_omega_band_max = min(theta_max, omega_band_max)
            if visible_omega_band_min <= visible_omega_band_max:
                omega_band_top_y = bottom - ((visible_omega_band_max - theta_min) / theta_span) * (bottom - top)
                omega_band_bottom_y = bottom - ((visible_omega_band_min - theta_min) / theta_span) * (bottom - top)
                self.canvas.create_rectangle(
                    left,
                    omega_band_top_y,
                    right,
                    omega_band_bottom_y,
                    fill="#2da44e",
                    stipple="gray25",
                    outline="",
                )
                self.canvas.create_line(left, omega_band_top_y, right, omega_band_top_y, fill="#2da44e", dash=(3, 3))
                self.canvas.create_line(left, omega_band_bottom_y, right, omega_band_bottom_y, fill="#2da44e", dash=(3, 3))

        theta_points: list[float] = []
        speed_points: list[float] = []
        omega_points: list[float] = []
        for sample in history:
            x = left + ((sample.timestamp_s - oldest_time_s) / time_span_s) * (right - left)
            theta_y = bottom - ((sample.filtered_theta_deg - theta_min) / theta_span) * (bottom - top)
            speed_y = bottom - ((sample.command_speed_mm_s - speed_min) / speed_span) * (bottom - top)
            omega_y = bottom - ((sample.filtered_theta_dot_deg_s - theta_min) / theta_span) * (bottom - top)
            theta_points.extend((x, theta_y))
            speed_points.extend((x, speed_y))
            omega_points.extend((x, omega_y))

        if len(theta_points) >= 4:
            self.canvas.create_line(*theta_points, fill="#0b6efd", width=2)
        if len(speed_points) >= 4:
            self.canvas.create_line(*speed_points, fill="#e5532d", width=2)
        if len(omega_points) >= 4:
            self.canvas.create_line(*omega_points, fill="#2da44e", width=2)

        self._draw_vertical_axis_ticks(
            axis_x=left,
            top=top,
            bottom=bottom,
            value_min=theta_min,
            value_max=theta_max,
            color="#0b6efd",
            label_suffix="deg",
            tick_side="left",
        )
        self._draw_vertical_axis_ticks(
            axis_x=right,
            top=top,
            bottom=bottom,
            value_min=speed_min,
            value_max=speed_max,
            color="#e5532d",
            label_suffix="mm/s",
            tick_side="right",
        )
        self.canvas.create_text(
            right,
            bottom + 32,
            text=f"{self.config.history_window_s:.1f} s window",
            anchor="e",
            fill="#555555",
            font=("Segoe UI", 9),
        )

        if theta_zero_y is not None:
            self.canvas.create_text(left + 24, theta_zero_y - 10, text="theta / omega 0", fill="#0b6efd", font=("Segoe UI", 9))
        if speed_zero_y is not None:
            self.canvas.create_text(right - 24, speed_zero_y - 10, text="speed 0", anchor="e", fill="#e5532d", font=("Segoe UI", 9))

        legend_x = right - 180
        self.canvas.create_line(legend_x, top + 12, legend_x + 24, top + 12, fill="#0b6efd", width=2)
        self.canvas.create_text(legend_x + 30, top + 12, text="filtered angle", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_line(legend_x, top + 30, legend_x + 24, top + 30, fill="#e5532d", width=2)
        self.canvas.create_text(legend_x + 30, top + 30, text="command speed", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_line(legend_x, top + 48, legend_x + 24, top + 48, fill="#2da44e", width=2)
        self.canvas.create_text(legend_x + 30, top + 48, text="filtered omega", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_rectangle(legend_x, top + 60, legend_x + 24, top + 72, fill="#0b6efd", stipple="gray12", outline="")
        self.canvas.create_text(legend_x + 30, top + 66, text="settle theta", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_rectangle(legend_x, top + 78, legend_x + 24, top + 90, fill="#2da44e", stipple="gray25", outline="")
        self.canvas.create_text(legend_x + 30, top + 84, text="settle omega", anchor="w", fill="#555555", font=("Segoe UI", 9))

    def _draw_vertical_axis_ticks(
        self,
        *,
        axis_x: float,
        top: float,
        bottom: float,
        value_min: float,
        value_max: float,
        color: str,
        label_suffix: str,
        tick_side: str,
    ) -> None:
        tick_count = 5
        height = bottom - top
        if height <= 0:
            return

        for index in range(tick_count + 1):
            fraction = index / tick_count
            y = bottom - fraction * height
            value = value_min + fraction * (value_max - value_min)
            self.canvas.create_line(axis_x - 4, y, axis_x + 4, y, fill="#c7c7c7")

            if tick_side == "left":
                self.canvas.create_text(
                    axis_x - 8,
                    y,
                    text=f"{value:.1f}",
                    anchor="e",
                    fill=color,
                    font=("Segoe UI", 9),
                )
            else:
                self.canvas.create_text(
                    axis_x + 8,
                    y,
                    text=f"{value:.1f}",
                    anchor="w",
                    fill=color,
                    font=("Segoe UI", 9),
                )

        if tick_side == "left":
            self.canvas.create_text(axis_x - 8, top - 12, text=label_suffix, anchor="e", fill=color, font=("Segoe UI", 9, "bold"))
        else:
            self.canvas.create_text(axis_x + 8, top - 12, text=label_suffix, anchor="w", fill=color, font=("Segoe UI", 9, "bold"))

    def _build_slider(
        self,
        parent: Frame,
        label_text: str,
        variable: DoubleVar,
        minimum: float,
        maximum: float,
        resolution: float,
    ) -> None:
        row = Frame(parent)
        row.pack(fill=X, pady=(0, 10))
        label = Label(row, text=label_text, anchor="w", font=("Segoe UI", 10, "bold"))
        label.pack(fill=X)
        scale = Scale(
            row,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient="horizontal",
            variable=variable,
            length=300,
            command=self._handle_weight_change,
        )
        scale.pack(fill=X)

    def _handle_weight_change(self, _value: str) -> None:
        self.runtime.update_weights(
            q_x=self.q_x_var.get(),
            q_x_dot=self.q_x_dot_var.get(),
            q_theta=self.q_theta_var.get(),
            q_theta_dot=self.q_theta_dot_var.get(),
            r_input=self.r_input_var.get(),
        )
        self.runtime.update_deadbands(
            theta_deadband_rad=self.theta_deadband_var.get(),
            theta_dot_deadband_rad_s=self.theta_dot_deadband_var.get() * (3.141592653589793 / 180.0),
            acceleration_deadband_m_s2=self.accel_deadband_var.get(),
            control_trigger_theta_rad=self.trigger_theta_var.get(),
            control_trigger_theta_dot_rad_s=self.trigger_theta_dot_var.get() * (3.141592653589793 / 180.0),
            control_trigger_samples=int(round(self.trigger_samples_var.get())),
            settle_theta_rad=self.settle_theta_var.get() * (3.141592653589793 / 180.0),
            settle_theta_dot_rad_s=self.settle_theta_dot_var.get() * (3.141592653589793 / 180.0),
            settle_samples=int(round(self.settle_samples_var.get())),
        )

    def _zero_encoder(self) -> None:
        self.runtime.encoder.zero_current_position()
        self.runtime.encoder.clear_snapshots()
        self.runtime._set_status("Encoder zero command sent.")

    def _handle_close(self) -> None:
        self.runtime.shutdown()
        self.root.quit()
        self.root.destroy()


def run_upright_control_ui(
    *,
    actuator_port: str = DEFAULT_UI_ACTUATOR_PORT,
    limits_port: str = DEFAULT_UI_LIMITS_PORT,
    encoder_port: str = DEFAULT_UI_ENCODER_PORT,
    baudrate: int = 115200,
    motion_config: MotionConfig | None = None,
    control_config: UprightControlConfig | None = None,
    ui_config: UprightControlUiConfig | None = None,
) -> None:
    runtime = UprightControlRuntime(
        actuator_port=actuator_port,
        limits_port=limits_port,
        encoder_port=encoder_port,
        baudrate=baudrate,
        motion_config=motion_config or MotionConfig(),
        control_config=control_config or UprightControlConfig(),
    )
    app = UprightControlUiApp(runtime=runtime, config=ui_config or UprightControlUiConfig())
    app.run()
