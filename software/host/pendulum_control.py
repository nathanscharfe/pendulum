from __future__ import annotations

import csv
from dataclasses import dataclass, fields, replace
from datetime import datetime
from pathlib import Path
from threading import Event, Thread
from time import monotonic, sleep
from typing import Callable

import numpy as np
from scipy.linalg import solve_continuous_are

from .arduino_actuator import ActuatorController
from .arduino_encoder import EncoderReader
from .arduino_limits import LimitSensorReader
from .motion_control import MotionConfig, MotionController
from .serial_worker import SerialSnapshot


@dataclass(frozen=True)
class PendulumControlConfig:
    period_s: float = 0.05
    lqr_mode: str = "python"
    pendulum_length_m: float = 0.500
    lqr_q_x: float = 1.0
    lqr_q_x_dot: float = 0.1
    lqr_q_theta: float = 5.0
    lqr_q_theta_dot: float = 1.0
    lqr_r_input: float = 10.0
    lqr_x_gain: float = 0.3162
    lqr_x_dot_gain: float = 0.8139
    lqr_theta_gain: float = -0.3088
    lqr_theta_dot_gain: float = -0.3479
    center_gain_per_s: float = 0.0
    estimator: str = "alpha-beta"
    theta_alpha: float = 0.35
    theta_beta: float = 0.04
    max_theta_residual_rad: float = 0.01
    armed_theta_alpha: float = 0.40
    armed_theta_beta: float = 0.06
    armed_max_theta_residual_rad: float = 0.025
    theta_filter_alpha: float = 0.25
    theta_dot_filter_alpha: float = 0.20
    max_theta_rate_rad_s: float = 3.0
    theta_deadband_rad: float = 0.003
    theta_dot_deadband_rad_s: float = 0.10
    acceleration_deadband_m_s2: float = 0.005
    control_trigger_theta_rad: float = 0.045
    control_trigger_theta_dot_rad_s: float = 0.30
    control_trigger_samples: int = 2
    settle_theta_rad: float = 0.03
    settle_theta_dot_rad_s: float = 0.15
    settle_samples: int = 25
    settle_raw_theta_rad: float = 0.03
    settle_theta_residual_rad: float = 0.02
    settle_command_speed_mm_s: float = 25.0
    velocity_leak_per_s: float = 0.0
    max_acceleration_m_s2: float = 0.5
    max_speed_mm_s: float = 150.0
    max_abs_theta_rad: float = 0.35
    actuator_command_sign: float = 1.0
    cart_position_sign: float = 1.0
    encoder_angle_sign: float = 1.0
    zero_encoder_on_start: bool = True
    home_before_start: bool = True
    move_middle_before_start: bool = True
    home_speed_mm_s: float = 50.0
    middle_speed_mm_s: float = 50.0


@dataclass(frozen=True)
class DownwardControlConfig(PendulumControlConfig):
    pass


@dataclass(frozen=True)
class UprightControlConfig(PendulumControlConfig):
    period_s: float = 0.01
    lqr_q_theta: float = 50.0
    lqr_r_input: float = 1.0
    lqr_x_gain: float = -1.0
    lqr_x_dot_gain: float = -2.0104
    lqr_theta_gain: float = -29.1450
    lqr_theta_dot_gain: float = -6.5238
    armed_theta_alpha: float = 0.40
    armed_theta_beta: float = 0.06
    control_trigger_theta_rad: float = 0.0
    control_trigger_theta_dot_rad_s: float = 0.0
    control_trigger_samples: int = 0
    settle_theta_rad: float = 0.01
    settle_theta_dot_rad_s: float = 0.05
    settle_samples: int = 50
    settle_raw_theta_rad: float = 0.01
    settle_theta_residual_rad: float = 0.005
    settle_command_speed_mm_s: float = 10.0
    max_acceleration_m_s2: float = 30.0
    max_speed_mm_s: float = 350.0


CONTROL_FIELDS = [
    "host_elapsed_s",
    "host_timestamp_s",
    "encoder_time_ms",
    "theta_deg",
    "theta_rad",
    "omega_rad_s",
    "filtered_theta_rad",
    "filtered_theta_dot_rad_s",
    "theta_residual_rad",
    "control_x_m",
    "control_x_dot_m_s",
    "control_theta_rad",
    "control_theta_dot_rad_s",
    "control_armed",
    "trigger_sample_count",
    "settle_sample_count",
    "actuator_command_sign",
    "cart_position_sign",
    "encoder_angle_sign",
    "raw_count",
    "filtered_count",
    "unwrapped_count",
    "connected",
    "magnet_detected",
    "magnet_too_weak",
    "magnet_too_strong",
    "agc",
    "magnitude",
    "actuator_time_ms",
    "position_steps",
    "position_mm",
    "motion_mode",
    "enabled",
    "left_limit",
    "right_limit",
    "any_limit",
    "command_speed_mm_s",
    "command_step_rate_s",
    "command_acceleration_m_s2",
    "stop_reason",
]

CONFIG_LOG_FIELDS = [f"config_{field.name}" for field in fields(UprightControlConfig)]


def default_output_path(prefix: str = "downward_control", subdir: str = "downward") -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware") / "control experiments" / subdir / f"{prefix}_{timestamp}.csv"


def run_downward_control(
    actuator: ActuatorController,
    limits: LimitSensorReader,
    encoder: EncoderReader,
    motion: MotionController,
    motion_config: MotionConfig,
    control_config: PendulumControlConfig,
    output_path: Path | None = None,
    output_prefix: str = "downward_control",
    output_subdir: str = "downward",
    experiment_name: str = "Downward pendulum control experiment",
    start_instruction: str = "Start with the cart near center and the pendulum hanging downward and motionless.",
    start_prompt: str = "Let the pendulum hang downward and motionless, then press Enter to zero the encoder and arm automatic control...",
    lqr_equilibrium: str = "downward",
    pre_setup_sequence: Callable[[LimitSensorReader], None] | None = None,
    startup_sequence: Callable[[LimitSensorReader, EncoderReader, PendulumControlConfig], None] | None = None,
) -> Path:
    control_config = resolve_lqr_config(control_config, equilibrium=lqr_equilibrium)
    output_path = output_path or default_output_path(output_prefix, output_subdir)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stop_event = Event()
    stop_reason = "operator_stop"
    rows: list[dict[str, object]] = []

    print(experiment_name)
    print(f"Output file: {output_path}")
    print(start_instruction)
    print(f"Control period: {control_config.period_s:.3f} s")
    print(f"Speed clamp: +/-{control_config.max_speed_mm_s:.3f} mm/s")
    print(f"Acceleration clamp: +/-{control_config.max_acceleration_m_s2:.3f} m/s^2")
    print(f"Angle cutoff: +/-{control_config.max_abs_theta_rad:.3f} rad")
    if control_config.home_before_start:
        print(f"Setup: home left at {control_config.home_speed_mm_s:g} mm/s")
    if control_config.move_middle_before_start:
        print(f"Setup: move to midpoint at {control_config.middle_speed_mm_s:g} mm/s")
    print("Controller: full-state LQR acceleration command integrated to actuator speed")
    print("u_m_s2 = actuator_sign * -K[x, x_dot, theta, theta_dot], with filtering, deadbands, and speed clamps")
    print(
        "LQR synthesis: "
        f"mode={control_config.lqr_mode}, "
        f"l={control_config.pendulum_length_m:g} m, "
        f"Q=diag([{control_config.lqr_q_x:g}, {control_config.lqr_q_x_dot:g}, "
        f"{control_config.lqr_q_theta:g}, {control_config.lqr_q_theta_dot:g}]), "
        f"R={control_config.lqr_r_input:g}"
    )
    print(
        "Gains: "
        f"Kx={control_config.lqr_x_gain:g}, "
        f"Kx_dot={control_config.lqr_x_dot_gain:g}, "
        f"Ktheta={control_config.lqr_theta_gain:g}, "
        f"Ktheta_dot={control_config.lqr_theta_dot_gain:g}, "
        f"Kcenter={control_config.center_gain_per_s:g}"
    )
    print(
        "Signs: "
        f"actuator_command={control_config.actuator_command_sign:g}, "
        f"cart_position={control_config.cart_position_sign:g}, "
        f"encoder_angle={control_config.encoder_angle_sign:g}"
    )
    print(
        "Estimator: "
        f"{control_config.estimator}, "
        f"alpha={control_config.theta_alpha:g}, "
        f"beta={control_config.theta_beta:g}, "
        f"residual clamp={control_config.max_theta_residual_rad:g} rad"
    )
    print(
        "Armed estimator: "
        f"alpha={control_config.armed_theta_alpha:g}, "
        f"beta={control_config.armed_theta_beta:g}, "
        f"residual clamp={control_config.armed_max_theta_residual_rad:g} rad"
    )
    print(
        "Deadbands: "
        f"theta={control_config.theta_deadband_rad:g} rad, "
        f"theta_dot={control_config.theta_dot_deadband_rad_s:g} rad/s, "
        f"accel={effective_acceleration_deadband(control_config):g} m/s^2"
    )
    print(
        "Control trigger: "
        f"|theta|>{control_config.control_trigger_theta_rad:g} rad or "
        f"|theta_dot|>{control_config.control_trigger_theta_dot_rad_s:g} rad/s "
        f"for {control_config.control_trigger_samples} samples"
    )
    print(
        "Settle disarm: "
        f"|theta|<{control_config.settle_theta_rad:g} rad and "
        f"|theta_dot|<{control_config.settle_theta_dot_rad_s:g} rad/s "
        f"for {control_config.settle_samples} samples"
    )
    print(f"Theta slew-rate limit: {control_config.max_theta_rate_rad_s:g} rad/s")

    if pre_setup_sequence is not None:
        pre_setup_sequence(limits)

    if control_config.home_before_start:
        print("Homing left...")
        motion.home(side="left", speed_mm_s=control_config.home_speed_mm_s)

    if control_config.move_middle_before_start:
        print("Moving to middle...")
        motion.move_to_middle(speed_mm_s=control_config.middle_speed_mm_s)

    if startup_sequence is None:
        input(start_prompt)

        if control_config.zero_encoder_on_start:
            encoder.zero_current_position()
            sleep(0.2)
    else:
        startup_sequence(limits, encoder, control_config)

    arm_reference_snapshot = encoder.get_latest()
    arm_reference_timestamp_s = arm_reference_snapshot.timestamp_s if arm_reference_snapshot is not None else 0.0

    actuator.clear_snapshots()
    limits.clear_snapshots()
    encoder.clear_snapshots()

    actuator.enable()
    start_enter_stop_thread(stop_event)

    fresh_encoder_snapshot = wait_for_new_snapshot(
        encoder,
        newer_than_timestamp_s=arm_reference_timestamp_s,
        timeout_s=max(1.0, 5.0 * control_config.period_s),
    )
    if fresh_encoder_snapshot is None:
        actuator.stop_motion()
        actuator.disable()
        raise RuntimeError("No fresh encoder sample arrived after arming control.")

    start_s = monotonic()
    next_loop_s = start_s
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
    print("Control is running. Press Enter to stop, or Ctrl+C for immediate stop.")

    try:
        while not stop_event.is_set():
            now_s = monotonic()
            if now_s < next_loop_s:
                sleep(min(0.005, next_loop_s - now_s))
                continue

            next_loop_s += control_config.period_s

            encoder_snapshot = encoder.get_latest()
            actuator_snapshot = actuator.get_latest()
            limits_snapshot = latest_limit_state(limits)

            if encoder_snapshot is None:
                stop_reason = "no_encoder_data"
                break

            if encoder_snapshot.timestamp_s <= last_encoder_timestamp_s:
                sleep(0.001)
                continue
            last_encoder_timestamp_s = encoder_snapshot.timestamp_s

            if actuator_snapshot is None:
                actuator.request_status()

            if limits_snapshot is None:
                limits.request_state()
                stop_reason = "no_limit_data"
                break

            left_limit = bool(limits_snapshot.data.get("left_limit", False))
            right_limit = bool(limits_snapshot.data.get("right_limit", False))
            any_limit = bool(limits_snapshot.data.get("any_limit", left_limit or right_limit))
            if any_limit:
                stop_reason = "limit_active"
                break

            raw_theta_rad = float(encoder_snapshot.data["theta_rad"])
            raw_omega_rad_s = float(encoder_snapshot.data["omega_rad_s"])
            if abs(raw_theta_rad) > control_config.max_abs_theta_rad:
                stop_reason = "angle_cutoff"
                break

            dt_s = control_config.period_s
            theta_rad = control_config.encoder_angle_sign * raw_theta_rad
            if control_config.estimator == "alpha-beta":
                theta_alpha = control_config.armed_theta_alpha if control_armed else control_config.theta_alpha
                theta_beta = control_config.armed_theta_beta if control_armed else control_config.theta_beta
                max_theta_residual_rad = (
                    control_config.armed_max_theta_residual_rad
                    if control_armed
                    else control_config.max_theta_residual_rad
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
                limited_theta_rad = slew_limit(
                    previous=filtered_theta_rad,
                    value=theta_rad,
                    max_rate=control_config.max_theta_rate_rad_s,
                    dt_s=dt_s,
                )
                previous_filtered_theta_rad = filtered_theta_rad
                filtered_theta_rad = lowpass(filtered_theta_rad, limited_theta_rad, control_config.theta_filter_alpha)
                theta_dot_rad_s = (
                    0.0 if previous_filtered_theta_rad is None else (filtered_theta_rad - previous_filtered_theta_rad) / dt_s
                )
                filtered_theta_dot_rad_s = lowpass(
                    filtered_theta_dot_rad_s,
                    theta_dot_rad_s,
                    control_config.theta_dot_filter_alpha,
                )
                theta_residual_rad = theta_rad - limited_theta_rad
            control_theta_rad = apply_deadband(filtered_theta_rad, control_config.theta_deadband_rad)
            control_theta_dot_rad_s = apply_deadband(
                filtered_theta_dot_rad_s,
                control_config.theta_dot_deadband_rad_s,
            )
            if not control_armed:
                trigger_sample_count = update_trigger_sample_count(
                    trigger_sample_count=trigger_sample_count,
                    theta_rad=filtered_theta_rad,
                    theta_dot_rad_s=filtered_theta_dot_rad_s,
                    config=control_config,
                )
                control_armed = trigger_sample_count >= control_config.control_trigger_samples
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
                    config=control_config,
                )
                if settle_sample_count >= control_config.settle_samples:
                    control_armed = False
                    trigger_sample_count = 0
                    settle_sample_count = 0
                    command_speed_mm_s = 0.0

            position_steps = int(actuator_snapshot.data["position_steps"]) if actuator_snapshot is not None else 0
            position_mm = position_steps / motion_config.steps_per_mm
            position_m = position_mm / 1000.0
            center_m = (motion_config.travel_mm / 1000.0) / 2.0
            control_x_m = control_config.cart_position_sign * (position_m - center_m)
            control_x_dot_m_s = control_config.cart_position_sign * command_speed_mm_s / 1000.0
            center_error_mm = position_mm - (motion_config.travel_mm / 2.0)

            if control_armed:
                command_acceleration_m_s2 = compute_lqr_acceleration(
                    x_m=control_x_m,
                    x_dot_m_s=control_x_dot_m_s,
                    theta_rad=control_theta_rad,
                    theta_dot_rad_s=control_theta_dot_rad_s,
                    config=control_config,
                )
            else:
                command_acceleration_m_s2 = 0.0
            if abs(command_acceleration_m_s2) < effective_acceleration_deadband(control_config):
                command_acceleration_m_s2 = 0.0

            leak = max(0.0, 1.0 - control_config.velocity_leak_per_s * dt_s)
            if control_armed:
                command_speed_mm_s = command_speed_mm_s * leak + command_acceleration_m_s2 * dt_s * 1000.0
                command_speed_mm_s -= control_config.center_gain_per_s * center_error_mm
            else:
                command_speed_mm_s = 0.0
            command_speed_mm_s = clamp(command_speed_mm_s, -control_config.max_speed_mm_s, control_config.max_speed_mm_s)
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
                actuator.set_step_rate(command_speed_mm_s * motion_config.steps_per_mm)
                last_command_speed_mm_s = command_speed_mm_s

            rows.append(
                build_log_row(
                    elapsed_s=now_s - start_s,
                    encoder_snapshot=encoder_snapshot,
                    actuator_snapshot=actuator_snapshot,
                    limits_snapshot=limits_snapshot,
                    position_mm=position_mm,
                    command_speed_mm_s=command_speed_mm_s,
                    command_acceleration_m_s2=command_acceleration_m_s2,
                    filtered_theta_rad=filtered_theta_rad,
                    filtered_theta_dot_rad_s=filtered_theta_dot_rad_s,
                    theta_residual_rad=theta_residual_rad,
                    control_x_m=control_x_m,
                    control_x_dot_m_s=control_x_dot_m_s,
                    control_theta_rad=control_theta_rad,
                    control_theta_dot_rad_s=control_theta_dot_rad_s,
                    control_armed=control_armed,
                    trigger_sample_count=trigger_sample_count,
                    settle_sample_count=settle_sample_count,
                    motion_config=motion_config,
                    control_config=control_config,
                    stop_reason="",
                )
            )

    except KeyboardInterrupt:
        print("\nControl interrupted.")
        stop_reason = "keyboard_interrupt"
    finally:
        actuator.stop_motion()
        actuator.disable()

    final_encoder = encoder.get_latest()
    final_actuator = actuator.get_latest()
    final_limits = latest_limit_state(limits)
    if final_encoder is not None and final_limits is not None:
        position_steps = int(final_actuator.data["position_steps"]) if final_actuator is not None else 0
        final_position_mm = position_steps / motion_config.steps_per_mm
        final_x_m = control_config.cart_position_sign * (
            (final_position_mm / 1000.0) - ((motion_config.travel_mm / 1000.0) / 2.0)
        )
        rows.append(
            build_log_row(
                elapsed_s=monotonic() - start_s,
                encoder_snapshot=final_encoder,
                actuator_snapshot=final_actuator,
                limits_snapshot=final_limits,
                position_mm=final_position_mm,
                command_speed_mm_s=0.0,
                command_acceleration_m_s2=0.0,
                filtered_theta_rad=filtered_theta_rad or 0.0,
                filtered_theta_dot_rad_s=filtered_theta_dot_rad_s or 0.0,
                theta_residual_rad=theta_residual_rad,
                control_x_m=final_x_m,
                control_x_dot_m_s=0.0,
                control_theta_rad=filtered_theta_rad or 0.0,
                control_theta_dot_rad_s=filtered_theta_dot_rad_s or 0.0,
                control_armed=control_armed,
                trigger_sample_count=trigger_sample_count,
                settle_sample_count=settle_sample_count,
                motion_config=motion_config,
                control_config=control_config,
                stop_reason=stop_reason,
            )
        )

    write_rows(output_path, rows)
    write_snapshot_history(output_path.with_name(f"{output_path.stem}_encoder_samples.csv"), encoder.drain_snapshots())
    write_snapshot_history(output_path.with_name(f"{output_path.stem}_actuator_samples.csv"), actuator.drain_snapshots())
    write_snapshot_history(output_path.with_name(f"{output_path.stem}_limit_samples.csv"), limits.drain_snapshots())
    print(f"Stopped: {stop_reason}")
    print(f"Wrote {len(rows)} control rows to {output_path}")
    print("Wrote sidecar raw sample logs next to the control CSV")
    return output_path


def run_upright_control(
    actuator: ActuatorController,
    limits: LimitSensorReader,
    encoder: EncoderReader,
    motion: MotionController,
    motion_config: MotionConfig,
    control_config: UprightControlConfig,
    output_path: Path | None = None,
) -> Path:
    return run_downward_control(
        actuator,
        limits,
        encoder,
        motion,
        motion_config,
        control_config=control_config,
        output_path=output_path,
        output_prefix="upright_control",
        output_subdir="upright",
        experiment_name="Upright pendulum control experiment",
        start_instruction="Start with the cart near center and the pendulum held upright.",
        start_prompt="",
        lqr_equilibrium="upright",
        pre_setup_sequence=release_upright_servo_for_setup,
        startup_sequence=upright_servo_startup_sequence,
    )


def compute_lqr_acceleration(
    x_m: float,
    x_dot_m_s: float,
    theta_rad: float,
    theta_dot_rad_s: float,
    config: PendulumControlConfig,
) -> float:
    model_acceleration_m_s2 = -(
        config.lqr_x_gain * x_m
        + config.lqr_x_dot_gain * x_dot_m_s
        + config.lqr_theta_gain * theta_rad
        + config.lqr_theta_dot_gain * theta_dot_rad_s
    )
    command = config.actuator_command_sign * model_acceleration_m_s2
    return clamp(command, -config.max_acceleration_m_s2, config.max_acceleration_m_s2)


def resolve_lqr_config(config: PendulumControlConfig, equilibrium: str) -> PendulumControlConfig:
    if config.lqr_mode == "manual":
        return config

    k_x, k_x_dot, k_theta, k_theta_dot = synthesize_lqr_gains(
        pendulum_length_m=config.pendulum_length_m,
        q_x=config.lqr_q_x,
        q_x_dot=config.lqr_q_x_dot,
        q_theta=config.lqr_q_theta,
        q_theta_dot=config.lqr_q_theta_dot,
        r_input=config.lqr_r_input,
        equilibrium=equilibrium,
    )
    return replace(
        config,
        lqr_x_gain=k_x,
        lqr_x_dot_gain=k_x_dot,
        lqr_theta_gain=k_theta,
        lqr_theta_dot_gain=k_theta_dot,
    )


def synthesize_lqr_gains(
    pendulum_length_m: float,
    q_x: float,
    q_x_dot: float,
    q_theta: float,
    q_theta_dot: float,
    r_input: float,
    equilibrium: str,
) -> tuple[float, float, float, float]:
    if pendulum_length_m <= 0.0:
        raise ValueError("pendulum_length_m must be positive")
    if any(weight <= 0.0 for weight in (q_x, q_x_dot, q_theta, q_theta_dot, r_input)):
        raise ValueError("LQR Q and R weights must be positive")

    gravity_term = 9.81 / pendulum_length_m
    if equilibrium == "upright":
        a_43 = gravity_term
    elif equilibrium == "downward":
        a_43 = -gravity_term
    else:
        raise ValueError(f"unknown equilibrium {equilibrium}")

    a_matrix = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, a_43, 0.0],
        ],
        dtype=float,
    )
    b_matrix = np.array([[0.0], [1.0], [0.0], [-1.0 / pendulum_length_m]], dtype=float)
    q_matrix = np.diag([q_x, q_x_dot, q_theta, q_theta_dot]).astype(float)
    r_matrix = np.array([[r_input]], dtype=float)

    p_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    gain_matrix = np.linalg.solve(r_matrix, b_matrix.T @ p_matrix)
    return tuple(float(value) for value in gain_matrix.ravel())


def update_alpha_beta_estimate(
    theta_estimate_rad: float | None,
    theta_dot_estimate_rad_s: float | None,
    measured_theta_rad: float,
    dt_s: float,
    alpha: float,
    beta: float,
    max_residual_rad: float,
) -> tuple[float, float, float]:
    if theta_estimate_rad is None or theta_dot_estimate_rad_s is None:
        return measured_theta_rad, 0.0, 0.0

    predicted_theta_rad = theta_estimate_rad + theta_dot_estimate_rad_s * dt_s
    residual_rad = measured_theta_rad - predicted_theta_rad
    limited_residual_rad = clamp(residual_rad, -max_residual_rad, max_residual_rad)

    next_theta_rad = predicted_theta_rad + alpha * limited_residual_rad
    next_theta_dot_rad_s = theta_dot_estimate_rad_s + beta * limited_residual_rad / dt_s
    return next_theta_rad, next_theta_dot_rad_s, residual_rad


def update_trigger_sample_count(
    trigger_sample_count: int,
    theta_rad: float,
    theta_dot_rad_s: float,
    config: PendulumControlConfig,
) -> int:
    triggered = (
        abs(theta_rad) >= config.control_trigger_theta_rad
        or abs(theta_dot_rad_s) >= config.control_trigger_theta_dot_rad_s
    )
    if triggered:
        return trigger_sample_count + 1
    return 0


def update_settle_sample_count(
    settle_sample_count: int,
    raw_theta_rad: float,
    theta_rad: float,
    theta_dot_rad_s: float,
    theta_residual_rad: float,
    command_speed_mm_s: float,
    config: PendulumControlConfig,
) -> int:
    settled = (
        abs(raw_theta_rad) <= config.settle_raw_theta_rad
        and abs(theta_rad) <= config.settle_theta_rad
        and abs(theta_dot_rad_s) <= config.settle_theta_dot_rad_s
        and abs(theta_residual_rad) <= config.settle_theta_residual_rad
        and abs(command_speed_mm_s) <= config.settle_command_speed_mm_s
    )
    if settled:
        return settle_sample_count + 1
    return 0


def effective_acceleration_deadband(config: PendulumControlConfig) -> float:
    if config.max_acceleration_m_s2 <= 0.0:
        return 0.0
    return min(config.acceleration_deadband_m_s2, 0.5 * config.max_acceleration_m_s2)


def upright_servo_startup_sequence(
    limits: LimitSensorReader,
    encoder: EncoderReader,
    control_config: PendulumControlConfig,
) -> None:
    print("Upright staging loop:")
    print("  p     move servo to HOLD (80 deg)")
    print("  g     move servo to RELEASE (0 deg)")
    print("  start zero encoder, release servo, and begin automatic control")

    while True:
        command = input("staging> ").strip().lower()
        if command == "p":
            print("Servo: hold latch.")
            limits.hold_servo()
            sleep(0.2)
            continue
        if command == "g":
            print("Servo: release latch.")
            limits.release_servo()
            sleep(0.2)
            report_release_fall_time(encoder, threshold_deg=10.0, timeout_s=5.0)
            continue
        if command == "start":
            break
        if not command:
            print("Type 'p', 'g', or 'start'.")
            continue
        print("Unknown staging command. Type 'p', 'g', or 'start'.")

    if control_config.zero_encoder_on_start:
        encoder.zero_current_position()
        sleep(0.1)

    print("Servo: release latch and start control.")
    limits.release_servo()
    sleep(0.05)


def release_upright_servo_for_setup(limits: LimitSensorReader) -> None:
    print("Servo: release latch for setup moves.")
    limits.release_servo()
    sleep(0.2)


def report_release_fall_time(
    encoder: EncoderReader,
    threshold_deg: float,
    timeout_s: float,
) -> None:
    release_snapshot = encoder.get_latest()
    if release_snapshot is None:
        print("Release timing: no encoder data available.")
        return

    release_theta_rad = float(release_snapshot.data["theta_rad"])
    threshold_rad = threshold_deg * 3.141592653589793 / 180.0
    start_s = monotonic()

    while True:
        current_snapshot = encoder.get_latest()
        if current_snapshot is not None:
            current_theta_rad = float(current_snapshot.data["theta_rad"])
            if abs(current_theta_rad - release_theta_rad) >= threshold_rad:
                elapsed_s = monotonic() - start_s
                print(f"Release timing: crossed +/-{threshold_deg:g} deg after {elapsed_s:.3f} s")
                return

        elapsed_s = monotonic() - start_s
        if elapsed_s >= timeout_s:
            print(f"Release timing: did not cross +/-{threshold_deg:g} deg within {timeout_s:.1f} s")
            return

        sleep(0.005)


def start_enter_stop_thread(stop_event: Event) -> None:
    def wait_for_enter() -> None:
        try:
            input()
        except EOFError:
            pass
        stop_event.set()

    Thread(target=wait_for_enter, name="downward-control-stop", daemon=True).start()


def latest_limit_state(limits: LimitSensorReader) -> SerialSnapshot | None:
    latest = limits.get_latest()
    if latest is None:
        return None
    if "left_limit" not in latest.data or "right_limit" not in latest.data:
        limits.request_state()
        sleep(0.02)
        latest = limits.get_latest()
    if latest is None or "left_limit" not in latest.data or "right_limit" not in latest.data:
        return None
    return latest


def wait_for_new_snapshot(
    worker: EncoderReader,
    newer_than_timestamp_s: float,
    timeout_s: float,
) -> SerialSnapshot | None:
    start_s = monotonic()
    while True:
        snapshot = worker.get_latest()
        if snapshot is not None and snapshot.timestamp_s > newer_than_timestamp_s:
            return snapshot
        if monotonic() - start_s >= timeout_s:
            return None
        sleep(0.001)


def build_log_row(
    elapsed_s: float,
    encoder_snapshot: SerialSnapshot,
    actuator_snapshot: SerialSnapshot | None,
    limits_snapshot: SerialSnapshot,
    position_mm: float,
    command_speed_mm_s: float,
    command_acceleration_m_s2: float,
    filtered_theta_rad: float,
    filtered_theta_dot_rad_s: float,
    theta_residual_rad: float,
    control_x_m: float,
    control_x_dot_m_s: float,
    control_theta_rad: float,
    control_theta_dot_rad_s: float,
    control_armed: bool,
    trigger_sample_count: int,
    settle_sample_count: int,
    motion_config: MotionConfig,
    control_config: PendulumControlConfig,
    stop_reason: str,
) -> dict[str, object]:
    actuator_data = actuator_snapshot.data if actuator_snapshot is not None else {}
    command_step_rate_s = command_speed_mm_s * motion_config.steps_per_mm
    row = {
        "host_elapsed_s": f"{elapsed_s:.6f}",
        "host_timestamp_s": f"{encoder_snapshot.timestamp_s:.6f}",
        "encoder_time_ms": encoder_snapshot.data["time_ms"],
        "theta_deg": encoder_snapshot.data["theta_deg"],
        "theta_rad": encoder_snapshot.data["theta_rad"],
        "omega_rad_s": encoder_snapshot.data["omega_rad_s"],
        "filtered_theta_rad": f"{filtered_theta_rad:.6f}",
        "filtered_theta_dot_rad_s": f"{filtered_theta_dot_rad_s:.6f}",
        "theta_residual_rad": f"{theta_residual_rad:.6f}",
        "control_x_m": f"{control_x_m:.6f}",
        "control_x_dot_m_s": f"{control_x_dot_m_s:.6f}",
        "control_theta_rad": f"{control_theta_rad:.6f}",
        "control_theta_dot_rad_s": f"{control_theta_dot_rad_s:.6f}",
        "control_armed": control_armed,
        "trigger_sample_count": trigger_sample_count,
        "settle_sample_count": settle_sample_count,
        "actuator_command_sign": f"{control_config.actuator_command_sign:g}",
        "cart_position_sign": f"{control_config.cart_position_sign:g}",
        "encoder_angle_sign": f"{control_config.encoder_angle_sign:g}",
        "raw_count": encoder_snapshot.data["raw_count"],
        "filtered_count": encoder_snapshot.data.get("filtered_count", encoder_snapshot.data["raw_count"]),
        "unwrapped_count": encoder_snapshot.data["unwrapped_count"],
        "connected": encoder_snapshot.data["connected"],
        "magnet_detected": encoder_snapshot.data["magnet_detected"],
        "magnet_too_weak": encoder_snapshot.data["magnet_too_weak"],
        "magnet_too_strong": encoder_snapshot.data["magnet_too_strong"],
        "agc": encoder_snapshot.data["agc"],
        "magnitude": encoder_snapshot.data["magnitude"],
        "actuator_time_ms": actuator_data.get("time_ms", ""),
        "position_steps": actuator_data.get("position_steps", ""),
        "position_mm": f"{position_mm:.6f}",
        "motion_mode": actuator_data.get("motion_mode", ""),
        "enabled": actuator_data.get("enabled", ""),
        "left_limit": limits_snapshot.data.get("left_limit", ""),
        "right_limit": limits_snapshot.data.get("right_limit", ""),
        "any_limit": limits_snapshot.data.get("any_limit", ""),
        "command_speed_mm_s": f"{command_speed_mm_s:.6f}",
        "command_step_rate_s": f"{command_step_rate_s:.6f}",
        "command_acceleration_m_s2": f"{command_acceleration_m_s2:.6f}",
        "stop_reason": stop_reason,
    }
    row.update(
        {
            f"config_{field.name}": format_config_value(getattr(control_config, field.name))
            for field in fields(control_config)
        }
    )
    return row


def write_rows(output_path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [*CONTROL_FIELDS, *CONFIG_LOG_FIELDS]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_snapshot_history(output_path: Path, snapshots: list[SerialSnapshot]) -> None:
    fieldnames = sorted({key for snapshot in snapshots for key in snapshot.data})
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["host_timestamp_s", "raw_line", *fieldnames])
        writer.writeheader()
        for snapshot in snapshots:
            row = {"host_timestamp_s": f"{snapshot.timestamp_s:.6f}", "raw_line": snapshot.raw_line}
            row.update(snapshot.data)
            writer.writerow(row)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def format_config_value(value: object) -> object:
    if isinstance(value, float):
        return f"{value:.6f}"
    return value


def lowpass(previous: float | None, value: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    if previous is None:
        return value
    return previous + alpha * (value - previous)


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) <= deadband:
        return 0.0
    return value


def slew_limit(previous: float | None, value: float, max_rate: float, dt_s: float) -> float:
    if previous is None or max_rate <= 0.0 or dt_s <= 0.0:
        return value
    max_delta = max_rate * dt_s
    return previous + clamp(value - previous, -max_delta, max_delta)
