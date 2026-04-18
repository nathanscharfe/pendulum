from __future__ import annotations

import argparse
import time
from contextlib import ExitStack
from pathlib import Path

from .arduino_actuator import ActuatorController
from .arduino_encoder import EncoderReader
from .arduino_limits import LimitSensorReader
from .downward_control import DownwardControlConfig, run_downward_control
from .encoder_capture import run_encoder_capture
from .motion_control import DEFAULT_STEPS_PER_MM, MotionConfig, MotionController, MotionSafetyError
from .serial_worker import SerialWorker
from .travel_calibration import DEFAULT_RELATIVE_STEPS, run_travel_calibration


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Initial host monitor for the three-Arduino pendulum setup.")
    parser.add_argument("--actuator-port", help="Serial port for Arduino #1 actuator controller, e.g. COM3.")
    parser.add_argument("--limits-port", help="Serial port for Arduino #2 limit sensor reader, e.g. COM4.")
    parser.add_argument("--encoder-port", help="Serial port for Arduino #3 AS5600 encoder reader, e.g. COM5.")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--poll-period", type=float, default=0.5, help="Seconds between printed snapshots.")
    parser.add_argument("--duration", type=float, help="Optional run duration in seconds.")
    parser.add_argument("--zero-encoder-on-start", action="store_true")
    parser.add_argument("--steps-per-mm", type=float, default=DEFAULT_STEPS_PER_MM)
    parser.add_argument("--travel-mm", type=float, default=600.0)

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("monitor", help="Read and print latest snapshots from configured Arduinos.")

    home = subparsers.add_parser("home", help="Home the actuator toward a limit switch.")
    home.add_argument("--side", choices=["left", "right"], default="left")
    home.add_argument("--speed-mm-s", type=float, default=25.0)

    move = subparsers.add_parser("move-mm", help="Move to an absolute position in millimeters after homing.")
    move.add_argument("target_mm", type=float)
    move.add_argument("--speed-mm-s", type=float, default=25.0)

    middle = subparsers.add_parser("middle", help="Move to the configured midpoint after homing.")
    middle.add_argument("--speed-mm-s", type=float, default=25.0)

    move_steps = subparsers.add_parser("move-steps", help="Move a relative number of raw actuator steps.")
    move_steps.add_argument("steps", type=int)
    move_steps.add_argument("--steps-per-second", type=float, default=100.0)

    goto_steps = subparsers.add_parser("goto-steps", help="Move to an absolute raw actuator step position.")
    goto_steps.add_argument("target_steps", type=int)
    goto_steps.add_argument("--steps-per-second", type=float, default=100.0)

    calibrate = subparsers.add_parser("calibrate-travel", help="Interactively collect tape-measure travel calibration data.")
    calibrate.add_argument("--output", type=Path, help="CSV output path.")
    calibrate.add_argument("--start-steps", type=int, default=100)
    calibrate.add_argument("--speeds", type=float, nargs="+", help="Optional queued step rates. Omit to choose speeds interactively.")
    calibrate.add_argument("--moves", type=int, nargs="+", default=list(DEFAULT_RELATIVE_STEPS))
    calibrate.add_argument("--home-speed-mm-s", type=float, default=10.0)
    calibrate.add_argument("--reposition-speed-steps-s", type=float, default=300.0)

    speed = subparsers.add_parser("speed", help="Run a signed speed command with host-side limit protection.")
    speed.add_argument("speed_mm_s", type=float)
    speed.add_argument("--duration", type=float, required=True)

    capture_encoder = subparsers.add_parser("capture-encoder", help="Record pendulum encoder data until Enter is pressed.")
    capture_encoder.add_argument("--output", type=Path, help="CSV output path.")
    capture_encoder.add_argument("--no-zero-on-start", action="store_true", help="Keep the current encoder zero instead of zeroing at capture start.")

    control_down = subparsers.add_parser("control-down", help="Run a first-pass downward pendulum damping controller and log data.")
    control_down.add_argument("--output", type=Path, help="CSV output path.")
    control_down.add_argument("--period-s", type=float, default=0.05)
    control_down.add_argument("--x-gain", type=float, default=0.3162, help="Downward LQR cart-position gain.")
    control_down.add_argument("--x-dot-gain", type=float, default=0.8139, help="Downward LQR cart-velocity gain.")
    control_down.add_argument("--theta-gain", type=float, default=-0.3088, help="Downward LQR theta gain.")
    control_down.add_argument("--omega-gain", type=float, default=-0.3479, help="Downward LQR theta-dot gain.")
    control_down.add_argument("--center-gain", type=float, default=0.0, help="Cart centering gain in 1/s.")
    control_down.add_argument("--estimator", choices=["alpha-beta", "legacy"], default="alpha-beta")
    control_down.add_argument("--theta-alpha", type=float, default=0.45, help="Alpha-beta estimator angle correction gain.")
    control_down.add_argument("--theta-beta", type=float, default=0.08, help="Alpha-beta estimator angular-rate correction gain.")
    control_down.add_argument("--max-theta-residual-rad", type=float, default=0.02, help="Clamp alpha-beta angle innovation to reject encoder jumps.")
    control_down.add_argument("--armed-theta-alpha", type=float, default=0.55, help="Alpha-beta angle gain after control is armed.")
    control_down.add_argument("--armed-theta-beta", type=float, default=0.12, help="Alpha-beta angular-rate gain after control is armed.")
    control_down.add_argument("--armed-max-theta-residual-rad", type=float, default=0.05, help="Alpha-beta innovation clamp after control is armed.")
    control_down.add_argument("--theta-filter-alpha", type=float, default=0.25)
    control_down.add_argument("--theta-dot-filter-alpha", type=float, default=0.20)
    control_down.add_argument("--max-theta-rate-rad-s", type=float, default=3.0)
    control_down.add_argument("--theta-deadband-rad", type=float, default=0.003)
    control_down.add_argument("--omega-deadband-rad-s", type=float, default=0.10, help="Theta-dot deadband in rad/s.")
    control_down.add_argument("--accel-deadband-m-s2", type=float, default=0.005)
    control_down.add_argument("--control-trigger-theta-rad", type=float, default=0.045, help="Hold actuator still until estimated theta exceeds this magnitude.")
    control_down.add_argument("--control-trigger-omega-rad-s", type=float, default=0.30, help="Hold actuator still until estimated theta-dot exceeds this magnitude.")
    control_down.add_argument("--control-trigger-samples", type=int, default=2, help="Consecutive trigger samples required before actuator commands are allowed.")
    control_down.add_argument("--settle-theta-rad", type=float, default=0.025, help="Disarm control after estimated theta stays below this magnitude.")
    control_down.add_argument("--settle-omega-rad-s", type=float, default=0.12, help="Disarm control after estimated theta-dot stays below this magnitude.")
    control_down.add_argument("--settle-samples", type=int, default=25, help="Consecutive settled samples required before disarming control.")
    control_down.add_argument("--velocity-leak-per-s", type=float, default=0.0)
    control_down.add_argument("--max-accel-m-s2", type=float, default=0.5)
    control_down.add_argument("--max-speed-mm-s", type=float, default=150.0)
    control_down.add_argument("--max-angle-rad", type=float, default=0.35)
    control_down.add_argument(
        "--invert-control",
        action="store_true",
        help="Compatibility alias for --invert-actuator-command.",
    )
    control_down.add_argument(
        "--invert-actuator-command",
        action="store_true",
        help="Flip the sign between the LQR acceleration command and actuator motion.",
    )
    control_down.add_argument(
        "--invert-cart-position",
        action="store_true",
        help="Flip the sign of cart position and cart velocity before applying the LQR gain.",
    )
    control_down.add_argument(
        "--invert-encoder-angle",
        action="store_true",
        help="Flip the sign of theta and theta-dot before applying the LQR gain.",
    )
    control_down.add_argument("--no-zero-on-start", action="store_true", help="Keep the current encoder zero instead of zeroing before arming.")
    control_down.add_argument("--home-speed-mm-s", type=float, default=50.0)
    control_down.add_argument("--middle-speed-mm-s", type=float, default=50.0)
    control_down.add_argument("--no-home", action="store_true", help="Skip homing left before arming control.")
    control_down.add_argument("--no-middle", action="store_true", help="Skip moving to the configured midpoint before arming control.")

    subparsers.add_parser("shell", help="Start an interactive motion shell without reopening serial ports between commands.")

    stop = subparsers.add_parser("stop", help="Send an immediate actuator stop command.")
    stop.set_defaults(command="stop")

    return parser


def start_worker(stack: ExitStack, name: str, worker: SerialWorker | None) -> SerialWorker | None:
    if worker is None:
        print(f"{name}: no port configured")
        return None

    print(f"{name}: opening {worker.port} at {worker.baudrate} baud")
    worker.start()
    stack.callback(worker.stop)
    return worker


def print_snapshot(name: str, worker: SerialWorker | None) -> None:
    if worker is None:
        return

    latest = worker.get_latest()
    if latest is None:
        print(f"{name}: no data yet")
    else:
        print(f"{name}: {latest.data}")

    error = worker.get_last_error()
    if error:
        print(f"{name} error: {error}")


def print_motion_status(actuator: ActuatorController | None, limits: LimitSensorReader | None, motion: MotionController | None) -> None:
    print_snapshot("actuator", actuator)
    print_snapshot("limits", limits)

    latest = actuator.get_latest() if actuator is not None else None
    if latest is not None and motion is not None:
        position_steps = int(latest.data["position_steps"])
        print(f"position_mm: {motion.steps_to_mm(position_steps):.3f}")


def run_motion_shell(actuator: ActuatorController, limits: LimitSensorReader, motion: MotionController) -> int:
    print("Interactive motion shell. Commands: status, home left|right, middle, move <mm>, steps <steps>, goto <steps>, speed <mm/s> <s>, stop, quit")

    while True:
        try:
            command_line = input("motion> ").strip()
        except EOFError:
            command_line = "quit"
        except KeyboardInterrupt:
            print()
            actuator.stop_motion()
            continue

        if not command_line:
            continue

        parts = command_line.split()
        command = parts[0].lower()

        try:
            if command in {"quit", "exit"}:
                actuator.stop_motion()
                return 0

            if command == "status":
                actuator.request_status()
                limits.request_state()
                time.sleep(0.6)
                print_motion_status(actuator, limits, motion)
                continue

            if command == "stop":
                actuator.stop_motion()
                continue

            if command == "home":
                side = parts[1] if len(parts) >= 2 else "left"
                speed = float(parts[2]) if len(parts) >= 3 else motion.config.default_speed_mm_s
                motion.home(side=side, speed_mm_s=speed)
                print("home complete")
                continue

            if command == "move":
                if len(parts) < 2:
                    print("usage: move <target_mm> [speed_mm_s]")
                    continue
                target_mm = float(parts[1])
                speed = float(parts[2]) if len(parts) >= 3 else motion.config.default_speed_mm_s
                motion.move_to_mm(target_mm, speed_mm_s=speed)
                print("move complete")
                continue

            if command in {"middle", "center"}:
                speed = float(parts[1]) if len(parts) >= 2 else motion.config.default_speed_mm_s
                motion.move_to_middle(speed_mm_s=speed)
                print("middle move complete")
                continue

            if command == "steps":
                if len(parts) < 2:
                    print("usage: steps <relative_steps> [steps_per_second]")
                    continue
                steps = int(parts[1])
                steps_per_second = float(parts[2]) if len(parts) >= 3 else 100.0
                motion.move_relative_steps(steps, steps_per_second)
                print("step move complete")
                continue

            if command in {"goto", "goto-steps"}:
                if len(parts) < 2:
                    print("usage: goto <target_steps> [steps_per_second]")
                    continue
                target_steps = int(parts[1])
                steps_per_second = float(parts[2]) if len(parts) >= 3 else 100.0
                motion.move_to_steps(target_steps, steps_per_second)
                print("absolute step move complete")
                continue

            if command == "speed":
                if len(parts) < 3:
                    print("usage: speed <speed_mm_s> <duration_s>")
                    continue
                speed = float(parts[1])
                duration = float(parts[2])
                motion.run_speed(speed, duration_s=duration)
                print("speed command complete")
                continue

            print(f"unknown command: {command}")

        except (MotionSafetyError, ValueError) as exc:
            print(f"Motion command failed: {exc}")


def main() -> int:
    args = build_parser().parse_args()
    command = args.command or "monitor"

    actuator = ActuatorController(args.actuator_port, args.baudrate) if args.actuator_port else None
    limits = LimitSensorReader(args.limits_port, args.baudrate) if args.limits_port else None
    encoder = EncoderReader(args.encoder_port, args.baudrate) if args.encoder_port else None

    if actuator is None and limits is None and encoder is None:
        print("No Arduino ports were provided. Use --actuator-port, --limits-port, and/or --encoder-port.")
        return 2

    with ExitStack() as stack:
        actuator = start_worker(stack, "actuator", actuator)
        limits = start_worker(stack, "limits", limits)
        encoder = start_worker(stack, "encoder", encoder)

        time.sleep(2.0)

        if actuator is not None:
            actuator.request_status()
        if limits is not None:
            limits.request_state()
        if encoder is not None and args.zero_encoder_on_start:
            encoder.zero_current_position()

        motion_config = MotionConfig(steps_per_mm=args.steps_per_mm, travel_mm=args.travel_mm)
        motion = MotionController(actuator, limits, motion_config) if actuator is not None and limits is not None else None

        if command != "monitor":
            if command == "stop":
                if actuator is None:
                    print("--actuator-port is required for stop")
                    return 2
                actuator.stop_motion()
                return 0

            if command == "capture-encoder":
                if encoder is None:
                    print("--encoder-port is required for capture-encoder")
                    return 2
                run_encoder_capture(encoder, output_path=args.output, zero_on_start=not args.no_zero_on_start)
                return 0

            if command == "control-down":
                if actuator is None or limits is None or encoder is None:
                    print("--actuator-port, --limits-port, and --encoder-port are required for control-down")
                    return 2
                control_config = DownwardControlConfig(
                    period_s=args.period_s,
                    lqr_x_gain=args.x_gain,
                    lqr_x_dot_gain=args.x_dot_gain,
                    lqr_theta_gain=args.theta_gain,
                    lqr_theta_dot_gain=args.omega_gain,
                    center_gain_per_s=args.center_gain,
                    estimator=args.estimator,
                    theta_alpha=args.theta_alpha,
                    theta_beta=args.theta_beta,
                    max_theta_residual_rad=args.max_theta_residual_rad,
                    armed_theta_alpha=args.armed_theta_alpha,
                    armed_theta_beta=args.armed_theta_beta,
                    armed_max_theta_residual_rad=args.armed_max_theta_residual_rad,
                    theta_filter_alpha=args.theta_filter_alpha,
                    theta_dot_filter_alpha=args.theta_dot_filter_alpha,
                    max_theta_rate_rad_s=args.max_theta_rate_rad_s,
                    theta_deadband_rad=args.theta_deadband_rad,
                    theta_dot_deadband_rad_s=args.omega_deadband_rad_s,
                    acceleration_deadband_m_s2=args.accel_deadband_m_s2,
                    control_trigger_theta_rad=args.control_trigger_theta_rad,
                    control_trigger_theta_dot_rad_s=args.control_trigger_omega_rad_s,
                    control_trigger_samples=args.control_trigger_samples,
                    settle_theta_rad=args.settle_theta_rad,
                    settle_theta_dot_rad_s=args.settle_omega_rad_s,
                    settle_samples=args.settle_samples,
                    velocity_leak_per_s=args.velocity_leak_per_s,
                    max_acceleration_m_s2=args.max_accel_m_s2,
                    max_speed_mm_s=args.max_speed_mm_s,
                    max_abs_theta_rad=args.max_angle_rad,
                    actuator_command_sign=-1.0 if args.invert_control or args.invert_actuator_command else 1.0,
                    cart_position_sign=-1.0 if args.invert_cart_position else 1.0,
                    encoder_angle_sign=-1.0 if args.invert_encoder_angle else 1.0,
                    zero_encoder_on_start=not args.no_zero_on_start,
                    home_before_start=not args.no_home,
                    move_middle_before_start=not args.no_middle,
                    home_speed_mm_s=args.home_speed_mm_s,
                    middle_speed_mm_s=args.middle_speed_mm_s,
                )
                run_downward_control(
                    actuator,
                    limits,
                    encoder,
                    motion,
                    motion_config=motion_config,
                    control_config=control_config,
                    output_path=args.output,
                )
                return 0

            if motion is None:
                print("--actuator-port and --limits-port are required for motion commands")
                return 2

            if command == "shell":
                return run_motion_shell(actuator, limits, motion)

            try:
                if command == "home":
                    motion.home(side=args.side, speed_mm_s=args.speed_mm_s)
                elif command == "middle":
                    motion.move_to_middle(speed_mm_s=args.speed_mm_s)
                elif command == "move-mm":
                    motion.move_to_mm(args.target_mm, speed_mm_s=args.speed_mm_s)
                elif command == "move-steps":
                    motion.move_relative_steps(args.steps, args.steps_per_second)
                elif command == "goto-steps":
                    motion.move_to_steps(args.target_steps, args.steps_per_second)
                elif command == "calibrate-travel":
                    run_travel_calibration(
                        actuator,
                        limits,
                        motion,
                        output_path=args.output,
                        start_steps=args.start_steps,
                        speeds_steps_s=tuple(args.speeds) if args.speeds is not None else None,
                        relative_steps=tuple(args.moves),
                        home_speed_mm_s=args.home_speed_mm_s,
                        reposition_speed_steps_s=args.reposition_speed_steps_s,
                    )
                elif command == "speed":
                    motion.run_speed(args.speed_mm_s, duration_s=args.duration)
                else:
                    raise RuntimeError(f"unknown command {command}")
            except (MotionSafetyError, ValueError) as exc:
                print(f"Motion command failed: {exc}")
                return 1

            return 0

        start_time = time.monotonic()
        next_print = start_time

        try:
            while True:
                now = time.monotonic()
                if args.duration is not None and (now - start_time) >= args.duration:
                    break

                if now >= next_print:
                    print_snapshot("actuator", actuator)
                    print_snapshot("limits", limits)
                    print_snapshot("encoder", encoder)
                    print("-" * 72)
                    next_print = now + args.poll_period

                latest_limits = limits.get_latest() if limits is not None else None
                if actuator is not None and latest_limits is not None and latest_limits.data.get("any_limit"):
                    actuator.stop_motion()

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping host monitor.")

        if actuator is not None:
            actuator.stop_motion()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
