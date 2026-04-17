from __future__ import annotations

import argparse
import time
from contextlib import ExitStack

from .arduino_actuator import ActuatorController
from .arduino_encoder import EncoderReader
from .arduino_limits import LimitSensorReader
from .serial_worker import SerialWorker


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Initial host monitor for the three-Arduino pendulum setup.")
    parser.add_argument("--actuator-port", help="Serial port for Arduino #1 actuator controller, e.g. COM3.")
    parser.add_argument("--limits-port", help="Serial port for Arduino #2 limit sensor reader, e.g. COM4.")
    parser.add_argument("--encoder-port", help="Serial port for Arduino #3 AS5600 encoder reader, e.g. COM5.")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--poll-period", type=float, default=0.5, help="Seconds between printed snapshots.")
    parser.add_argument("--duration", type=float, help="Optional run duration in seconds.")
    parser.add_argument("--zero-encoder-on-start", action="store_true")
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


def main() -> int:
    args = build_parser().parse_args()

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
