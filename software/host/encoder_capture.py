from __future__ import annotations

import csv
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep
from typing import Callable

from .arduino_encoder import EncoderReader
from .serial_worker import SerialSnapshot


ENCODER_FIELDS = [
    "time_ms",
    "connected",
    "magnet_detected",
    "magnet_too_weak",
    "magnet_too_strong",
    "agc",
    "magnitude",
    "raw_count",
    "filtered_count",
    "unwrapped_count",
    "theta_deg",
    "theta_rad",
    "omega_rad_s",
]


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware") / "magnetic encoder" / "captures" / f"encoder_capture_{timestamp}.csv"


def default_period_test_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware") / "magnetic encoder" / "captures" / f"pendulum 1_period_test_{timestamp}.csv"


@dataclass(frozen=True)
class CaptureSessionConfig:
    title: str
    ready_message: str
    zero_message: str
    keep_zero_message: str
    recording_message: str
    output_path_factory: Callable[[], Path]


GENERIC_CAPTURE_CONFIG = CaptureSessionConfig(
    title="Pendulum encoder capture",
    ready_message="Start with the pendulum hanging downward and motionless.",
    zero_message="The current encoder angle will be zeroed immediately before recording.",
    keep_zero_message="The encoder will keep its current zero.",
    recording_message="Recording. Displace and release the pendulum, then let it swing to rest.",
    output_path_factory=default_output_path,
)


PERIOD_TEST_CAPTURE_CONFIG = CaptureSessionConfig(
    title="Pendulum period test capture",
    ready_message="Start with the pendulum hanging downward and motionless at its small-angle equilibrium.",
    zero_message="The current encoder angle will be zeroed immediately before recording so the resting position is near zero.",
    keep_zero_message="The encoder will keep its current zero.",
    recording_message="Recording. Displace the pendulum by a small angle, release it, and let it swing freely for several cycles before stopping.",
    output_path_factory=default_period_test_output_path,
)


def run_encoder_capture(
    encoder: EncoderReader,
    output_path: Path | None = None,
    zero_on_start: bool = True,
    session: CaptureSessionConfig = GENERIC_CAPTURE_CONFIG,
) -> Path:
    output_path = output_path or session.output_path_factory()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(session.title)
    print(f"Output file: {output_path}")
    print(session.ready_message)
    if zero_on_start:
        print(session.zero_message)
    else:
        print(session.keep_zero_message)

    input("Press Enter when you are ready to start recording...")

    if zero_on_start:
        encoder.zero_current_position()
        sleep(0.2)

    encoder.clear_snapshots()
    start_monotonic_s = monotonic()
    print(session.recording_message)

    try:
        input("Press Enter to stop recording...")
    except KeyboardInterrupt:
        print("\nCapture interrupted. Writing samples collected so far.")

    stop_monotonic_s = monotonic()
    snapshots = encoder.drain_snapshots()
    write_snapshots(output_path, snapshots)

    duration_s = max(0.0, stop_monotonic_s - start_monotonic_s)
    sample_rate = len(snapshots) / duration_s if duration_s > 0.0 else 0.0
    print(f"Wrote {len(snapshots)} encoder rows to {output_path}")
    print(f"Capture duration: {duration_s:.3f} s")
    print(f"Average captured sample rate: {sample_rate:.2f} Hz")

    return output_path


def run_period_test_capture(encoder: EncoderReader, output_path: Path | None = None, zero_on_start: bool = True) -> Path:
    return run_encoder_capture(
        encoder,
        output_path=output_path,
        zero_on_start=zero_on_start,
        session=PERIOD_TEST_CAPTURE_CONFIG,
    )


def write_snapshots(output_path: Path, snapshots: list[SerialSnapshot]) -> None:
    fieldnames = ["host_timestamp_s", *ENCODER_FIELDS]

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for snapshot in snapshots:
            row = {"host_timestamp_s": f"{snapshot.timestamp_s:.6f}"}
            row.update({field: snapshot.data[field] for field in ENCODER_FIELDS})
            writer.writerow(row)
