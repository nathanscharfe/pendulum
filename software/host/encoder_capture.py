from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

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
    "unwrapped_count",
    "theta_deg",
    "theta_rad",
    "omega_rad_s",
]


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware") / "magnetic encoder" / "captures" / f"encoder_capture_{timestamp}.csv"


def run_encoder_capture(encoder: EncoderReader, output_path: Path | None = None, zero_on_start: bool = True) -> Path:
    output_path = output_path or default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Pendulum encoder capture")
    print(f"Output file: {output_path}")
    print("Start with the pendulum hanging downward and motionless.")
    if zero_on_start:
        print("The current encoder angle will be zeroed immediately before recording.")
    else:
        print("The encoder will keep its current zero.")

    input("Press Enter when you are ready to start recording...")

    if zero_on_start:
        encoder.zero_current_position()
        sleep(0.2)

    encoder.clear_snapshots()
    start_monotonic_s = monotonic()
    print("Recording. Displace and release the pendulum, then let it swing to rest.")

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


def write_snapshots(output_path: Path, snapshots: list[SerialSnapshot]) -> None:
    fieldnames = ["host_timestamp_s", *ENCODER_FIELDS]

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for snapshot in snapshots:
            row = {"host_timestamp_s": f"{snapshot.timestamp_s:.6f}"}
            row.update({field: snapshot.data[field] for field in ENCODER_FIELDS})
            writer.writerow(row)
