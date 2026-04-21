from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from time import time
from tkinter import BOTH, LEFT, RIGHT, Button, Canvas, Frame, Label, StringVar, Tk, X

from .arduino_encoder import EncoderReader
from .serial_worker import SerialSnapshot


@dataclass(frozen=True)
class EncoderUiConfig:
    poll_period_ms: int = 10
    history_window_s: float = 10.0
    canvas_width: int = 760
    canvas_height: int = 320
    padding_px: int = 28
    theta_margin_deg: float = 2.0


class EncoderUiApp:
    def __init__(self, encoder: EncoderReader, config: EncoderUiConfig) -> None:
        self.encoder = encoder
        self.config = config
        self._history: deque[tuple[float, float, float]] = deque()
        self._latest_filtered_theta_deg = 0.0
        self._latest_filtered_theta_rad = 0.0
        self._latest_raw_theta_deg = 0.0
        self._latest_raw_theta_rad = 0.0
        self._latest_sample_time_s = 0.0

        self.root = Tk()
        self.root.title("Pendulum Encoder Test")
        self.root.geometry("860x470")
        self.root.minsize(640, 360)
        self.root.protocol("WM_DELETE_WINDOW", self._handle_close)

        controls = Frame(self.root)
        controls.pack(fill=X, padx=12, pady=(12, 6))

        self.filtered_theta_text = StringVar(value="filtered: waiting for data")
        self.raw_theta_text = StringVar(value="raw: waiting for data")
        self.status_text = StringVar(value="status: connecting")
        self.magnitude_text = StringVar(value="magnitude: --")
        self.agc_text = StringVar(value="agc: --")

        theta_block = Frame(controls)
        theta_block.pack(side=LEFT)
        Label(theta_block, textvariable=self.filtered_theta_text, font=("Segoe UI", 15, "bold")).pack(anchor="w")
        Label(theta_block, textvariable=self.raw_theta_text, font=("Segoe UI", 11)).pack(anchor="w")
        Button(controls, text="Zero", command=self._zero_encoder, width=10).pack(side=RIGHT)

        indicators_row = Frame(self.root)
        indicators_row.pack(fill=X, padx=12, pady=(0, 6))
        self.good_indicator = self._make_indicator(indicators_row, "Magnet OK", "#b0b0b0")
        self.detected_indicator = self._make_indicator(indicators_row, "Detected", "#b0b0b0")
        self.weak_indicator = self._make_indicator(indicators_row, "Too Weak", "#b0b0b0")
        self.strong_indicator = self._make_indicator(indicators_row, "Too Strong", "#b0b0b0")
        Label(indicators_row, textvariable=self.magnitude_text, width=18, anchor="w").pack(side=LEFT, padx=(16, 0))
        Label(indicators_row, textvariable=self.agc_text, width=12, anchor="w").pack(side=LEFT, padx=(10, 0))

        status_row = Frame(self.root)
        status_row.pack(fill=X, padx=12, pady=(0, 6))
        Label(status_row, textvariable=self.status_text, anchor="w").pack(fill=X)

        self.canvas = Canvas(
            self.root,
            width=self.config.canvas_width,
            height=self.config.canvas_height,
            background="white",
            highlightthickness=1,
            highlightbackground="#c7c7c7",
        )
        self.canvas.pack(fill=BOTH, expand=True, padx=12, pady=(0, 12))

    def run(self) -> None:
        self.encoder.clear_snapshots()
        self._schedule_update()
        self.root.mainloop()

    def _schedule_update(self) -> None:
        self._update_ui()
        self.root.after(self.config.poll_period_ms, self._schedule_update)

    def _update_ui(self) -> None:
        snapshots = self.encoder.drain_snapshots()
        if snapshots:
            self._append_snapshots(snapshots)

        latest = self.encoder.get_latest()
        if latest is None:
            self.filtered_theta_text.set("filtered: waiting for data")
            self.raw_theta_text.set("raw: waiting for data")
        else:
            self._latest_filtered_theta_deg = float(latest.data["theta_deg"])
            self._latest_filtered_theta_rad = float(latest.data["theta_rad"])
            filtered_count = int(latest.data.get("filtered_count", latest.data["raw_count"]))
            raw_count = int(latest.data["raw_count"])
            raw_minus_filtered_count = shortest_count_delta(raw_count, filtered_count)
            self._latest_raw_theta_deg = count_delta_to_degrees(raw_minus_filtered_count) + self._latest_filtered_theta_deg
            self._latest_raw_theta_rad = count_delta_to_radians(raw_minus_filtered_count) + self._latest_filtered_theta_rad
            self._latest_sample_time_s = latest.timestamp_s
            self.filtered_theta_text.set(
                f"filtered: {self._latest_filtered_theta_deg:+.3f} deg   ({self._latest_filtered_theta_rad:+.5f} rad)"
            )
            self.raw_theta_text.set(
                f"raw: {self._latest_raw_theta_deg:+.3f} deg   ({self._latest_raw_theta_rad:+.5f} rad)"
            )

        self._update_status(latest)
        self._draw_plot()

    def _append_snapshots(self, snapshots: list[SerialSnapshot]) -> None:
        for snapshot in snapshots:
            filtered_theta_deg = float(snapshot.data["theta_deg"])
            filtered_count = int(snapshot.data.get("filtered_count", snapshot.data["raw_count"]))
            raw_count = int(snapshot.data["raw_count"])
            raw_minus_filtered_count = shortest_count_delta(raw_count, filtered_count)
            raw_theta_deg = count_delta_to_degrees(raw_minus_filtered_count) + filtered_theta_deg
            self._history.append((snapshot.timestamp_s, raw_theta_deg, filtered_theta_deg))

        newest_time_s = self._history[-1][0]
        oldest_allowed_s = newest_time_s - self.config.history_window_s
        while self._history and self._history[0][0] < oldest_allowed_s:
            self._history.popleft()

    def _update_status(self, latest: SerialSnapshot | None) -> None:
        if latest is None:
            self._set_indicator(self.good_indicator, False, "#b0b0b0")
            self._set_indicator(self.detected_indicator, False, "#b0b0b0")
            self._set_indicator(self.weak_indicator, False, "#b0b0b0")
            self._set_indicator(self.strong_indicator, False, "#b0b0b0")
            self.magnitude_text.set("magnitude: --")
            self.agc_text.set("agc: --")
            self.status_text.set("status: waiting for first encoder sample")
            return

        age_ms = max(0.0, (time() - latest.timestamp_s) * 1000.0)
        connected = bool(latest.data.get("connected"))
        magnet_detected = bool(latest.data.get("magnet_detected"))
        magnet_too_weak = bool(latest.data.get("magnet_too_weak"))
        magnet_too_strong = bool(latest.data.get("magnet_too_strong"))
        magnet_good = magnet_detected and not magnet_too_weak and not magnet_too_strong
        magnitude = latest.data.get("magnitude", "--")
        agc = latest.data.get("agc", "--")
        self._set_indicator(self.good_indicator, magnet_good, "#2da44e")
        self._set_indicator(self.detected_indicator, magnet_detected, "#2da44e")
        self._set_indicator(self.weak_indicator, magnet_too_weak, "#d1242f")
        self._set_indicator(self.strong_indicator, magnet_too_strong, "#d1242f")
        self.magnitude_text.set(f"magnitude: {magnitude}")
        self.agc_text.set(f"agc: {agc}")
        self.status_text.set(
            "status: "
            f"connected={connected}  "
            f"magnet_detected={magnet_detected}  "
            f"weak={magnet_too_weak}  "
            f"strong={magnet_too_strong}  "
            f"sample_age_ms={age_ms:.1f}"
        )

    def _draw_plot(self) -> None:
        width = max(self.canvas.winfo_width(), self.config.canvas_width)
        height = max(self.canvas.winfo_height(), self.config.canvas_height)
        left = self.config.padding_px
        top = self.config.padding_px
        right = width - self.config.padding_px
        bottom = height - self.config.padding_px

        self.canvas.delete("all")
        self.canvas.create_rectangle(left, top, right, bottom, outline="#d0d0d0")

        if not self._history:
            self.canvas.create_text(
                width / 2,
                height / 2,
                text="Waiting for encoder samples...",
                fill="#666666",
                font=("Segoe UI", 12),
            )
            return

        times_s = [time_s for time_s, _, _ in self._history]
        newest_time_s = times_s[-1]
        oldest_time_s = max(times_s[0], newest_time_s - self.config.history_window_s)
        visible_points = [
            (time_s, raw_theta_deg, filtered_theta_deg)
            for time_s, raw_theta_deg, filtered_theta_deg in self._history
            if time_s >= oldest_time_s
        ]

        theta_min_deg = min(min(raw_theta_deg, filtered_theta_deg) for _, raw_theta_deg, filtered_theta_deg in visible_points)
        theta_max_deg = max(max(raw_theta_deg, filtered_theta_deg) for _, raw_theta_deg, filtered_theta_deg in visible_points)
        if theta_min_deg == theta_max_deg:
            theta_min_deg -= 1.0
            theta_max_deg += 1.0

        theta_min_deg -= self.config.theta_margin_deg
        theta_max_deg += self.config.theta_margin_deg
        theta_span_deg = max(theta_max_deg - theta_min_deg, 1e-6)
        time_span_s = max(newest_time_s - oldest_time_s, 1e-6)

        if theta_min_deg <= 0.0 <= theta_max_deg:
            zero_y = bottom - ((0.0 - theta_min_deg) / theta_span_deg) * (bottom - top)
            self.canvas.create_line(left, zero_y, right, zero_y, fill="#e4e4e4", dash=(4, 3))
            self.canvas.create_text(left + 24, zero_y - 10, text="0 deg", fill="#777777", font=("Segoe UI", 9))

        raw_points: list[float] = []
        filtered_points: list[float] = []
        for time_s, raw_theta_deg, filtered_theta_deg in visible_points:
            x = left + ((time_s - oldest_time_s) / time_span_s) * (right - left)
            raw_y = bottom - ((raw_theta_deg - theta_min_deg) / theta_span_deg) * (bottom - top)
            filtered_y = bottom - ((filtered_theta_deg - theta_min_deg) / theta_span_deg) * (bottom - top)
            raw_points.extend((x, raw_y))
            filtered_points.extend((x, filtered_y))

        if len(raw_points) >= 4:
            self.canvas.create_line(*raw_points, fill="#a0a0a0", width=1, smooth=False)
        if len(filtered_points) >= 4:
            self.canvas.create_line(*filtered_points, fill="#0b6efd", width=2, smooth=False)

        self.canvas.create_text(left, top - 12, text=f"{theta_max_deg:.1f} deg", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_text(left, bottom + 12, text=f"{theta_min_deg:.1f} deg", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_text(right, bottom + 12, text=f"{self.config.history_window_s:.1f} s window", anchor="e", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_line(right - 170, top + 12, right - 146, top + 12, fill="#0b6efd", width=2)
        self.canvas.create_text(right - 142, top + 12, text="filtered", anchor="w", fill="#555555", font=("Segoe UI", 9))
        self.canvas.create_line(right - 170, top + 30, right - 146, top + 30, fill="#a0a0a0", width=1)
        self.canvas.create_text(right - 142, top + 30, text="raw", anchor="w", fill="#555555", font=("Segoe UI", 9))

    def _zero_encoder(self) -> None:
        self.encoder.zero_current_position()
        self.encoder.clear_snapshots()
        self._history.clear()
        self._latest_filtered_theta_deg = 0.0
        self._latest_filtered_theta_rad = 0.0
        self._latest_raw_theta_deg = 0.0
        self._latest_raw_theta_rad = 0.0
        self.filtered_theta_text.set("filtered: zeroing...")
        self.raw_theta_text.set("raw: zeroing...")
        self.status_text.set("status: zero command sent")

    def _handle_close(self) -> None:
        self.root.quit()
        self.root.destroy()

    def _make_indicator(self, parent: Frame, text: str, color: str) -> Label:
        label = Label(
            parent,
            text=text,
            width=11,
            relief="solid",
            borderwidth=1,
            bg=color,
            fg="white",
            font=("Segoe UI", 9, "bold"),
        )
        label.pack(side=LEFT, padx=(0, 8))
        return label

    def _set_indicator(self, label: Label, active: bool, active_color: str) -> None:
        if active:
            label.configure(bg=active_color, fg="white")
        else:
            label.configure(bg="#b0b0b0", fg="white")


def run_encoder_ui(encoder: EncoderReader, config: EncoderUiConfig | None = None) -> None:
    app = EncoderUiApp(encoder=encoder, config=config or EncoderUiConfig())
    app.run()


def count_delta_to_degrees(count_delta: int) -> float:
    return count_delta * (360.0 / 4096.0)


def count_delta_to_radians(count_delta: int) -> float:
    return count_delta * (6.28318530718 / 4096.0)


def shortest_count_delta(current_count: int, reference_count: int) -> int:
    delta = current_count - reference_count
    if delta > 2048:
        delta -= 4096
    elif delta < -2048:
        delta += 4096
    return delta
