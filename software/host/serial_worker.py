from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from time import time
from typing import Any


@dataclass(frozen=True)
class SerialSnapshot:
    timestamp_s: float
    raw_line: str
    data: dict[str, Any]


class SerialWorker:
    """Background serial reader that owns one Arduino serial port."""

    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 1.0) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s

        self._serial = None
        self._thread: Thread | None = None
        self._stop_event = Event()
        self._lock = Lock()
        self._write_lock = Lock()

        self._latest: SerialSnapshot | None = None
        self._last_error: str | None = None
        self._last_comment: str | None = None
        self._last_ack: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._open_serial()
        self._thread = Thread(target=self._read_loop, name=f"{type(self).__name__}:{self.port}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def get_latest(self) -> SerialSnapshot | None:
        with self._lock:
            return self._latest

    def get_last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    def get_last_comment(self) -> str | None:
        with self._lock:
            return self._last_comment

    def get_last_ack(self) -> str | None:
        with self._lock:
            return self._last_ack

    def write_command(self, command: str) -> None:
        if self._serial is None:
            raise RuntimeError(f"{self.port} is not open")

        line = command.rstrip("\r\n") + "\n"
        with self._write_lock:
            self._serial.write(line.encode("ascii"))
            self._serial.flush()

    def _open_serial(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required. Install with: python -m pip install -r software/host/requirements.txt") from exc

        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)

    def _read_loop(self) -> None:
        assert self._serial is not None

        while not self._stop_event.is_set():
            try:
                raw_bytes = self._serial.readline()
            except Exception as exc:  # Serial errors should not silently kill the host.
                with self._lock:
                    self._last_error = str(exc)
                continue

            if not raw_bytes:
                continue

            line = raw_bytes.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        if line.startswith("ack,"):
            with self._lock:
                self._last_ack = line
            return

        if line.startswith("error,"):
            with self._lock:
                self._last_error = line
            return

        parsed = self.parse_line(line)
        if parsed is not None:
            snapshot = SerialSnapshot(timestamp_s=time(), raw_line=line, data=parsed)
            with self._lock:
                self._latest = snapshot
            return

        if line.startswith("#"):
            with self._lock:
                self._last_comment = line

    def parse_line(self, line: str) -> dict[str, Any] | None:
        raise NotImplementedError


def parse_bool_int(value: str) -> bool:
    return int(value.strip()) != 0
