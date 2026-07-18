"""Asynchronous evidence journal for supervised live Follow policy decisions."""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Sequence


FOLLOW_AUDIT_SCHEMA = "live_follow_canary_audit_v1"
_STOP = object()


class FollowAuditUnavailable(RuntimeError):
    """The supervised Follow evidence stream can no longer be trusted."""


def _finite_vector(name: str, values: Sequence[float], length: int) -> list[float]:
    result = list(values)
    if len(result) != length or not all(
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(value)
        for value in result
    ):
        label = "38 finite" if length == 38 else "four finite"
        raise ValueError(f"{name} must contain {label} numbers")
    return [float(value) for value in result]


def build_follow_audit_record(
    *,
    sequence: int,
    trajectory_id: str,
    state_seq: int,
    state_stamp_ms: int,
    elapsed_ms: float,
    observation: Sequence[float],
    raw_normalized: Sequence[float],
    applied_normalized: Sequence[float],
    physical_action: Sequence[float],
) -> dict[str, Any]:
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("sequence must be a non-negative integer")
    if not isinstance(trajectory_id, str) or not trajectory_id:
        raise ValueError("trajectory_id must be non-empty")
    if not math.isfinite(elapsed_ms) or elapsed_ms < 0.0:
        raise ValueError("elapsed_ms must be finite and non-negative")
    return {
        "schema": FOLLOW_AUDIT_SCHEMA,
        "recorded_at_pc_ms": time.time_ns() // 1_000_000,
        "sequence": sequence,
        "trajectory_id": trajectory_id,
        "state_seq": int(state_seq),
        "state_stamp_ms": int(state_stamp_ms),
        "elapsed_ms": float(elapsed_ms),
        "observation_schema": "scale_excavator_v2_38d",
        "observation": _finite_vector("observation", observation, 38),
        "raw_normalized": _finite_vector("raw_normalized", raw_normalized, 4),
        "applied_normalized": _finite_vector(
            "applied_normalized", applied_normalized, 4
        ),
        "physical_action": _finite_vector("physical_action", physical_action, 4),
    }


class FollowAuditWriter:
    """Write every canary decision to JSONL and atomically refresh a latest snapshot."""

    def __init__(self, journal_directory: Path, latest_snapshot: Path) -> None:
        self._directory = Path(journal_directory)
        self._latest = Path(latest_snapshot)
        self._directory.mkdir(parents=True, exist_ok=True)
        self._latest.parent.mkdir(parents=True, exist_ok=True)
        started_at = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
        self.journal_path = self._directory / (
            f"follow_canary.{started_at}.{os.getpid()}.jsonl"
        )
        self._queue: queue.Queue[bytes | object] = queue.Queue(maxsize=2048)
        self._failure = threading.Event()
        self._failure_message = ""
        self._closed = False
        self._thread = threading.Thread(
            target=self._write_loop,
            name="follow_canary_audit",
            daemon=True,
        )
        self._thread.start()

    @property
    def is_healthy(self) -> bool:
        return not self._failure.is_set()

    def submit(self, record: dict[str, Any]) -> None:
        if self._closed:
            raise FollowAuditUnavailable("Follow audit writer is closed")
        if self._failure.is_set():
            raise FollowAuditUnavailable(self._failure_message)
        payload = (json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
        try:
            self._queue.put_nowait(payload)
        except queue.Full as exc:
            self._set_failure("Follow audit queue is full")
            raise FollowAuditUnavailable(self._failure_message) from exc

    def close(self) -> None:
        if self._closed:
            if self._failure.is_set():
                raise FollowAuditUnavailable(self._failure_message)
            return
        self._closed = True
        self._queue.put(_STOP)
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            self._set_failure("Follow audit writer did not stop")
        if self._failure.is_set():
            raise FollowAuditUnavailable(self._failure_message)

    def _write_loop(self) -> None:
        wrote_record = False
        try:
            with self.journal_path.open("ab", buffering=0) as journal:
                while True:
                    item = self._queue.get()
                    if item is _STOP:
                        break
                    journal.write(item)
                    latest_next = self._latest.with_name(self._latest.name + ".next")
                    latest_next.write_bytes(item)
                    os.replace(latest_next, self._latest)
                    wrote_record = True
        except OSError as exc:
            self._set_failure(f"Follow audit write failed: {exc}")
        finally:
            if not wrote_record:
                try:
                    self.journal_path.unlink(missing_ok=True)
                except OSError as exc:
                    self._set_failure(f"Follow audit cleanup failed: {exc}")

    def _set_failure(self, message: str) -> None:
        self._failure_message = message
        self._failure.set()
