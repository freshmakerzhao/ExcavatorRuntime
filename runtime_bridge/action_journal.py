"""PC→Orin UDP动作发送边界及本地可回放审计记录。"""

from __future__ import annotations

import base64
import fcntl
import hashlib
import json
import os
import queue
import re
import socket
import sys
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .runtime_config import ActionJournalConfig


ACTION_SEND_SCHEMA = "pc_orin_action_send_v1"
_SOURCE_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_STOP = object()


class ActionJournalUnavailable(RuntimeError):
    """发送审计不可用；调用方必须停止继续发送动作。"""


class RecordedUdpSender:
    """先发送UDP，再异步追加精确payload和可读packet；磁盘I/O不阻塞控制循环。"""

    def __init__(
        self,
        udp_socket: socket.socket,
        destination: tuple[str, int],
        *,
        journal_config: ActionJournalConfig,
        source: str,
    ) -> None:
        if _SOURCE_PATTERN.fullmatch(source) is None:
            raise ValueError(f"source必须是安全的snake_case标识，实际为 {source!r}")
        self._socket = udp_socket
        self._destination = destination
        self._source = source
        self._queue: queue.Queue[bytes | object] = queue.Queue(maxsize=8192)
        self._send_lock = threading.Lock()
        self._writer_failed = threading.Event()
        self._failure_message: str | None = None
        self._closed = False
        self._journal_dir = Path(journal_config.directory)
        self._max_file_bytes = journal_config.max_file_bytes
        self._retained_files = journal_config.retained_files
        self._journal_dir.mkdir(parents=True, exist_ok=True)
        started_at = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
        self._session_name = f"{source}.{started_at}.{os.getpid()}"
        self._part_index = 0
        self.journal_path = self._part_path(self._part_index)
        self._journal = self._open_locked_part(self.journal_path)
        self._current_size = self.journal_path.stat().st_size
        self._part_has_records = self._current_size > 0
        self._writer = threading.Thread(
            target=self._write_loop,
            name=f"{source}_action_journal",
            daemon=True,
        )
        self._writer.start()

    @property
    def is_healthy(self) -> bool:
        """供运行监控查询writer状态；关闭不等同于写盘故障。"""
        return not self._writer_failed.is_set()

    def send(self, payload: bytes) -> int:
        """发送一个完整UDP datagram；仅成功发送后生成审计记录。"""
        with self._send_lock:
            self._require_available()
            if self._queue.full():
                raise ActionJournalUnavailable(
                    f"action journal queue full; refuse unsupervised send: {self.journal_path}"
                )
            sent_bytes = self._socket.sendto(payload, self._destination)
            if sent_bytes != len(payload):
                raise OSError(f"UDP发送长度异常: sent={sent_bytes}, expected={len(payload)}")
            self._queue.put_nowait(self._build_record(payload, sent_bytes))
            return sent_bytes

    def close(self) -> None:
        """排空已排队记录并结束写线程；可重复调用。"""
        with self._send_lock:
            if self._closed:
                self._raise_if_failed()
                return
            self._closed = True
            try:
                self._queue.put(_STOP, timeout=5.0)
            except queue.Full:
                self._set_failure("action journal关闭时队列无法排空")
        self._writer.join(timeout=5.0)
        if self._writer.is_alive():
            self._set_failure("action journal写线程未能在5秒内关闭")
        self._raise_if_failed()

    def __enter__(self) -> RecordedUdpSender:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _build_record(self, payload: bytes, sent_bytes: int) -> bytes:
        try:
            packet = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            packet = None
        record = {
            "schema": ACTION_SEND_SCHEMA,
            "recorded_at_pc_ms": time.time_ns() // 1_000_000,
            "source": self._source,
            "destination": {
                "host": self._destination[0],
                "port": self._destination[1],
            },
            "sent_bytes": sent_bytes,
            "payload_sha256": hashlib.sha256(payload).hexdigest(),
            "payload_base64": base64.b64encode(payload).decode("ascii"),
            "packet": packet,
        }
        return (json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")

    def _write_loop(self) -> None:
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    return
                if self._writer_failed.is_set():
                    continue
                try:
                    self._rotate_if_needed(len(item))
                    self._write_all(item)
                    if not self._part_has_records:
                        self._part_has_records = True
                        self._prune_old_files()
                except OSError as exc:
                    self._set_failure(f"action journal写盘失败: {exc}")
        finally:
            final_path = Path(self._journal.name)
            final_has_records = self._part_has_records
            self._journal.close()
            try:
                if final_has_records:
                    self._prune_old_files()
                else:
                    final_path.unlink(missing_ok=True)
            except OSError as exc:
                self._set_failure(f"action journal收尾失败: {exc}")

    def _require_available(self) -> None:
        if self._closed:
            raise ActionJournalUnavailable("RecordedUdpSender已经关闭")
        if self._writer_failed.is_set():
            raise ActionJournalUnavailable(self._failure_message or "action journal不可用")

    def _part_path(self, part_index: int) -> Path:
        return self._journal_dir / f"{self._session_name}.part{part_index:04d}.jsonl"

    def _rotate_if_needed(self, record_size: int) -> None:
        if self._current_size == 0 or self._current_size + record_size <= self._max_file_bytes:
            return
        self._journal.close()
        self._part_index += 1
        self._journal = self._open_locked_part(self._part_path(self._part_index))
        self._current_size = 0
        self._part_has_records = False

    def _write_all(self, data: bytes) -> None:
        remaining = memoryview(data)
        while remaining:
            written = self._journal.write(remaining)
            if written is None or written <= 0:
                raise OSError("action journal出现短写")
            self._current_size += written
            remaining = remaining[written:]

    def _prune_old_files(self) -> None:
        lock_path = self._journal_dir / ".retention.lock"
        with lock_path.open("a+b") as directory_lock:
            fcntl.flock(directory_lock.fileno(), fcntl.LOCK_EX)
            files = sorted(
                self._journal_dir.glob("*.jsonl"),
                key=lambda path: (path.stat().st_mtime_ns, path.name),
            )
            excess = max(0, len(files) - self._retained_files)
            current_path = Path(self._journal.name)
            for stale_path in files:
                if excess == 0:
                    break
                if stale_path == current_path:
                    continue
                try:
                    with stale_path.open("rb") as candidate:
                        fcntl.flock(
                            candidate.fileno(),
                            fcntl.LOCK_EX | fcntl.LOCK_NB,
                        )
                        stale_path.unlink(missing_ok=True)
                        excess -= 1
                except (BlockingIOError, FileNotFoundError):
                    continue

    @staticmethod
    def _open_locked_part(path: Path):
        journal = path.open("ab", buffering=0)
        try:
            fcntl.flock(journal.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            journal.close()
            raise
        return journal

    def _raise_if_failed(self) -> None:
        if self._writer_failed.is_set():
            raise ActionJournalUnavailable(
                self._failure_message or "action journal不可用"
            )

    def _set_failure(self, message: str) -> None:
        if self._writer_failed.is_set():
            return
        self._failure_message = message
        self._writer_failed.set()
        print(
            f"{message}; subsequent PC→Orin sends are refused",
            file=sys.stderr,
            flush=True,
        )
