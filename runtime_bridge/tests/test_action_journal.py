import base64
import hashlib
import json
import socket
import tempfile
import time
import unittest
from pathlib import Path

from runtime_bridge.action_journal import ActionJournalUnavailable, RecordedUdpSender
from runtime_bridge.protocol import encode_packet, make_zero_action
from runtime_bridge.runtime_config import ActionJournalConfig


def journal_config(directory: str, *, max_file_bytes: int = 1048576, retained_files: int = 4):
    return ActionJournalConfig(
        directory=Path(directory),
        max_file_bytes=max_file_bytes,
        retained_files=retained_files,
    )


class ActionJournalTest(unittest.TestCase):
    def test_successful_udp_send_is_preserved_as_readable_and_exact_payload(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1.0)
        destination = receiver.getsockname()
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packet = make_zero_action(seq=7, valid_for_ms=100, stamp_ms=123456)
        payload = encode_packet(packet)

        try:
            with tempfile.TemporaryDirectory() as directory:
                with RecordedUdpSender(
                    sender_socket,
                    destination,
                    journal_config=journal_config(directory),
                    source="test_sender",
                ) as sender:
                    sender.send(payload)
                    journal_path = sender.journal_path

                received, _ = receiver.recvfrom(4096)
                records = [
                    json.loads(line)
                    for line in journal_path.read_text(encoding="utf-8").splitlines()
                ]
        finally:
            sender_socket.close()
            receiver.close()

        self.assertEqual(received, payload)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["schema"], "pc_orin_action_send_v1")
        self.assertEqual(records[0]["source"], "test_sender")
        self.assertEqual(records[0]["destination"]["port"], destination[1])
        self.assertEqual(records[0]["packet"], packet.to_dict())
        self.assertEqual(base64.b64decode(records[0]["payload_base64"]), payload)
        self.assertEqual(records[0]["payload_sha256"], hashlib.sha256(payload).hexdigest())

    def test_failed_udp_send_is_not_written_as_sent(self):
        closed_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        closed_socket.close()

        with tempfile.TemporaryDirectory() as directory:
            with RecordedUdpSender(
                closed_socket,
                ("127.0.0.1", 18082),
                journal_config=journal_config(directory),
                source="test_sender",
            ) as sender:
                with self.assertRaises(OSError):
                    sender.send(b"not-sent")
                journal_path = sender.journal_path

            self.assertFalse(journal_path.exists())

    def test_rotates_and_retains_only_configured_number_of_files(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            with tempfile.TemporaryDirectory() as directory:
                with RecordedUdpSender(
                    sender_socket,
                    receiver.getsockname(),
                    journal_config=journal_config(
                        directory,
                        max_file_bytes=1024,
                        retained_files=2,
                    ),
                    source="test_sender",
                ) as sender:
                    for seq in range(6):
                        sender.send(
                            encode_packet(
                                make_zero_action(seq=seq, valid_for_ms=100, stamp_ms=123456 + seq)
                            )
                        )

                journal_files = sorted(Path(directory).glob("*.jsonl"))
                retained_records = [
                    json.loads(line)
                    for path in journal_files
                    for line in path.read_text(encoding="utf-8").splitlines()
                ]
        finally:
            sender_socket.close()
            receiver.close()

        self.assertEqual(len(journal_files), 2)
        self.assertEqual([record["packet"]["seq"] for record in retained_records], [4, 5])

    def test_writer_failure_refuses_next_datagram_before_send(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.1)
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            with tempfile.TemporaryDirectory() as directory:
                journal_dir = Path(directory)
                with self.assertRaises(ActionJournalUnavailable):
                    with RecordedUdpSender(
                        sender_socket,
                        receiver.getsockname(),
                        journal_config=journal_config(
                            directory,
                            max_file_bytes=1024,
                            retained_files=2,
                        ),
                        source="test_sender",
                    ) as sender:
                        sender.send(encode_packet(make_zero_action(0, 100, stamp_ms=1)))
                        self._wait_until(lambda: sender.journal_path.stat().st_size > 0)
                        journal_dir.chmod(0o500)
                        try:
                            sender.send(encode_packet(make_zero_action(1, 100, stamp_ms=2)))
                            self._wait_until(lambda: not sender.is_healthy)
                        finally:
                            journal_dir.chmod(0o700)

                        receiver.recvfrom(4096)
                        receiver.recvfrom(4096)
                        with self.assertRaises(ActionJournalUnavailable):
                            sender.send(encode_packet(make_zero_action(2, 100, stamp_ms=3)))
                        with self.assertRaises(TimeoutError):
                            receiver.recvfrom(4096)
        finally:
            sender_socket.close()
            receiver.close()

    def test_retention_does_not_unlink_another_active_sender(self):
        first_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        second_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))

        try:
            with tempfile.TemporaryDirectory() as directory:
                config = journal_config(directory, max_file_bytes=1024, retained_files=1)
                with RecordedUdpSender(
                    first_socket,
                    receiver.getsockname(),
                    journal_config=config,
                    source="first_sender",
                ) as first:
                    first.send(encode_packet(make_zero_action(0, 100, stamp_ms=1)))
                    self._wait_until(lambda: first.journal_path.stat().st_size > 0)
                    with RecordedUdpSender(
                        second_socket,
                        receiver.getsockname(),
                        journal_config=config,
                        source="second_sender",
                    ) as second:
                        second.send(encode_packet(make_zero_action(1, 100, stamp_ms=2)))
                        self._wait_until(lambda: second.journal_path.stat().st_size > 0)
                        self.assertTrue(first.journal_path.exists())
                        self.assertTrue(second.journal_path.exists())
        finally:
            first_socket.close()
            second_socket.close()
            receiver.close()

    def test_empty_session_does_not_evict_recorded_history(self):
        sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))

        try:
            with tempfile.TemporaryDirectory() as directory:
                config = journal_config(directory, retained_files=1)
                with RecordedUdpSender(
                    sender_socket,
                    receiver.getsockname(),
                    journal_config=config,
                    source="recorded_sender",
                ) as recorded:
                    recorded.send(encode_packet(make_zero_action(0, 100, stamp_ms=1)))
                    history_path = recorded.journal_path

                with RecordedUdpSender(
                    sender_socket,
                    receiver.getsockname(),
                    journal_config=config,
                    source="empty_sender",
                ):
                    pass

                self.assertTrue(history_path.exists())
                self.assertEqual(list(Path(directory).glob("*.jsonl")), [history_path])
        finally:
            sender_socket.close()
            receiver.close()

    @staticmethod
    def _wait_until(predicate, timeout_s: float = 1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        raise AssertionError("condition did not become true before timeout")


if __name__ == "__main__":
    unittest.main()
