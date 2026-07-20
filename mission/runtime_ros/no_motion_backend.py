"""Shadow execution Adapter: it has no sender and can only confirm quiescence."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NoMotionBackend:
    sender_constructed: bool = False
    action_datagrams: int = 0

    def stop_and_confirm(self) -> bool:
        """Confirm the compile-time no-motion implementation is quiescent."""
        return not self.sender_constructed and self.action_datagrams == 0
