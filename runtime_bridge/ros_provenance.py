"""Explicit adapters between machine-state provenance and ROS header time."""

from __future__ import annotations


def epoch_ms_to_ros_time_fields(epoch_ms: int) -> tuple[int, int]:
    """Convert an Orin epoch timestamp in milliseconds into ROS Time fields.

    The caller must retain the source identity separately: ROS ``Header`` has no
    sequence field.  This adapter deliberately does not accept an STM32 boot
    tick because it is not an epoch clock and cannot be represented as ROS time.
    """
    if isinstance(epoch_ms, bool) or not isinstance(epoch_ms, int) or epoch_ms < 0:
        raise ValueError("epoch_ms 必须是非负整数")
    seconds, milliseconds = divmod(epoch_ms, 1000)
    return seconds, milliseconds * 1_000_000


def set_ros_header_stamp(header: object, epoch_ms: int) -> None:
    """Write an Orin source timestamp into a ROS-like header stamp object."""
    seconds, nanoseconds = epoch_ms_to_ros_time_fields(epoch_ms)
    header.stamp.sec = seconds
    header.stamp.nanosec = nanoseconds
