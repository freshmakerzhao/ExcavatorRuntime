"""保持真机源时间，同时为显式 rosbag replay 提供 wall-time 适配。"""

from __future__ import annotations

from typing import TypeVar


Stamp = TypeVar("Stamp")


def select_cloud_header_stamp(
    input_stamp: Stamp,
    current_stamp: Stamp,
    *,
    replay_restamp: bool,
) -> Stamp:
    """默认保留传感器时间；仅显式 replay 模式采用当前节点时间。"""
    return current_stamp if replay_restamp else input_stamp
