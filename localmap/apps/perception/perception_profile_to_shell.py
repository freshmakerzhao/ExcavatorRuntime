#!/usr/bin/env python3
"""把经过校验的感知 profile 转成供 Bash source 的安全环境变量。"""

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.perception_profile import load_perception_profile, perception_stack_environment


def main() -> int:
    parser = argparse.ArgumentParser(description="输出感知 profile 对应的 shell 环境变量")
    parser.add_argument("--profile", type=Path, required=True, help="perception profile JSON")
    args = parser.parse_args()
    profile = load_perception_profile(args.profile)
    for key, value in perception_stack_environment(profile).items():
        print(f"export {key}={shlex.quote(value)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
