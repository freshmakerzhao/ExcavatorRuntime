#!/usr/bin/env python3
"""Evaluate a captured Bucket Tip calibration data set without changing calibration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.bucket_tip_calibration import BucketTipCalibrationError, evaluate_bucket_tip_records
from localmap_core.io import load_json, write_json


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只读评估Bucket Tip标定采样；不会修改几何、零位或外参。"
    )
    parser.add_argument("--capture", type=Path, required=True, help="bucket_tip_calibration.v1采样JSON")
    parser.add_argument("--output", type=Path, required=True, help="写入可审阅的误差报告JSON")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        report = evaluate_bucket_tip_records(load_json(args.capture))
    except (BucketTipCalibrationError, OSError, ValueError) as exc:
        print(f"bucket tip calibration evaluation failed: {exc}", file=sys.stderr)
        return 2
    write_json(args.output, report)
    print(f"wrote calibration report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
