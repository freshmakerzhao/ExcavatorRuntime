"""Offline error summaries for bucket-tip calibration records.

The evaluator deliberately never changes geometry, joint offsets, or frame
transforms.  It turns a captured, matched FK/physical-measurement data set into
an auditable residual report that can be used as the diagnostic feedback loop.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Mapping

import numpy as np


class BucketTipCalibrationError(ValueError):
    """Raised when a calibration capture cannot support a trustworthy result."""


def evaluate_bucket_tip_records(capture: Mapping[str, Any]) -> dict[str, Any]:
    """Summarise measured-minus-FK position residuals by motion phase.

    ``samples`` must contain paired positions in ``machine_root``, in metres.
    The output contains no fitted replacement calibration: its mean residual is
    only evidence for a later, separately reviewed calibration decision.
    """
    if capture.get("schema") != "bucket_tip_calibration.v1":
        raise BucketTipCalibrationError("schema 必须为 'bucket_tip_calibration.v1'")
    samples = capture.get("samples")
    if not isinstance(samples, list) or not samples:
        raise BucketTipCalibrationError("samples 必须是非空数组")

    grouped_errors: dict[str, list[np.ndarray]] = defaultdict(list)
    sequences: list[int] = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise BucketTipCalibrationError(f"samples[{index}] 必须是对象")
        phase = sample.get("phase")
        if not isinstance(phase, str) or not phase:
            raise BucketTipCalibrationError(f"samples[{index}].phase 必须是非空字符串")
        sequence = sample.get("state_seq")
        if not isinstance(sequence, int):
            raise BucketTipCalibrationError(f"samples[{index}].state_seq 必须是整数")
        predicted = _position(sample, "fk_tip_machine_root_m", index)
        measured = _position(sample, "measured_tip_machine_root_m", index)
        grouped_errors[phase].append(measured - predicted)
        sequences.append(sequence)

    phases = {phase: _summarise(errors) for phase, errors in sorted(grouped_errors.items())}
    duplicate_sequences = sorted({sequence for sequence in sequences if sequences.count(sequence) > 1})
    return {
        "schema": "bucket_tip_calibration_report.v1",
        "sample_count": len(samples),
        "phases": phases,
        "quality": {"duplicate_state_sequences": duplicate_sequences},
    }


def _position(sample: Mapping[str, Any], field: str, index: int) -> np.ndarray:
    value = np.asarray(sample.get(field), dtype=np.float64)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise BucketTipCalibrationError(f"samples[{index}].{field} 必须是3个有限米制坐标")
    return value


def _summarise(errors: list[np.ndarray]) -> dict[str, Any]:
    values = np.asarray(errors, dtype=np.float64)
    norms = np.linalg.norm(values, axis=1)
    return {
        "sample_count": int(len(values)),
        "mean_error_m": np.mean(values, axis=0).astype(float).tolist(),
        "rms_error_m": np.sqrt(np.mean(values * values, axis=0)).astype(float).tolist(),
        "max_error_norm_m": float(np.max(norms)),
    }
