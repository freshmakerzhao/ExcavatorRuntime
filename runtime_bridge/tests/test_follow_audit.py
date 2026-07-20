import json

import pytest

from runtime_bridge.follow_audit import FollowAuditWriter, build_follow_audit_record


def _record(sequence):
    return build_follow_audit_record(
        sequence=sequence,
        trajectory_id="live-field-cycle-dig-001",
        state_seq=100 + sequence,
        state_stamp_ms=2000 + sequence,
        elapsed_ms=50.0 * sequence,
        observation=[0.0] * 38,
        raw_normalized=[1.0, -1.0, 0.5, 0.25],
        applied_normalized=[0.1, -0.1, 0.05, 0.0],
        physical_action=[-0.00351, 0.00444, -0.002095, 0.0],
    )


def test_follow_audit_writes_complete_jsonl_and_atomic_latest_snapshot(tmp_path):
    journal_directory = tmp_path / "journal"
    latest = tmp_path / "latest.json"
    writer = FollowAuditWriter(journal_directory, latest)

    writer.submit(_record(1))
    writer.submit(_record(2))
    writer.close()

    paths = list(journal_directory.glob("*.jsonl"))
    assert len(paths) == 1
    records = [json.loads(line) for line in paths[0].read_text().splitlines()]
    assert [record["sequence"] for record in records] == [1, 2]
    latest_record = json.loads(latest.read_text())
    assert latest_record["sequence"] == 2
    assert latest_record["observation_schema"] == "scale_excavator_v2_38d"
    assert latest_record["applied_normalized"] == [0.1, -0.1, 0.05, 0.0]
    assert writer.is_healthy


def test_follow_audit_rejects_incomplete_or_non_finite_control_evidence():
    with pytest.raises(ValueError, match="38 finite"):
        build_follow_audit_record(
            sequence=1,
            trajectory_id="trajectory-001",
            state_seq=1,
            state_stamp_ms=1,
            elapsed_ms=1.0,
            observation=[0.0] * 37,
            raw_normalized=[0.0] * 4,
            applied_normalized=[0.0] * 4,
            physical_action=[0.0] * 4,
        )
    with pytest.raises(ValueError, match="four finite"):
        build_follow_audit_record(
            sequence=1,
            trajectory_id="trajectory-001",
            state_seq=1,
            state_stamp_ms=1,
            elapsed_ms=1.0,
            observation=[0.0] * 38,
            raw_normalized=[0.0, 0.0, float("nan"), 0.0],
            applied_normalized=[0.0] * 4,
            physical_action=[0.0] * 4,
        )
