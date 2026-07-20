from pathlib import Path


ENTRYPOINT = (
    Path(__file__).resolve().parents[1]
    / "localmap_core/runtime_ros/fixture_plan_action_server.py"
)


def test_fixture_plan_entrypoint_uses_the_jazzy_system_python():
    source = ENTRYPOINT.read_bytes()
    assert source.splitlines()[0] == b"#!/usr/bin/python3"
    assert b'if __name__ == "__main__":' in source
