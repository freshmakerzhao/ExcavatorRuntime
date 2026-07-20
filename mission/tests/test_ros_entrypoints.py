from pathlib import Path


RUNTIME_ROS = Path(__file__).resolve().parents[1] / "runtime_ros"


def test_ros_executables_use_the_jazzy_system_python():
    for path in sorted(RUNTIME_ROS.glob("*.py")):
        if path.name in {"__init__.py", "no_motion_backend.py"}:
            continue
        source = path.read_bytes()
        assert source.splitlines()[0] == b"#!/usr/bin/python3"
        assert b'if __name__ == "__main__":' in source
