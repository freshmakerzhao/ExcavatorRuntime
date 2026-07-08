#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _compat import expose_app, run_app_main

_module = expose_app(Path(__file__).resolve().parents[1] / "apps" / "visualization" / "publish_reachable_workspace_markers.py", globals())

if __name__ == "__main__":
    raise SystemExit(run_app_main(_module))
