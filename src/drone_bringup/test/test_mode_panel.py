"""Exercise the source-free mode Panel as part of the ROS workspace test suite."""

import subprocess
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]


def test_mode_panel_smoke():
    subprocess.run(
        [sys.executable, str(WORKSPACE / "scripts" / "test_mode_panel.py")],
        cwd=WORKSPACE,
        check=True,
        timeout=40,
    )
