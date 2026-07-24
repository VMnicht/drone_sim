import subprocess
import sys
from pathlib import Path


def test_all_declared_runtime_parameters_are_covered_by_yaml():
    workspace = Path(__file__).resolve().parents[3]
    verifier = workspace / "scripts" / "verify_yaml_parameters.py"
    subprocess.run([sys.executable, str(verifier)], cwd=workspace, check=True)
