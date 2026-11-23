"""Build and run the ODrive recovery state-machine tests with a host C++ compiler."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CXX") or next(
    (path for name in ("g++", "clang++", "zig") if (path := shutil.which(name))), None
)
if not compiler:
    sys.exit("Set CXX to a host C++ compiler (g++, clang++, or zig).")

command = [compiler]
if Path(compiler).stem.lower() == "zig":
    command.append("c++")
with tempfile.TemporaryDirectory(prefix="odrive-recovery-") as build:
    exe = Path(build) / ("test.exe" if os.name == "nt" else "test")
    subprocess.run(command + [
        "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "include"), str(Path(__file__).with_name("main.cpp")),
        "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
