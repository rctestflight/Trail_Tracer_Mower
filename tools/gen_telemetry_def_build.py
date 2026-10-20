"""PlatformIO extra script: writes the telemetry definition of this build, from the telemetry table with the build
flags (-D ...), to <build folder>/telemetry_def.json. A table error stops the build here, before the compile.
docs/telemetry_def.json (the default build) stays as it is: python tools/gen_telemetry_def.py writes it."""
import os
import re
import subprocess
import sys

Import("env")  # noqa: F821 (PlatformIO SCons)

flags = re.findall(r"-D\s*([A-Za-z_]\w*)", str(env.GetProjectOption("build_flags")))  # noqa: F821
out = os.path.join(env.subst("$BUILD_DIR"), "telemetry_def.json")  # noqa: F821
cmd = [sys.executable, os.path.join(env.subst("$PROJECT_DIR"), "tools", "gen_telemetry_def.py"), "--quiet", "-o", out]  # noqa: F821
for f in flags:
    cmd += ["-D", f]
r = subprocess.run(cmd)
if r.returncode:
    env.Exit(1)  # noqa: F821
