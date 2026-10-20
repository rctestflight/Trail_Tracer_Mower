"""PlatformIO extra script (env settings_newkey): writes the settings definition of this build, from the table with
its build flags (-D SETTINGS_*), to <build folder>/settings_def.json. docs/settings_def.json stays as it is."""
import os
import re
import subprocess
import sys

Import("env")  # noqa: F821 (PlatformIO SCons)

flags = re.findall(r"-D\s*(SETTINGS_\w+)", str(env.GetProjectOption("build_flags")))  # noqa: F821
out = os.path.join(env.subst("$BUILD_DIR"), "settings_def.json")  # noqa: F821
cmd = [sys.executable, os.path.join(env.subst("$PROJECT_DIR"), "tools", "gen_settings_def.py"), "--quiet", "-o", out]  # noqa: F821
for f in flags:
    cmd += ["-D", f]
subprocess.run(cmd, check=True)
