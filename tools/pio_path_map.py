"""PlatformIO extra script: no local path in the binary. File names in asserts and logs (__FILE__) start
with "pio/" for the PlatformIO packages and "./" for this project, on any PC."""
Import("env")  # noqa: F821 (PlatformIO SCons)

maps = []
for path, short in ((env.subst("$PROJECT_PACKAGES_DIR"), "pio"), (env.subst("$PROJECT_DIR"), ".")):  # noqa: F821
    for form in {path, path.replace("\\", "/")}:
        maps += [f"-fmacro-prefix-map={form}={short}", f"-ffile-prefix-map={form}={short}"]
env.Append(CCFLAGS=maps)  # noqa: F821 (libraries and the framework build from clones of this env)
