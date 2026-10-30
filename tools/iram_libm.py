# PlatformIO pre script, diagnostic build diag_iramfp (2026-10-28, float cost study): the float library functions
# of the wire solve and the autopilot run from internal RAM, not flash. The script copies their objects from the
# toolchain libm.a unchanged (same machine code), renames the code and literal sections to .iram1.* and the constant
# tables to .dram1.* (the linker script puts both in internal RAM), and links them before libm. The linker then takes
# these definitions and pulls nothing else of them from libm.a.
import os
import subprocess

Import("env")  # noqa: F821

MEMBERS = ["wf_log", "ef_log", "wf_exp", "ef_exp", "wf_atan2", "ef_atan2", "sf_atan", "sf_sin", "sf_cos", "kf_sin",
           "kf_cos", "ef_rem_pio2", "wf_asin", "ef_asin", "wf_acos", "ef_acos", "wf_sqrt", "ef_sqrt", "sf_fabs"]

tc = env.PioPlatform().get_package_dir("toolchain-xtensa-esp32s2")  # noqa: F821
pre = os.path.join(tc, "bin", "xtensa-esp32s2-elf-")
libm = os.path.join(tc, "xtensa-esp32s2-elf", "lib", "libm.a")
out = os.path.join(env.subst("$BUILD_DIR"), "iram_libm")  # noqa: F821
os.makedirs(out, exist_ok=True)
objs = []
for m in MEMBERS:
    name = "lib_a-%s.o" % m
    subprocess.check_call([pre + "ar", "x", libm, name], cwd=out)
    dst = os.path.join(out, "iram_" + name)
    subprocess.check_call([pre + "objcopy",
                           "--rename-section", ".literal=.iram1.libm_literal",
                           "--rename-section", ".text=.iram1.libm_text",
                           "--rename-section", ".rodata=.dram1.libm_rodata",
                           os.path.join(out, name), dst])
    objs.append(dst)
env.Append(LINKFLAGS=objs)  # noqa: F821
print("iram_libm: %d libm objects into internal RAM" % len(objs))
