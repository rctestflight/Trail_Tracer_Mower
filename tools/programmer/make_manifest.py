"""make_manifest.py - put one Rx firmware build into a build source folder for the Rx programmer page.

A build source folder holds index.json and one folder per build: <repo>/<branch>/<commit>/ with the image
files and manifest.json (chip, board revisions, version, source repository, branch and commit, files with
size and SHA-256, and the load data of the chip). The page reads index.json, then the manifests.

ESP32 firmware from a PlatformIO build:
  python tools/make_manifest.py esp32 --repo-dir <firmware repo> --out builds
  (files from .pio/build/<env>; version from ROVER_FW_VERSION in src/main.cpp; git data from the repo)
ESP32 files from a folder, with the source named by hand:
  python tools/make_manifest.py esp32 --files <folder with firmware.bin ...> --repo <owner/repo>
      --branch <branch> --commit <sha> --version <version> --out builds
STM32 app update files (.commit_header, .bin.encrypted; signed and encrypted, never a plain image):
  python tools/make_manifest.py stm32 --header <x.commit_header> --image <x.bin.encrypted> --repo <owner/repo>
      --branch <branch> --commit <sha> --out builds

An existing build folder is never written again: same files, nothing to do; other files, stop.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import zlib
from pathlib import Path

PIO_BOOT_APP0 = Path.home() / ".platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
BOARDS = ["RX_REVA"]                       # Trail Tracer Rx revision A: the only revision built now
ESP32_LINKS = ["v1", "v2"]                 # STM32 link versions of rover firmware 0.5.x (v1 and v2 frames)



def git(repo_dir, *args):
    return subprocess.run(["git", "-C", str(repo_dir), *args], capture_output=True, text=True, check=True).stdout.strip()


def repo_info(repo_dir):
    url = git(repo_dir, "remote", "get-url", "origin")
    m = re.search(r"github\.com[:/](.+?)(\.git)?$", url)
    dirty = bool(git(repo_dir, "status", "--porcelain", "--untracked-files=no"))
    return {"repo": m.group(1) if m else url, "branch": git(repo_dir, "branch", "--show-current"),
            "commit": git(repo_dir, "rev-parse", "HEAD") + ("-dirty" if dirty else ""),
            "subject": git(repo_dir, "log", "-1", "--format=%s")}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def partition_table(data, name):
    """The entries of an ESP partition table (32 B each, magic 0xAA50), its name and its CRC-32. Rover firmware
    0.5.55 and later report both in "ota status": table=<name> table_crc=<CRC-32 of the 3072 B at 0x8000>."""
    rows = []
    for i in range(0, len(data) - 31, 32):
        e = data[i:i + 32]
        if e[:2] != b"\xaa\x50":
            break
        ptype, sub, off, size = e[2], e[3], *struct.unpack("<II", e[4:12])
        rows.append({"name": e[12:28].rstrip(b"\0").decode(), "type": ptype, "subtype": sub,
                     "offset": hex(off), "size": hex(size)})
    if not rows:
        raise SystemExit("stop: partitions.bin has no partition table entries")
    return {"name": name, "crc32": f"{zlib.crc32(data):08x}", "entries": rows}


def data_ranges(data):
    """The data partitions of the new table except otadata (boot_app0 writes it): erased when the board had
    another table, so no settings of the old firmware stay (ROM download mode has no full erase without the stub)."""
    rows = partition_table(data, "")["entries"]
    return [{"from": r["offset"], "to": hex(int(r["offset"], 16) + int(r["size"], 16))} for r in rows if r["type"] == 1 and r["subtype"] != 0]


def place(out, src, files, manifest):
    if getattr(place, "label", None):                 # the plain label of programmer.json, shown on the page
        manifest["label"] = place.label
    """Copy the files into <out>/<repo>/<branch>/<commit>/, write manifest.json, add the build to index.json."""
    s = manifest["source"]
    rel = Path(s["repo"].split("/")[-1]) / re.sub(r"[^A-Za-z0-9._-]", "-", s["branch"]) / s["commit"][:12]
    if manifest["chip"] != "esp32s2":                 # STM32 files of the same commit: a folder of their own
        rel = rel / "stm32"
    d = Path(out) / rel
    manifest["files"] = [{"path": n, "size": os.path.getsize(p), "sha256": sha(p)} for n, p in files.items()]
    if d.exists():
        old = json.loads((d / "manifest.json").read_text())
        if {f["path"]: f["sha256"] for f in old["files"]} == {f["path"]: f["sha256"] for f in manifest["files"]}:
            print(f"{d}: this build is there already; nothing to do")
            return
        raise SystemExit(f"stop: {d} holds other files for the same commit; build from a clean commit")
    d.mkdir(parents=True)
    for n, p in files.items():
        shutil.copyfile(p, d / n)
    (d / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    idx_path = Path(out) / "index.json"
    idx = json.loads(idx_path.read_text()) if idx_path.exists() else {"schema": 1, "builds": []}
    idx["builds"].append({"manifest": (rel / "manifest.json").as_posix(), "repo": s["repo"], "branch": s["branch"],
                          "commit": s["commit"], "subject": s.get("subject", ""), "chip": manifest["chip"],
                          "name": manifest["name"], "version": manifest["version"], "built_utc": s["built_utc"]})
    idx["builds"].sort(key=lambda b: b["built_utc"], reverse=True)
    tmp = idx_path.with_suffix(".tmp")
    tmp.write_text(json.dumps(idx, indent=2) + "\n")
    os.replace(tmp, idx_path)
    print(f"{d}: {manifest['name']} {manifest['version']}, {len(files)} files")


def source(a, repo_dir):
    s = repo_info(repo_dir) if repo_dir else {"repo": a.repo, "branch": a.branch, "commit": a.commit, "subject": a.subject or ""}
    for k in ("repo", "branch", "commit"):
        if not s.get(k):
            raise SystemExit(f"stop: no {k} (give --repo-dir, or --repo --branch --commit)")
    s["built_utc"] = a.built or dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    s["builder"] = "local"
    return s


def esp32(a):
    repo_dir = Path(a.repo_dir) if a.repo_dir else None
    fdir = Path(a.files) if a.files else repo_dir / ".pio/build" / a.env
    files = {n: fdir / n for n in ("bootloader.bin", "partitions.bin", "firmware.bin")}
    files["boot_app0.bin"] = fdir / "boot_app0.bin" if (fdir / "boot_app0.bin").exists() else PIO_BOOT_APP0
    for n, p in files.items():
        if not p.exists():
            raise SystemExit(f"stop: no {p}")
    fw = files["firmware.bin"].read_bytes()
    if fw[0] != 0xE9 or struct.unpack("<H", fw[12:14])[0] != 2 or struct.unpack("<I", fw[0x20:0x24])[0] != 0xABCD5432:
        raise SystemExit("stop: firmware.bin is not an ESP32-S2 app image")
    rover = a.family == "rover"
    if rover and b"rover-firmware" not in fw:
        raise SystemExit("stop: firmware.bin is not rover firmware (no 'rover-firmware' text)")
    version = a.version
    if not version and repo_dir:
        m = re.search(r'#define ROVER_FW_VERSION "([^"]+)"', (repo_dir / "src/main.cpp").read_text(encoding="utf-8"))
        version = m and m.group(1)
    if rover and (not version or version.encode() not in fw):
        raise SystemExit(f"stop: version {version!r} is not in firmware.bin")
    manifest = {
        "schema": 1, "product": "trail-tracer-rx", "chip": "esp32s2", "boards": BOARDS,
        "name": "ESP32 rover firmware" if rover else "ESP32 firmware (no version report)", "version": version, "source": source(a, repo_dir),
        "esp32": {
            "partition_table": dict(partition_table(files["partitions.bin"].read_bytes(), a.table), file="partitions.bin"),
            "usb_update": {"file": "firmware.bin"} if rover else None,
            "full_load": {"flash_mode": "dio", "flash_freq": "80m", "flash_size": "4MB",
                          "parts": [{"offset": "0x1000", "file": "bootloader.bin"}, {"offset": "0x8000", "file": "partitions.bin"},
                                    {"offset": "0xe000", "file": "boot_app0.bin"}, {"offset": "0x10000", "file": "firmware.bin"}],
                          "erase_if_other_table": data_ranges(files["partitions.bin"].read_bytes())},
            "stm32_links": ESP32_LINKS if rover else ["v1"],
        },
    }
    place(a.out, None, files, manifest)


def stm32(a):
    header, image = Path(a.header), Path(a.image)
    h, img = header.read_bytes(), image.read_bytes()
    if len(h) != 52 or h[:4] != b"SIAP":
        raise SystemExit("stop: the header is not a 52-byte SIAP commit header")
    _, product, ver, size, chunk, count = struct.unpack("<4sIIIHH", h[:20])
    if len(img) != count * (chunk + 16):
        raise SystemExit(f"stop: image {len(img)} B is not {count} chunks of {chunk + 16} B")
    version = f"{ver & 255}.{ver >> 8 & 255}.{ver >> 16 & 255}"
    if a.version and a.version != version:
        raise SystemExit(f"stop: the header says {version}, not {a.version}")
    manifest = {
        "schema": 1, "product": "trail-tracer-rx", "chip": "stm32g070", "boards": BOARDS,
        "name": "STM32 app", "version": version, "source": source(a, Path(a.repo_dir) if a.repo_dir else None),
        # link: app 2.x speaks link v2; 0.x and 1.x speak v1
        "stm32": {"app_version": version, "link": a.link or ("v2" if ver & 255 >= 2 else "v1"),
                  "boot_checker_min": "0.1.0", "product_id": hex(product),
                  "update": {"header": "app.commit_header", "image": "app.bin.encrypted", "chunk_size": chunk + 16, "chunks": count},
                  "needs_esp32": "0.5.26"},
    }
    keys = {"app_version", "link", "boot_checker_min", "product_id", "update", "needs_esp32"}
    if set(manifest["stm32"]) != keys or set(manifest["stm32"]["update"]) != {"header", "image", "chunk_size", "chunks"}:
        raise SystemExit(f"stop: STM32 manifest keys {sorted(manifest['stm32'])} are not {sorted(keys)}")
    place(a.out, None, {"app.commit_header": header, "app.bin.encrypted": image}, manifest)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("chip", choices=["esp32", "stm32"])
    ap.add_argument("--out", required=True, help="the build source folder (index.json is there)")
    ap.add_argument("--repo-dir", help="the git repo of the build: source data from git")
    ap.add_argument("--repo"), ap.add_argument("--branch"), ap.add_argument("--commit"), ap.add_argument("--subject")
    ap.add_argument("--version"), ap.add_argument("--built", help="build time, UTC, 2026-10-12T02:00:07Z")
    ap.add_argument("--label", help="plain label of the item (programmer.json), shown on the page")
    ap.add_argument("--env", default="lolin_s2_mini", help="PlatformIO environment (esp32)")
    ap.add_argument("--files", help="folder with the ESP32 files, in place of the PlatformIO build folder")
    ap.add_argument("--header"), ap.add_argument("--image")
    ap.add_argument("--family", default="rover", choices=["rover", "main"], help="rover firmware, or a firmware that does not report its version (esp32)")
    ap.add_argument("--table", default="rover-4mb-1", help="partition table name, as the firmware reports it (esp32)")
    ap.add_argument("--link", help="STM32 link version of the app (stm32); default from the app version: 2.x v2, else v1")
    a = ap.parse_args()
    place.label = a.label
    (esp32 if a.chip == "esp32" else stm32)(a)


if __name__ == "__main__":
    sys.exit(main())
