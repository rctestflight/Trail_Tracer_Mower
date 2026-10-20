"""build_all.py - build the firmware the programmer page offers, from the public Rx firmware repos.

Which refs: the branches main, dev and dev-tyler and every tag of each repo; no other branch.
programmer.json at the root of a ref names what it offers (docs/programmer-json.md). A ref with a build
of its current commit is skipped. For each new commit: shallow clone, PlatformIO build (programmer.json "esp32"
env, else the default environment) with prefix maps (no local path goes into the image), and the STM32
update pairs (programmer.json "stm32", else every .commit_header with its .bin.encrypted); then
tools/make_manifest.py adds each build to <out> (index.json and <repo>/<ref>/<commit>/).

Drops the builds of refs that are no longer offered (branch or tag deleted) and of commits that no branch
or tag reaches any more; keeps only the build of the newest commit of each ref (KEEP = 1). A failed build is
reported and skipped. No GitHub API: one blobless clone of each repo gives refs, opt-in files and
reachable commits.

Usage: python tools/build_all.py <out folder>
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPOS = ["rctestflight/Trail_Tracer_Mower", "rctestflight/Trail-Tracer"]
KEEP = 1                                   # the newest commit of each ref only
BRANCHES = ("main", "dev", "dev-tyler")      # the branches on the page; every tag too
OPT_IN = "programmer.json"
HERE = Path(__file__).resolve().parent
LABEL = re.compile(r"^[A-Za-z0-9 .,:()+/_-]{1,60}$")        # a plain label: shown on the page as text
DESC = re.compile(r"^[A-Za-z0-9 .,:;()+/'_-]{1,160}$")      # a plain description of the branch or tag
ENV = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
PATH = re.compile(r"^(?!/)(?!.*\.\.)[A-Za-z0-9._/-]{1,200}$")


def git(*args, cwd=None, check=True):
    return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, check=check)


def slug(ref):
    return re.sub(r"[^A-Za-z0-9._-]", "-", ref)


def default_env(ini):
    m = re.search(r"^default_envs\s*=\s*([^\s,]+)", ini, re.M)
    return m.group(1) if m else re.search(r"^\[env:([^\]]+)\]", ini, re.M).group(1)


def read_opt_in(text):
    """programmer.json, checked: {"title", "description", "esp32": {"env", "label"}, "stm32": [{"header", "image",
    "label"}]}; every part optional, all text plain. Raises ValueError on anything else."""
    cfg = json.loads(text)
    if not isinstance(cfg, dict) or set(cfg) - {"title", "description", "esp32", "stm32"}:
        raise ValueError("keys: title, description, esp32, stm32")
    if "title" in cfg and not LABEL.match(str(cfg["title"])):
        raise ValueError("title: plain text, 60 characters at most")
    if "description" in cfg and not DESC.match(str(cfg["description"])):
        raise ValueError("description: plain text, 160 characters at most")
    e = cfg.get("esp32")
    if e is not None and not (isinstance(e, dict) and ENV.match(str(e.get("env", ""))) and LABEL.match(str(e.get("label", ""))) and set(e) <= {"env", "label"}):
        raise ValueError("esp32: {env, label}")
    for s in [] if cfg.get("stm32") == "latest" else cfg.get("stm32", []):
        if not (isinstance(s, dict) and set(s) <= {"header", "image", "label"} and PATH.match(str(s.get("header", "")))
                and str(s["header"]).endswith(".commit_header") and PATH.match(str(s.get("image", "")))
                and str(s["image"]).endswith(".bin.encrypted") and LABEL.match(str(s.get("label", "")))):
            raise ValueError("stm32: [{header .commit_header, image .bin.encrypted, label}]")
    return cfg


def newest_pair(names):
    """(header, image, version) of the newest STM32 update pair among file paths in stm32/: a .commit_header and
    a .bin.encrypted with the same name before the suffix, newest by the version vX.Y.Z in the name; or None."""
    names = set(names)
    best = None
    for h in names:
        if not (h.startswith("stm32/") and h.endswith(".commit_header")):
            continue
        img = h[:-len(".commit_header")] + ".bin.encrypted"
        m = re.search(r"v(\d+)\.(\d+)\.(\d+)", h.rsplit("/", 1)[-1])
        if img in names and m:
            v = tuple(int(x) for x in m.groups())
            if not best or v > best[2]:
                best = (h, img, v)
    return best


def clones(work):
    """One blobless bare clone of each repo: refs, opt-in files and reachable commits, no API calls."""
    out = {}
    for repo in REPOS:
        d = work / ("git-" + repo.replace("/", "-"))
        git("clone", "--quiet", "--bare", "--filter=blob:none", f"https://github.com/{repo}.git", str(d))
        out[repo] = d
    return out


def targets(cl):
    """(repo, ref, commit, programmer.json config or None, is tag): the branches in BRANCHES and every tag."""
    out = []
    for repo, d in cl.items():
        lines = git("-C", str(d), "for-each-ref", "--format=%(refname)\t%(objectname)\t%(*objectname)",
                    "refs/heads", "refs/tags").stdout.splitlines()
        for line in lines:
            name, obj, peeled = line.split("\t")
            sha, tag = peeled or obj, name.startswith("refs/tags/")
            ref = name.split("/", 2)[2]
            has = git("-C", str(d), "cat-file", "-e", f"{sha}:{OPT_IN}", check=False).returncode == 0
            if not (tag or ref in BRANCHES):
                continue
            cfg = None
            if has:
                try:
                    cfg = read_opt_in(git("-C", str(d), "show", f"{sha}:{OPT_IN}").stdout)
                except ValueError as ex:
                    print(f"{repo} {ref}: {OPT_IN} not valid ({ex}): not offered", flush=True)
                    continue
                if cfg.get("stm32") == "latest":       # the newest encrypted pair in stm32/, by the version in the name
                    files = git("-C", str(d), "ls-tree", "-r", "--name-only", sha, "stm32/", check=False).stdout.split()
                    pair = newest_pair(files)
                    if not pair:
                        print(f"{repo} {ref}: {OPT_IN} asks the latest STM32 pair, stm32/ has none: not offered", flush=True)
                        continue
                    cfg = {**cfg, "stm32": [{"header": pair[0], "image": pair[1], "label": "STM32 app"}]}
            out.append((repo, ref, sha, cfg, tag))
    return out


def inside(root, rel):
    p = (root / rel).resolve()
    if root.resolve() not in p.parents:
        raise ValueError(f"{rel} is outside the repo")
    return p


def build(repo, ref, sha, cfg, out, work):
    src = work / f"{repo.split('/')[-1]}-{slug(ref)}-{sha[:12]}"           # a new folder for each build
    git("clone", "--quiet", "--depth", "1", "--branch", ref, f"https://github.com/{repo}.git", str(src))
    head = git("-C", str(src), "rev-parse", "HEAD").stdout.strip()
    if head != sha:
        raise RuntimeError(f"head moved: {head[:12]}, not {sha[:12]}")
    esp = (cfg or {}).get("esp32")
    if cfg is None or esp:
        env = esp["env"] if esp else default_env((src / "platformio.ini").read_text(encoding="utf-8"))
        pio_home = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")).as_posix()
        s = src.as_posix()
        flags = f"-ffile-prefix-map={s}=. -fmacro-prefix-map={s}=. -ffile-prefix-map={pio_home}=pio -fmacro-prefix-map={pio_home}=pio"
        subprocess.run(["pio", "run", "-e", env, "-d", str(src)], check=True, env=dict(os.environ, PLATFORMIO_BUILD_FLAGS=flags))
        bdir = src / ".pio" / "build" / env
        fw = (bdir / "firmware.bin").read_bytes()
        cmd = [sys.executable, str(HERE / "make_manifest.py"), "esp32", "--files", str(bdir), "--repo", repo, "--branch", ref,
               "--commit", sha, "--subject", "", "--out", str(out)] + (["--label", esp["label"]] if esp else [])
        if b"rover-firmware" in fw:
            main_cpp = (src / "src" / "main.cpp").read_text(encoding="utf-8", errors="replace")
            versions = [v for v in re.findall(r'#define ROVER_FW_VERSION "([^"]+)"', main_cpp) if v.encode() in fw]
            if not versions:
                raise RuntimeError("no ROVER_FW_VERSION of src/main.cpp is in firmware.bin")
            cmd += ["--version", versions[0]]
        else:
            cmd += ["--family", "main", "--table", "main", "--version", f"{ref} {sha[:7]}"]
        subprocess.run(cmd, check=True)
    # STM32 update pairs (encrypted and signed); plain STM32 images are never taken.
    if cfg is not None:
        pairs = [(inside(src, s["header"]), inside(src, s["image"]), s["label"]) for s in cfg.get("stm32", [])]
    else:
        pairs = [(h, h.with_name(h.name[:-len(".commit_header")] + ".bin.encrypted"), None)
                 for h in sorted(src.rglob("*.commit_header")) if ".git" not in h.parts]
    for hdr, img, label in pairs:
        if not img.exists():
            if cfg is not None:
                raise RuntimeError(f"{img.name} is not in the branch")
            continue
        subprocess.run([sys.executable, str(HERE / "make_manifest.py"), "stm32", "--header", str(hdr), "--image", str(img),
                        "--repo", repo, "--branch", ref, "--commit", sha, "--subject", "", "--out", str(out)]
                       + (["--label", label] if label else []), check=True)


def prune(out, offered, reach):
    """Drop the builds of refs that are no longer offered and of commits that no branch or tag reaches,
    then keep the newest KEEP builds of each repo and ref: older folders and index entries go."""
    idx_path = out / "index.json"
    if not idx_path.exists():
        return
    idx = json.loads(idx_path.read_text())
    seen, keep = {}, []
    for b in sorted(idx["builds"], key=lambda b: b["built_utc"], reverse=True):
        why = ("the ref is no longer offered" if (b["repo"], b["branch"]) not in offered
               else "no branch or tag reaches it" if b["commit"] not in reach.get(b["repo"], {b["commit"]}) else None)
        k = (b["repo"], b["branch"], b["chip"])
        seen[k] = seen.get(k, 0) + (0 if why else 1)
        if not why and seen[k] > KEEP:
            why = f"more than {KEEP} builds of the ref"
        if why:
            shutil.rmtree(out / Path(b["manifest"]).parent, ignore_errors=True)
            print(f"dropped {b['repo']} {b['branch']} {b['commit'][:7]} {b['chip']}: {why}")
        else:
            keep.append(b)
    idx["builds"] = keep
    idx_path.write_text(json.dumps(idx, indent=2) + "\n")


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    failed = []
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        work = Path(tmp)
        cl = clones(work)
        tg = targets(cl)
        for repo, ref, sha, cfg, _ in tg:
            if (out / repo.split("/")[-1] / slug(ref) / sha[:12]).exists():
                continue
            print(f"== {repo} {ref} {sha[:12]}" + (f" ({OPT_IN})" if cfg is not None else ""), flush=True)
            try:
                build(repo, ref, sha, cfg, out, work)
            except Exception as ex:                            # one ref that does not build stops nothing else
                failed.append(f"{repo} {ref} {sha[:12]}: {ex}")
                print(f"   failed: {ex}", flush=True)
        reach = {repo: set(git("-C", str(d), "rev-list", "--all").stdout.split()) for repo, d in cl.items()}
        prune(out, {(r, ref) for r, ref, *_ in tg}, reach)
    # refs.json: the offered branches and tags with their menu title and description (programmer.json).
    refs = [{"repo": repo, "ref": ref, "tag": tag, **{k: cfg[k] for k in ("title", "description") if cfg and k in cfg}}
            for repo, ref, sha, cfg, tag in tg]
    (out / "refs.json").write_text(json.dumps({"refs": refs}, indent=2) + "\n")
    # releases.json: the tagged commits; the page marks those builds as releases.
    rel = {sha: ref for repo, ref, sha, cfg, tag in tg if tag}
    (out / "releases.json").write_text(json.dumps({"commits": rel}, indent=2) + "\n")
    for f in failed:
        print("FAILED " + f)


if __name__ == "__main__":
    main()
