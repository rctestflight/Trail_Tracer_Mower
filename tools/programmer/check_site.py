"""check_site.py - check the site folder before it is published; exit 1 on any finding.

Site layout: at the top only the page files; under builds/ only .json, .bin, .bin.encrypted and .commit_header
files (a build must not put .html, .js or .svg on the page's address); no dot names, no links, only regular
files. Text files: banned words, secrets, private paths and addresses, e-mail addresses. Binary files: the
same checks over their printable strings, and no Cortex-M vector table (a plain STM32 image under any
name). .bin.encrypted: entropy 7.9 bits per byte or more. No file over 5 MB, no banned path.

Usage: python tools/check_site.py <site folder>
"""
import json
import math
import os
import re
import struct
import sys
from pathlib import Path

# The word, path and name lists (banned words, banned folder names, private paths and addresses) are not
# in this public file: they come from the environment variable GATE_LISTS (a JSON object with BANNED_WORDS,
# BANNED_PATHS, PRIVATE: regular expressions), an Actions secret. With no lists the check fails.
try:
    _LISTS = json.loads(os.environ["GATE_LISTS"])
    BANNED_WORDS, BANNED_PATHS, PRIVATE = _LISTS["BANNED_WORDS"], _LISTS["BANNED_PATHS"], _LISTS["PRIVATE"]
except (KeyError, ValueError, TypeError):
    sys.exit("check_site: no GATE_LISTS (JSON with BANNED_WORDS, BANNED_PATHS, PRIVATE): nothing is published")
SECRET_PATTERNS = [
    r"-----BEGIN [A-Z ]*PRIVATE KEY-----",
    r"ghp_[A-Za-z0-9]{30,}", r"github_pat_[A-Za-z0-9_]{30,}", r"gho_[A-Za-z0-9]{30,}",
    r"AKIA[0-9A-Z]{16}", r"xox[abprs]-[A-Za-z0-9-]{10,}", r"sk-[A-Za-z0-9]{20,}",
    r"(?i)(password|passwd|secret|api[_-]?key|token)\s*[:=]\s*['\"][^'\"]{6,}['\"]",
    r"(?i)aes[_-]?key\s*[:=]", r"(?i)encryption[_-]?key\s*[:=]",
]
CLOSED_STM32 = r"(?i)(factory_iap|_swd|stm32.*\.(bin|hex|elf)$)"
ALLOWED_STM32 = r"(?i)\.bin\.encrypted$|\.commit_header$"
EMAIL = r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}"
TEXT = {".html", ".js", ".json", ".md", ".txt", ".css", ".yml", ".yaml", ".py", ".csv"}
MAX_FILE = 5 * 1024 * 1024
# Strings in the ESP-IDF and Arduino libraries that look like an e-mail address but are format text.
EMAIL_OK = r"(?i)@(example\.com|espressif\.com)$"


def strings(data, n=6):
    return [m.group(0).decode("ascii") for m in re.finditer(rb"[\x20-\x7e]{%d,}" % n, data)]


def texty(s):
    """A printable run from a binary that reads as text: 80 % or more letters, digits, spaces and punctuation
    marks of words. Machine code gives short runs of mixed signs; those are not text."""
    return len(re.findall(r"[A-Za-z0-9 .,:;_/()'-]", s)) >= 0.8 * len(s)


def word_case(w):
    """A word as text writes it: any case except a lower-case first letter with a capital after it. Machine code
    gives that mixed case as often as not; such a match in a binary is not a word."""
    letters = [c for c in w if c.isalpha()]
    return not (letters and letters[0].islower() and any(c.isupper() for c in letters[1:]))


def banned_in(line, binary):
    """A banned word in a text line; in a binary, only in a run that reads as text, in a case that text uses."""
    if not binary:
        return re.search(BANNED_WORDS, line, re.I) is not None
    return texty(line) and any(word_case(m.group(0)) for m in re.finditer(BANNED_WORDS, line, re.I))


def check_text(rel, text, findings, binary=False):
    """Findings name the file and the list only, never the matched text: the deploy log of a public repo is
    public, and a match would show an entry of the word lists."""
    if any(banned_in(line, binary) for line in (text.split("\n") if binary else [text])):
        findings.append(f"{rel}: banned word list")
    if any(re.search(p, text) for p in SECRET_PATTERNS):
        findings.append(f"{rel}: possible secret")
    if re.search(PRIVATE, text):
        findings.append(f"{rel}: private path or address list")
    if any(not re.search(EMAIL_OK, m) and not m.endswith("noreply.github.com") for m in re.findall(EMAIL, text)):
        findings.append(f"{rel}: e-mail address")


def entropy(data):
    """Shannon entropy in bits per byte: encrypted data is near 8, a plain firmware image near 6."""
    if not data:
        return 0.0
    counts = [0] * 256
    for x in data:
        counts[x] += 1
    return -sum(c / len(data) * math.log2(c / len(data)) for c in counts if c)


def vector_table(data):
    """Offset of a Cortex-M vector table, or None: a stack pointer word in SRAM (0x20000000 to 0x2003FFFF)
    and then 3 handler words in flash (0x08000000 to 0x080FFFFF) with bit 0 set (Thumb): reset, NMI, HardFault.
    4-byte aligned. (The next word of a Cortex-M0 table is reserved and can be 0.)"""
    for off in range(0, len(data) - 16, 4):
        sp, *vec = struct.unpack_from("<4I", data, off)
        if 0x20000000 <= sp < 0x20040000 and all(0x08000000 <= v < 0x08100000 and v & 1 for v in vec):
            return off
    return None


def check_encrypted(data):
    out = []
    e = entropy(data)
    if e < 7.9:
        out.append(f"entropy {e:.3f} bits per byte, under 7.9: not encrypted data")
    off = vector_table(data)
    if off is not None:
        out.append(f"ARM vector table at offset {off}: a plain STM32 image")
    return out


PAGE_FILES = {"index.html", "program.js", "icon-32.png", "receiver.jpg", "sources.json", "version.json", ".nojekyll"}
BUILD_FILE = r"^builds/([^/]+/)*[^/.][^/]*\.(json|bin|bin\.encrypted|commit_header)$"


def layout(rel, p):
    """Where a file may be, and what it may be."""
    if p.is_symlink() or not p.is_file():
        return f"not a regular file: {rel}"
    if "/" not in rel:
        return None if rel in PAGE_FILES else f"file not allowed at the top: {rel}"
    if any(part.startswith(".") for part in rel.split("/")):
        return f"dot name: {rel}"
    if not re.match(BUILD_FILE, rel):
        return f"file type or place not allowed (builds/ holds .json, .bin, .bin.encrypted, .commit_header): {rel}"
    return None


def main():
    site = Path(sys.argv[1])
    findings, n = [], 0
    for p in sorted(site.rglob("*")):
        rel = p.relative_to(site).as_posix()
        if rel == ".git" or rel.startswith(".git/"):
            continue                                  # the git data of the site folder (a worktree): not published
        if p.is_dir() and not p.is_symlink():
            continue
        n += 1
        bad = layout(rel, p)
        if bad:
            findings.append(bad)
            continue
        if re.search(BANNED_PATHS, rel):
            findings.append(f"path not allowed: {rel}")
        if re.search(BANNED_WORDS, rel, re.I):
            findings.append(f"banned word in path: {rel}")
        if re.search(CLOSED_STM32, p.name) and not re.search(ALLOWED_STM32, p.name):   # the file name, not the branch folder
            findings.append(f"plain STM32 image: {rel}")
        if p.stat().st_size > MAX_FILE:
            findings.append(f"file over 5 MB: {rel}")
        data = p.read_bytes()
        if p.suffix.lower() in TEXT:
            check_text(rel, data.decode("utf-8", errors="replace"), findings)
        elif rel.endswith(".bin.encrypted"):          # encrypted data has no text; a renamed plain image must fail
            findings += [f"{rel}: {w}" for w in check_encrypted(data)]
        else:
            check_text(rel + " (strings)", "\n".join(strings(data)), findings, binary=True)
        if not rel.endswith(".bin.encrypted"):        # every file, text too (check_encrypted does its own)
            off = vector_table(data)
            if off is not None:
                findings.append(f"{rel}: ARM vector table at offset {off}: a plain STM32 image")
        if rel.endswith(".commit_header") and (len(data) != 52 or data[:4] != b"SIAP"):
            findings.append(f"{rel}: not a 52-byte commit header")
    print(f"check_site {site}: {n} files, {len(findings)} findings")
    for f in findings[:200]:
        print("  " + f)
    sys.exit(1 if findings else 0)


if __name__ == "__main__":
    main()
