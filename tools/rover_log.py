r"""Decoder for the rover telemetry log (docs/espnow_protocol.md, "Telemetry log" and "Telemetry definition").

Every record is decoded from telemetry_def.json (tools/gen_telemetry_def.py, from include/telemetry_table.h): its fields,
types, offsets, scales and "no value" codes. Only the sweep readings (bodies 0x32 and 0x33, "special" in the file) have
code here. Which file: the table records (bodies 0x34 to 0x36) carry the table hash and use the file of that "version";
the other records use the file of the hash that the same boot's table records named, else the first file loaded. With
no --def, the file next to this script or docs/telemetry_def.json (the default build) is loaded.

Library use:  load_defs([paths]) (optional), then decode_frame(bytes) -> dict
Commands:
  decode  <file> [--def telemetry_def.json ...]
                         decode a file of lines that hold one frame in hex each ("LOG <hex>" from the
                         USB "log dump" command, or "R <us> <mac> <len> <hex>" from the monitor)
  csv     <file> --def telemetry_def.json [--def ...] [--out file.csv]
                         the table values (bodies 0x34, 0x35, 0x36) as CSV: one row per sample, one column per
                         value; the definition file of each build is chosen by the table hash in the record
  usbdump <bench daemon url or COM port> [--stream S] [--from N] [--count C] [--out file]
                         ask the rover for "log dump" (through a bench daemon, or a COM port
                         directly: needs pyserial and tools/directport.py) and decode it
Writes one JSON object per line to stdout or --out. Python 3.8+, no packages beyond the standard library.
"""
import argparse
import json
import os
import struct
import sys
import time
import urllib.request

T_LOG, T_REQ, T_INFO = 0x40, 0x41, 0x42
B_SWEEP, B_SWEEP_RAW = 0x32, 0x33  # "special" records: the readings after the header are decoded here
B_POLAR = 0x37                     # polarity detector blocks (Rx STM32 2.0.67+): decoded here
TLM_BODIES = (0x34, 0x35, 0x36)    # table records: u32 table hash right after the body type
NO_DATA24 = -8388608  # 0x800000: the sensor is on no STM32 lane
NO_SEQ = 0xFFFFFFFF
DEF_FORMAT = 2
DEFS = {}       # table hash (int) -> telemetry_def.json content
DEF_ORDER = []  # table hashes in load order; the first decodes the fixed records of a boot with no table record
BOOT_DEF = {}   # boot_id -> table hash named by a table record of that boot
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DEFS = [os.path.join(HERE, "telemetry_def.json"), os.path.join(HERE, "..", "docs", "telemetry_def.json")]


def bf16(h):
    """bfloat16 (top 16 bits of a float32) to float."""
    return struct.unpack("<f", struct.pack("<I", h << 16))[0]


def load_defs(paths):
    """Reads telemetry_def.json files into DEFS, by their "version" (the table hash)."""
    for p in paths or []:
        d = json.load(open(p, encoding="utf-8"))
        if d.get("format") != "telemetry_def" or d.get("format_version") != DEF_FORMAT:
            raise ValueError("%s: not a telemetry_def.json of format version %d" % (p, DEF_FORMAT))
        h = int(d["version"], 16)
        DEFS[h] = d
        if h not in DEF_ORDER:
            DEF_ORDER.append(h)
    return DEFS


def _ensure_def():
    if DEF_ORDER:
        return
    for p in DEFAULT_DEFS:
        if os.path.exists(p):
            load_defs([p])
            return
    raise FileNotFoundError("no telemetry_def.json: give --def, or put the file next to rover_log.py")


def _fixed_def(boot):
    h = BOOT_DEF.get(boot)
    if h in DEFS:
        return DEFS[h]
    _ensure_def()
    return DEFS[DEF_ORDER[0]]


_SIZE = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "u24": 3, "i24": 3, "u32": 4, "i32": 4, "f32": 4, "bf16": 2,
         "version": 3, "unix_ms": 6}


def _read(f, b, o, end):
    """The wire value of field f at byte o of b (fields end at end); None past the end."""
    t = f["type"]
    if t == "text":
        if "length_offset" in f:
            lo = o - f["offset"] + f["length_offset"]
            if lo >= end:
                return None
            return b[o:min(o + b[lo], end)].decode("ascii", "replace")
        return b[o:end].decode("ascii", "replace") if o <= end else None
    n = _SIZE[t]
    if o + n > end:
        return None
    if t == "f32":
        return struct.unpack_from("<f", b, o)[0]
    if t == "bf16":
        return bf16(struct.unpack_from("<H", b, o)[0])
    if t == "version":
        return "%d.%d.%d" % tuple(b[o:o + 3])
    if t == "unix_ms":
        return struct.unpack_from("<IH", b, o)
    return int.from_bytes(b[o:o + n], "little", signed=t[0] == "i")


def _scaled(x, s):
    """x times the scale; a scale of 1/k (k whole) divides by k, so 7 x 0.0001 gives 0.0007 as 7 / 10000 does."""
    if s == 1:
        return x
    if float(s).is_integer():
        return x * int(s)
    inv = 1.0 / s
    k = round(inv)
    if abs(inv - k) < 1e-9 * k:
        return x / k
    return x * s


def _value(f, raw, ms):
    if raw is None:
        return None
    t, none = f["type"], f.get("none")
    if t in ("text", "version"):
        return raw
    if t == "unix_ms":
        sec, msec = raw
        return None if none is not None and sec == none else sec * 1000 + msec
    if t in ("f32", "bf16"):
        if raw != raw:
            return None if none == "NaN" else raw
        return _scaled(raw, f["scale"])
    if none is not None and raw == none:
        return None
    x = raw
    if "shift" in f:
        x >>= f["shift"]
    if "mask" in f:
        x &= f["mask"]
    k = f["kind"]
    if k == "time":
        return ms + x
    if k == "hex":
        return "%08x" % x
    if k == "enum":
        nm = f["names"].get(str(x))
        return nm if nm is not None else f.get("unknown", x)
    return _scaled(x, f["scale"])


def _put(out, name, idx, val):
    lst = out.setdefault(name, [])
    for j, i in enumerate(idx):
        last = j == len(idx) - 1
        while len(lst) <= i:
            lst.append(None if last else [])
        if last:
            lst[i] = val
        else:
            lst = lst[i]


def _fields(fields, b, base, end, ms, out):
    for f in fields:
        w = f.get("when")
        if w and out.get(w[0]) != w[1]:
            continue
        raw = _read(f, b, base + f["offset"], end)
        if f["kind"] == "bits":
            for i, nm in enumerate(f["bits"]):
                if nm:
                    out[nm] = None if raw is None else raw >> i & 1
            continue
        val = _value(f, raw, ms)
        if "list" in f:
            _put(out, f["list"], f["index"], val)
        else:
            out[f["name"]] = val


def decode_record(rec, body, ms):
    """One record body from its telemetry_def.json entry. Offsets count from the byte after the body type."""
    b = body[1:]
    if len(b) < rec["min_bytes"]:
        if rec["name"] == "tlm":
            return {"body": "tlm", "error": "record shorter than its header"}
        return {"body": "unknown 0x%02x" % body[0]}
    d = {"body": rec["name"]}
    d.update(rec.get("const", {}))
    _fields(rec["fields"], b, 0, len(b), ms, d)
    rp = rec.get("repeat")
    if rp:
        n, w, o0 = b[rp["count_offset"]], rp["stride"], rp["offset"]
        step = None
        if "rate_offset" in rp:
            hz = b[rp["rate_offset"]]
            if hz < 1:
                d["error"] = "rate 0"
                return d
            step = 1000 // hz
        if len(b) != o0 + n * w:
            d["error"] = "length %d, not %d + %d x %d" % (len(body), 1 + o0, n, w)
            return d
        out = []
        for i in range(n):
            s = {} if step is None else {"ms": ms + i * step}
            _fields(rp["fields"], b, o0 + i * w, o0 + (i + 1) * w, ms, s)
            out.append(s)
        d[rp["name"]] = out
    return d


def decode_sweep(b):
    """Body 0x32: table id (a change starts a new trace), f0 and step (Hz), grid points P, sweep bins n, and per update
    the n readings: grid point, its frequency (Hz) and the 4 coil values (sensor 1 coil 1, coil 2, sensor 2 coil 1,
    coil 2) in STM32 magnitude LSB; None = the sensor is on no lane."""
    f0 = int.from_bytes(b[2:5], "little")
    step, pts, n, u = struct.unpack_from("<HH", b, 5) + (b[9], b[10])
    ups, o = [], 11
    for _ in range(u):
        rd = []
        for _ in range(n):
            g = struct.unpack_from("<H", b, o)[0]
            vals = [int.from_bytes(b[o + 2 + 3 * c:o + 5 + 3 * c], "little", signed=True) for c in range(4)]
            rd.append({"point": g, "hz": (f0 + g * step) / 10.0, "coils": [None if v == NO_DATA24 else v for v in vals]})
            o += 14
        ups.append(rd)
    return {"table_id": b[1], "f0_hz": f0 / 10.0, "step_hz": step / 10.0, "points": pts, "sweep_bins": n, "updates": ups}


def decode_sweep_raw(b):
    """Body 0x33: table id, f0 and step (Hz), grid points P, sweep bins n, bin mask, and per frame: seq, page (the frame
    holds the sweep page), overrun, tag and previous-frame tag (None without the page), and per sweep bin the 4 coil
    values (sensor 1 coil 1, coil 2, sensor 2 coil 1, coil 2) in STM32 magnitude LSB; None = the sensor is on no lane.
    Each reading gets its grid point and frequency from sweep_points()."""
    f0 = int.from_bytes(b[2:5], "little")
    step, pts = struct.unpack_from("<HH", b, 5)
    n, mask, u = b[9], b[10], b[11]
    frames, o = [], 12
    for _ in range(u):
        seq, fl, tag, prev = struct.unpack_from("<BBHH", b, o)
        o += 6
        rd = []
        for _ in range(n):
            vals = [int.from_bytes(b[o + 3 * c:o + 3 + 3 * c], "little", signed=True) for c in range(4)]
            rd.append({"coils": [None if v == NO_DATA24 else v for v in vals]})
            o += 12
        page = bool(fl & 1)
        frames.append({"seq": seq, "page": page, "overrun": bool(fl & 2), "tag": tag if page else None,
                       "tag_prev": prev if page else None, "readings": rd})
    rec = {"table_id": b[1], "f0_hz": f0 / 10.0, "step_hz": step / 10.0, "points": pts, "sweep_bins": n,
           "mask": mask, "frames": frames}
    sweep_points(rec)
    return rec


def decode_polar(b):
    """Body 0x37: per frame the frame seq and the Rx STM32 detector block: n samples, detector, samples lost (flag),
    gap count, output index of the first sample, input index after the last housekeeping gap, n complex samples
    (re, im int16). None if the record is cut."""
    u, o, frames = b[1], 2, []
    for _ in range(u):
        if o + 9 > len(b):
            return None
        seq, n, fl, gaps, oi, ii = struct.unpack_from("<BBBHHH", b, o)
        n = min(n, 10)
        o += 9
        if o + 4 * n > len(b):
            return None
        iq = struct.unpack_from("<%dh" % (2 * n), b, o)
        o += 4 * n
        frames.append({"seq": seq, "n": n, "det": fl & 7, "lost": bool(fl & 0x80), "gaps": gaps, "out_index": oi,
                       "in_index": ii, "iq": [[iq[2 * i], iq[2 * i + 1]] for i in range(n)]})
    return {"frames": frames}


def sweep_points(rec):
    """Grid point and frequency of each reading in a decoded 0x33 record (PC side; Rx STM32 rules):
    tag step per frame s = tag - previous-frame tag (mod P) on a page frame.
    s = n > 1 (Rx STM32 2.0.57+, block sweep): bin i reads grid point (tag + i) mod P.
    s = 1 (2.0.51 - 2.0.56, spread sweep): bin i reads grid point (tag + i x P / n) mod P.
    A frame without the page takes its tag from the nearest page frame of the record by seq: page tag + s x (seq
    difference). Frames with no page frame in the record, or with the overrun bit, get point None."""
    P, n, fr = rec["points"], rec["sweep_bins"], rec["frames"]
    pages = [f for f in fr if f["page"] and f["tag"] is not None and f["tag"] < P]
    s = (pages[0]["tag"] - pages[0]["tag_prev"]) % P if pages and P else None
    for f in fr:
        tag = None
        if s is not None and not f["overrun"]:
            ref = min(pages, key=lambda q: abs(((f["seq"] - q["seq"] + 128) % 256) - 128))
            d = ((f["seq"] - ref["seq"] + 128) % 256) - 128
            tag = (ref["tag"] + s * d) % P
        f["tag_est"] = tag
        for i, r in enumerate(f["readings"]):
            if tag is None:
                r["point"] = r["hz"] = None
                continue
            g = (tag + i) % P if (n > 1 and s == n) else (tag + i * P // n) % P
            r["point"] = g
            r["hz"] = (rec["f0_hz"] * 10 + g * rec["step_hz"] * 10) / 10.0
    rec["tag_step"] = s
    return rec


def decode_body(body, ms, boot=None):
    if not body:
        return {"body": None}
    t = body[0]
    if t in TLM_BODIES:
        _ensure_def()
        if len(body) < 5:
            return {"body": "tlm", "error": "record shorter than its header"}
        h = struct.unpack_from("<I", body, 1)[0]
        df = DEFS.get(h)
        if df is None:
            return {"body": "tlm", "hash": "%08x" % h, "error": "unknown definition hash %08x" % h}
        if boot is not None:
            BOOT_DEF[boot] = h
        rec = df["records"].get("0x%02x" % t)
        if rec is None:
            return {"body": "tlm", "hash": "%08x" % h, "error": "body 0x%02x not in definition %08x" % (t, h)}
        return decode_record(rec, body, ms)
    rec = _fixed_def(boot)["records"].get("0x%02x" % t)
    if rec is None:
        return {"body": "unknown 0x%02x" % t}
    if rec.get("special"):
        if t == B_SWEEP and len(body) >= 11 and len(body) >= 11 + 14 * body[9] * body[10]:
            return {"body": "sweep", **decode_sweep(body)}
        if t == B_SWEEP_RAW and len(body) >= 12 and len(body) >= 12 + (6 + 12 * body[9]) * body[11]:
            return {"body": "sweep_raw", **decode_sweep_raw(body)}
        if t == B_POLAR and len(body) >= 2:
            d = decode_polar(body)
            if d is not None:
                return {"body": "polar", **d}
        return {"body": "unknown 0x%02x" % t}
    return decode_record(rec, body, ms)


# Firmware 0.5.45+ with a layout 2 STM32 (status bit 24, run): the 4 coil fields are magnitudes, sensor 1
# coil 1, sensor 1 coil 2, sensor 2 coil 1, sensor 2 coil 2, at each sensor's wire bin. Known per boot from
# its last status record; the samples then also carry "coil_mag".
COIL_MAG = {}


def _coil_mag(d):
    fw = tuple(int(x) for x in d["fw_version"].split(".")) if d.get("fw_version") else (0, 0, 0)
    return fw >= (0, 5, 45) and bool(d["status_bits"] >> 24 & 1)


def decode_frame(fr):
    t = fr[0]
    if t == T_LOG:
        stream, boot, seq, ms = struct.unpack("<BIII", fr[1:14])
        d = {"type": "LOG", "stream": stream, "boot_id": boot, "seq": seq, "ms": ms, **decode_body(fr[14:], ms, boot)}
        if d["body"] == "status":
            COIL_MAG[boot] = _coil_mag(d)
        elif d["body"] == "samples" and COIL_MAG.get(boot):
            for smp in d["samples"]:
                smp["coil_mag"] = list(smp["coil"])
        return d
    if t == T_INFO and len(fr) >= 41:
        v = struct.unpack("<II8I", fr[1:41])

        def rng(o, n):
            return None if o == NO_SEQ else [o, n]
        return {"type": "INFO", "boot_id": v[0], "prev_boot_id": v[1],
                "fast": rng(v[2], v[3]), "slow": rng(v[4], v[5]), "fast_prev": rng(v[6], v[7]),
                "slow_prev": rng(v[8], v[9])}
    if t == T_REQ and len(fr) >= 18:
        stream, boot, frm, mask = struct.unpack("<BIIQ", fr[1:18])
        return {"type": "REQ", "stream": stream, "boot_id": boot, "from_seq": frm, "mask": mask}
    if t == 0x2D:
        return {"type": "TEXT", "text": fr.decode("ascii", "replace")}
    return {"type": "0x%02x" % t, "len": len(fr)}


def frames_from_lines(lines):
    """Yields (source, bytes) from "LOG <hex>" lines and monitor "R <us> <mac> <len> <hex>" lines."""
    for ln in lines:
        p = ln.split()
        try:  # "LOG end", "LOG off ..." and other text lines are not frames
            if len(p) == 2 and p[0] == "LOG":
                yield "usb", bytes.fromhex(p[1])
            elif len(p) == 5 and p[0] == "R":
                yield p[2], bytes.fromhex(p[4])
        except ValueError:
            continue


def cmd_decode(a):
    load_defs(a.defs)
    out = open(a.out, "w") if a.out else sys.stdout
    for src, fr in frames_from_lines(open(a.file, encoding="utf-8", errors="replace")):
        out.write(json.dumps({"src": src, **decode_frame(fr)}) + "\n")


def cmd_csv(a):
    """Table values as CSV: src, boot_id, stream, seq, rate, ms, then one column per value name (every name of the
    loaded definitions, in file order; empty where a row has no such value or the value is "no value")."""
    load_defs(a.defs)
    names = []
    for h in DEF_ORDER:
        for r in DEFS[h]["records"].values():
            if r["name"] == "tlm":
                names += [f["name"] for f in r["repeat"]["fields"] if f["name"] not in names]
    out = open(a.out, "w", newline="") if a.out else sys.stdout
    import csv
    wr = csv.writer(out)
    wr.writerow(["src", "boot_id", "stream", "seq", "rate", "ms"] + names)
    rows = errors = 0
    for src, fr in frames_from_lines(open(a.file, encoding="utf-8", errors="replace")):
        d = decode_frame(fr)
        if d.get("body") != "tlm":
            continue
        if "error" in d:
            errors += 1
            print("record %s/%s/%s: %s" % (d["boot_id"], d["stream"], d["seq"], d["error"]), file=sys.stderr)
            continue
        for s in d["samples"]:
            wr.writerow([src, d["boot_id"], d["stream"], d["seq"], d["rate"], s["ms"]]
                        + ["" if s.get(k) is None else s[k] for k in names])
            rows += 1
    print("%d rows, %d table records not decoded" % (rows, errors), file=sys.stderr)
    return 1 if errors else 0


def cmd_polar(a):
    """Polarity detector samples (body 0x37) as CSV, one row per sample: src, boot_id, record seq, record ms, frame seq,
    detector, samples lost flag, gap count, output index of the frame's first sample, input index after the last
    housekeeping gap, sample number in the frame, re, im. Rows come in record order (the PC sorts by output index)."""
    out = open(a.out, "w", newline="") if a.out else sys.stdout
    import csv
    wr = csv.writer(out)
    wr.writerow(["src", "boot_id", "rec_seq", "ms", "frame_seq", "det", "lost", "gaps", "out_index", "in_index", "k", "re", "im"])
    rows = recs = 0
    for src, fr in frames_from_lines(open(a.file, encoding="utf-8", errors="replace")):
        d = decode_frame(fr)
        if d.get("body") != "polar":
            continue
        recs += 1
        for f in d["frames"]:
            for k, (re_, im_) in enumerate(f["iq"]):
                wr.writerow([src, d.get("boot_id"), d.get("seq"), d.get("ms"), f["seq"], f["det"], int(f["lost"]), f["gaps"],
                             f["out_index"], f["in_index"], k, re_, im_])
                rows += 1
    print("%d samples from %d records" % (rows, recs), file=sys.stderr)
    return 0


def cmd_usbdump(a):
    if a.url.upper().startswith("COM"):
        # A PC with no bench daemon: the COM port directly (directport.py, needs pyserial).
        from directport import DirectPort
        port = DirectPort(a.url.upper())
        t0 = time.time()
        while port.api("/status")["state"] != "open" and time.time() - t0 < 10:
            time.sleep(0.2)
        get = port.api
        send = lambda text: port.api("/write?nl=1", text.encode())  # noqa: E731
    else:
        url = a.url.rstrip("/")

        def get(path):
            with urllib.request.urlopen(url + path, timeout=5) as r:
                return json.loads(r.read())

        def send(text):
            urllib.request.urlopen(urllib.request.Request(url + "/write?nl=1", data=text.encode()), timeout=5).read()
    start = get("/status")["next"]
    send(f"log dump {a.stream} {a.frm} {a.count}")
    out = open(a.out, "w") if a.out else sys.stdout
    nxt, idle, n = start, 0.0, 0
    while idle < 3.0:
        got = get(f"/lines?since={nxt}&max=500")
        nxt = got["next"]
        if not got["lines"]:
            time.sleep(0.2)
            idle += 0.2
            continue
        idle = 0.0
        for _, _, text in got["lines"]:
            for src, fr in frames_from_lines([text]):
                out.write(json.dumps({"src": src, **decode_frame(fr)}) + "\n")
                n += 1
            if text.startswith("LOG end"):
                idle = 99
    print(f"{n} records", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("decode")
    d.add_argument("file")
    d.add_argument("--def", dest="defs", action="append", default=[], help="telemetry_def.json (table values)")
    d.add_argument("--out")
    c = sub.add_parser("csv")
    c.add_argument("file")
    c.add_argument("--def", dest="defs", action="append", required=True, help="telemetry_def.json (one per build)")
    c.add_argument("--out")
    pl = sub.add_parser("polar")
    pl.add_argument("file")
    pl.add_argument("--out")
    u = sub.add_parser("usbdump")
    u.add_argument("url")
    u.add_argument("--stream", type=int, default=1)
    u.add_argument("--from", dest="frm", type=int, default=0)
    u.add_argument("--count", type=int, default=1000000)
    u.add_argument("--out")
    a = ap.parse_args()
    sys.exit({"decode": cmd_decode, "csv": cmd_csv, "polar": cmd_polar, "usbdump": cmd_usbdump}[a.cmd](a))


if __name__ == "__main__":
    main()
