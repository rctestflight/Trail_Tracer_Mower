r"""Decoder for the rover telemetry log (docs/espnow_protocol.md, "Telemetry log").

Library use:  decode_frame(bytes) -> dict
Commands:
  decode  <file>         decode a file of lines that hold one frame in hex each ("LOG <hex>" from the
                         USB "log dump" command, or "R <us> <mac> <len> <hex>" from the monitor)
  usbdump <bench daemon url or COM port> [--stream S] [--from N] [--count C] [--out file]
                         ask the rover for "log dump" (through a bench daemon, or a COM port
                         directly: needs pyserial and tools/directport.py) and decode it
Writes one JSON object per line to stdout or --out. Python 3.8+, no packages beyond the standard library.
"""
import argparse
import json
import struct
import sys
import time
import urllib.request

T_LOG, T_REQ, T_INFO = 0x40, 0x41, 0x42
B_SAMPLES, B_STATUS, B_SAMPLES_IMU, B_EVENT = 0x20, 0x21, 0x22, 0x30
B_SAMPLES_DEC, B_SAMPLES_IMU_DEC = 0x24, 0x25  # 0x20 / 0x22 below 100 Hz: one more header byte, the rate in Hz
# Layout 3 (firmware 0.5.58): 50-byte samples (60 with the IMU axes); 0x2C / 0x2D below 100 Hz
B3_SAMPLES, B3_SAMPLES_IMU, B3_SAMPLES_DEC, B3_SAMPLES_IMU_DEC = 0x2A, 0x2B, 0x2C, 0x2D
# Firmware 0.5.66: coil magnitudes of all 8 bins, stream 0, one record each window (about 200 ms)
B_BINS = 0x2E
NO_DATA24 = -8388608  # 0x800000: the sensor is on no STM32 lane, or the bin is off
# Firmware 0.5.69: the vehicle name, stream 1, each 10 s and after a change
B_NAME = 0x31
NO_SEQ = 0xFFFFFFFF
AP_STATES = ["roam", "seek", "acquire", "follow", "depart", "halted"]
EVENTS = {1: "boot", 2: "mode", 3: "ap_state", 4: "stuck", 5: "charge_step", 6: "odrive_error",
          7: "halted", 8: "setting", 9: "stall", 10: "bad_key", 11: "run", 12: "imu", 13: "restart_refused", 14: "prev_boot", 15: "stm32", 16: "time", 17: "monitor",
          18: "stm32_cmd", 19: "wifi"}
# Event 18 (stm32_cmd), field a: lib/stm_settings/stm_settings.h, enum Kind
STM32_CMD_KINDS = {1: "no_answer", 2: "wrong_answer", 3: "read_back_differs", 4: "packet_gap", 5: "queue_full",
                   6: "hello_boot", 7: "stm_reset", 8: "refused", 9: "cal_restore"}


def bf16(h):
    """bfloat16 (top 16 bits of a float32) to float."""
    return struct.unpack("<f", struct.pack("<I", h << 16))[0]


def _none16(v, scale):
    return None if v == -32768 else v / scale


def decode_sample(b, block_ms):
    """One sample: 32 bytes (0x20), 42 (0x22, IMU axes at 32), 50 (layout 3) or 60 (layout 3, IMU axes at 50)."""
    (t, x, y, r, flags, ap, drive, steer, v0, v1, gz, bt1, bt2, c0, c1, c2, c3, xf) = struct.unpack(
        "<HhhHBBhhhhhBBHHHHh", b[:32])
    c0, c1, c2, c3 = (bf16(c) for c in (c0, c1, c2, c3))
    return {
        "ms": block_ms + t, "x1_mm": x, "y1_mm": y, "r1_mm": r,
        "lost": flags & 1, "overload": flags >> 1 & 1, "disabled": flags >> 2 & 1, "weak": flags >> 3 & 1,
        "entry_btx": flags >> 4 & 1, "exit_btx": flags >> 5 & 1, "charging": flags >> 6 & 1,
        "wheels_fresh": flags >> 7 & 1,
        "ap_state": None if ap == 0xFF else AP_STATES[ap >> 4] if (ap >> 4) < len(AP_STATES) else ap >> 4,
        "ap_sub": None if ap == 0xFF else ap & 0x0F,
        "drive": drive / 10000, "steer": steer / 10000, "vel0_rps": v0 / 1000, "vel1_rps": v1 / 1000,
        "gyro_dps": _none16(gz, 50), "btx1_mm": bt1 * 4, "btx2_mm": bt2 * 4,
        "coil": [c0, c1, c2, c3], "xf_mm": None if xf == -32768 else xf,
        **(_imu(b[32:42]) if len(b) in (42,) else {}),
        **(_layout3(b[32:50]) if len(b) >= 50 else {}),
        **(_imu(b[50:60]) if len(b) >= 60 else {}),
    }


def _layout3(b):
    """Layout 3, bytes 32-49: sensor 2 beacons, wheel positions and their ages, sensor 2 x/y, heading."""
    bt1, bt2, p0, p1, a0, a1, x2, y2, hd = struct.unpack("<BBiiBBhhH", b)
    return {"s2_btx1_mm": bt1 * 4, "s2_btx2_mm": bt2 * 4,
            "enc_pos_turn": [p0 / 10000, p1 / 10000], "enc_age_ms": [None if a0 == 255 else a0, None if a1 == 255 else a1],
            "x2_mm": None if x2 == -32768 else x2, "y2_mm": None if y2 == -32768 else y2,
            "heading_deg": hd * 360 / 65536}


def _imu(b):
    ax, ay, az, gx, gy = struct.unpack("<hhhhh", b)
    return {"ax_g": ax / 1000, "ay_g": ay / 1000, "az_g": az / 1000, "gx_dps": gx / 50, "gy_dps": gy / 50}


def decode_status(b):
    f = struct.unpack("<BBbBHHhHbBBBIIHHHHHHHHh6BIBBIHHhHB", b[:67])
    keys = ["body", "drive_mode", "charge_step", "vehicle_status", "vbatt_mv", "vcharge_mv", "icharge_ma",
            "v5v_mv", "temperature_c", "ap_state", "ap_sub", "flags", "status_bits", "charge_code",
            "crc_errors", "overruns", "packets_last_s", "loop_max_us", "bounces", "turnarounds", "charges",
            "odrive_vbus_mv", "odrive_ibus_ma"]
    d = dict(zip(keys, f[:23]))
    d["axis_state"] = list(f[23:29])
    rest = ["axis_errors_or", "error_nodes", "heartbeat_nodes", "free_heap", "send_fail", "backfill_last_s",
            "xf_mm", "dist_mm", "angle_deg"]
    d.update(dict(zip(rest, f[29:])))
    d["monitor_age_s"] = b[67] if len(b) >= 68 else None  # 0.5.21 and later; 255 = no monitor heard
    if len(b) >= 74:  # 0.5.22 and later: PC time at this record (0 = unknown)
        sec, ms = struct.unpack("<IH", b[68:74])
        d["pc_unix_ms"] = sec * 1000 + ms if sec else None
    d["fw_version"] = "%d.%d.%d" % tuple(b[74:77]) if len(b) >= 77 else None  # 0.5.24 and later
    if d["xf_mm"] == -32768:
        d["xf_mm"] = None
    fl = d["flags"]
    d.update({"go_home": fl & 1, "gyro": fl >> 1 & 1, "can": fl >> 2 & 1, "espnow": fl >> 3 & 1,
              "rover_enabled": fl >> 4 & 1, "locked": fl >> 5 & 1, "imu_fake": fl >> 6 & 1,
              "moving": fl >> 7 & 1})
    del d["body"]
    return d


def decode_event(b):
    code, a, bb = struct.unpack("<Hii", b[1:11])
    d = {"code": code, "event": EVENTS.get(code, "?"), "a": a, "b": bb,
         "text": b[11:].decode("ascii", "replace")}
    if code == 18:
        d["kind"] = STM32_CMD_KINDS.get(a, "?")
    return d


def decode_bins(b):
    """Body 0x2E: filter factor N (100 Hz / rate; firmware 0.5.66-0.5.67: frames averaged), output period ms
    (0.5.66-0.5.67: the window), 8 bin frequencies (Hz; None = bin off), and per bin the 4 coil values (sensor 1
    coil 1, coil 2, sensor 2 coil 1, coil 2) in STM32 magnitude LSB; None = no data."""
    n, per = b[1], struct.unpack_from("<H", b, 2)[0]
    hz = [f * 2 if f else None for f in struct.unpack_from("<8H", b, 4)]
    vals = [int.from_bytes(b[20 + 3 * i:23 + 3 * i], "little", signed=True) for i in range(32)]
    vals = [None if v == NO_DATA24 else v for v in vals]
    return {"filter_n": n, "period_ms": per, "hz": hz, "coils": [vals[4 * k:4 * k + 4] for k in range(8)]}


def decode_name(b):
    """Body 0x31: vehicle number and vehicle name."""
    n = b[2]
    return {"vehicle_id": b[1], "name": b[3:3 + n].decode("ascii", "replace")}


def decode_body(body, ms):
    if not body:
        return {"body": None}
    t = body[0]
    if t in (B_SAMPLES, B_SAMPLES_IMU, B_SAMPLES_DEC, B_SAMPLES_IMU_DEC,
             B3_SAMPLES, B3_SAMPLES_IMU, B3_SAMPLES_DEC, B3_SAMPLES_IMU_DEC):
        layout = 3 if t >= B3_SAMPLES else 1
        dec = t in (B_SAMPLES_DEC, B_SAMPLES_IMU_DEC, B3_SAMPLES_DEC, B3_SAMPLES_IMU_DEC)
        imu = t in (B_SAMPLES_IMU, B_SAMPLES_IMU_DEC, B3_SAMPLES_IMU, B3_SAMPLES_IMU_DEC)
        w = (60 if imu else 50) if layout == 3 else (42 if imu else 32)
        n, h = body[1], 3 if dec else 2
        return {"body": "samples", "layout": layout, "imu": imu, "rate_hz": body[2] if dec else 100,
                "samples": [decode_sample(body[h + w * i: h + w * (i + 1)], ms) for i in range(n)]}
    if t == B_STATUS:
        return {"body": "status", **decode_status(body)}
    if t == B_EVENT:
        return {"body": "event", **decode_event(body)}
    if t == B_NAME and len(body) >= 3:
        return {"body": "name", **decode_name(body)}
    if t == B_BINS and len(body) >= 116:
        return {"body": "bins", **decode_bins(body)}
    return {"body": "unknown 0x%02x" % t}


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
        d = {"type": "LOG", "stream": stream, "boot_id": boot, "seq": seq, "ms": ms, **decode_body(fr[14:], ms)}
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
    out = open(a.out, "w") if a.out else sys.stdout
    for src, fr in frames_from_lines(open(a.file, encoding="utf-8", errors="replace")):
        out.write(json.dumps({"src": src, **decode_frame(fr)}) + "\n")


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
    d.add_argument("--out")
    u = sub.add_parser("usbdump")
    u.add_argument("url")
    u.add_argument("--stream", type=int, default=1)
    u.add_argument("--from", dest="frm", type=int, default=0)
    u.add_argument("--count", type=int, default=1000000)
    u.add_argument("--out")
    a = ap.parse_args()
    {"decode": cmd_decode, "usbdump": cmd_usbdump}[a.cmd](a)


if __name__ == "__main__":
    main()
