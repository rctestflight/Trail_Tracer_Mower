r"""Rx data over USB, for the PC and the equipment helpers: time-domain samples, polarity, and the magnitude and
phase of the frequency bins, from any Rx board by its COM port.

Port rules (tools/directport.py): one open per call, no retry when the open fails, close with RTS off then DTR off,
and the port is closed when the call returns (also on an error).

The firmware commands it uses (no firmware change):
  boot, get rx_stm32_version        the versions
  stm pol (ESP32 0.6.21+)           per sensor x, y, radius; per Tx channel the field sign, polarity and sign spread
  stm bins                          per lane and bin the two values of the last frame: coil 1 and coil 2 signed
                                    magnitude (a bin in magnitude mode), or x and y (a bin in xy mode)
  get dsp_bin_hz_<i>_eff, get dsp_xy_mask, bin <i> <hz>
                                    the bin frequencies in effect, the xy mode mask, a bin frequency (not saved: the
                                    next settings change or Rx restart sends the dsp_bin_hz settings again)
  stm raw <mask> <shift> <s>        the raw sample stream (RAW_STREAM_INTERFACE.md in the Rx STM32 repo): complex
                                    samples per coil at 27 397.26 per second, the coil signal at 54 794.52 Hz + f

Phase: the frame gives each bin a signed magnitude per coil (a sign, no phase). The phase comes from the samples on
the PC: per gap-free stretch (a stretch starts after each housekeeping gap or sequence break and drops its 4 settling
samples, as tools/raw_check.py in RxPortd), a Hann-window DFT at the bin frequency. The phase of a coil is given
against the first coil of the capture (the start of a capture has no fixed time against the Tx). The amplitude is
the peak of the tone in sample units (z) and in ADC counts (z / 10.7).

CLI (prints one JSON object; exit 0 when the Rx answered, 1 on a refusal or a port error):
  python rx_usb.py COM23 info
  python rx_usb.py COM23 polarity
  python rx_usb.py COM23 bins [--bins 0,1,5]                       firmware bin values, no stream
  python rx_usb.py COM23 bins --phase [--sensor 1] [--seconds 1]   also magnitude and phase from the samples
  python rx_usb.py COM23 pga [--hold -1|0|1|2]                     PGA step per lane; hold = CID 29, RAM only
  python rx_usb.py COM23 set-bin <index 0-7> <hz 46800-62800 | 0>
  python rx_usb.py COM23 samples [--sensor 1|2] [--mask 1|2|3|4|8|12] [--shift 0-9] [--seconds 1-600]
                                 [--out capture.npz] [--freqs 53333.3,55315.5]

Python:
  import rx_usb
  cap = rx_usb.samples("COM23", sensor=1, seconds=2)        # cap["iq"]: complex64 array [coil, sample]
  rx_usb.tones(cap, [53333.3])                              # amplitude and phase per coil
  rx_usb.polarity("COM23"); rx_usb.bins("COM23", phase=True); rx_usb.set_bin("COM23", 7, 50000); rx_usb.pga("COM23", hold=1)
  with rx_usb.Rx("COM23") as rx: rx.bins(); rx.polarity()   # several reads in one open
"""
import argparse
import json
import re
import struct
import sys
import time

import numpy as np
import serial
from serial.tools import list_ports

FS = 64e6 / 146 / 16  # 27 397.26 complex samples a second per coil
MIX = 64e6 / 146 / 8  # 54 794.52 Hz at the coil for a sample at 0 Hz
Z_PER_COUNT = 10.7  # |z| per ADC count of input amplitude (peak), at the PGA step in effect
SETTLE = 4  # samples dropped after each gap
COIL_NAMES = ["s1c1", "s1c2", "s2c1", "s2c2"]  # stream rank 0-3: sensor 1 coils 1, 2; sensor 2 coils 1, 2
SENSOR_MASK = {1: 3, 2: 12}
BINS = 8


class RxError(Exception):
    """The Rx refused a command, or gave no answer."""


def _crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x8005) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class Rx:
    """One open of an Rx COM port. Use it with "with": the port closes at the end of the block."""

    def __init__(self, port, wait_s=0.0):
        self.port = port
        self.ser = None
        self.wait_s = wait_s

    def __enter__(self):
        t0 = time.time()
        while not any(p.device == self.port for p in list_ports.comports()):  # a port that just came back: 2 s first
            if time.time() - t0 > self.wait_s:
                raise RxError(f"{self.port} is not present")
            time.sleep(0.2)
        if time.time() - t0 > 0.1:
            time.sleep(2.0)
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05, write_timeout=1)
        except serial.SerialException as e:  # one open only: no retry
            raise RxError(f"open of {self.port} failed (held by another program?): {e}") from None
        return self

    def __exit__(self, *exc):
        if self.ser is not None:
            for attr in ("rts", "dtr"):
                try:
                    setattr(self.ser, attr, False)
                except Exception:
                    pass
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        return False

    # Text commands ------------------------------------------------------------------------------------------------
    def _send(self, text):
        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode())

    def ask(self, text, want, timeout=2.0, count=1, stop=None):
        """Send one line; return the lines that start with want (a prefix or a tuple of them), count of them, or
        until a line starts with stop."""
        self._send(text)
        out, buf, t0 = [], b"", time.time()
        while time.time() - t0 < timeout:
            buf += self.ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                s = raw.rstrip(b"\r").decode("utf-8", "replace")
                if s.startswith(("STM usage", "STM raw usage", "STM refused", "STM raw refused", "RAW refused")):
                    raise RxError(f"{text}: {s}")
                if s.startswith(want):
                    out.append(s)
                    if (stop is None and len(out) >= count) or (stop is not None and s.startswith(stop)):
                        return out
        if not out:
            raise RxError(f"{text}: no answer in {timeout} s")
        return out

    def cid_answer(self, text, tag, timeout=2.0):
        """Send a "stm cmd" or "stm read" line; return the "STM ans" line that contains tag (e.g. "cid=29r"). The
        answers to the ESP32's own Rx STM32 reads (other CIDs) come in the same lines and are skipped."""
        self._send(text)
        buf, t0 = b"", time.time()
        while time.time() - t0 < timeout:
            buf += self.ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                s = raw.rstrip(b"\r").decode("utf-8", "replace")
                if s.startswith("STM usage"):
                    raise RxError(f"{text}: {s}")
                if s.startswith("STM ans seq=") and f" {tag} " in s:
                    return s
        raise RxError(f"{text}: no {tag} answer in {timeout} s")

    def get(self, key, timeout=2.0):
        """A setting or value by "get <key>": the text after '=' ('' when empty)."""
        s = self.ask(f"get {key}", f"CFG ", timeout)[0]
        m = re.match(r"CFG (\d+) (\S+?)=(.*)$", s)
        if not m or m.group(2) != key:
            raise RxError(f"get {key}: {s}")
        if m.group(1) != "0":
            raise RxError(f"get {key}: result {m.group(1)}: {m.group(3)}")
        return m.group(3).strip()

    def info(self):
        b = self.ask("boot", "BOOT rover-firmware")[0]
        f = dict(kv.split("=", 1) for kv in b.split()[3:] if "=" in kv)
        return {"port": self.port, "esp32": b.split()[2], "rx_stm32": self.get("rx_stm32_version"),
                "mac": f.get("mac"), "board": f.get("board")}

    def polarity(self):
        """Per sensor: lost flag, x, y, radius (mm as the Rx gives them); per Tx channel 1-5: field sign, polarity
        (+1 or -1, 0 = none yet) and sign spread (deg; None when the Rx has sent none). Also the followed channel and
        its inside and heading facts. Needs ESP32 0.6.21+."""
        lines = self.ask("stm pol", "STM pol", count=3)
        head = dict(kv.split("=") for kv in lines[0].split()[2:])
        out = {"age_ms": int(head["age_ms"]), "channel": int(head["ch"]), "inside": int(head["inside"]),
               "heading": int(head["heading"]), "sensors": {}}
        for s in lines[1:]:
            p = s.split()
            sens = {k: float(v) for k, v in (kv.split("=") for kv in p[3:7])}
            sens["lost"] = bool(int(sens["lost"]))
            ch = {}
            for item in p[7:]:
                name, vals = item.split(":")
                sign, pol, spread = vals.split("/")
                ch[name] = {"sign": int(sign), "polarity": int(pol), "spread_deg": None if float(spread) < 0 else float(spread)}
            sens["channels"] = ch
            out["sensors"][p[2]] = sens
        return out

    def bin_setup(self):
        """The bin frequencies in effect (Hz, 0 = the sweep, None = not yet answered) and the mode of each bin."""
        mask = int(float(self.get("dsp_xy_mask") or 0))
        hz = []
        for i in range(BINS):
            v = self.get(f"dsp_bin_hz_{i}_eff")
            hz.append(float(v) if v else None)
        return [{"bin": i, "hz": hz[i], "mode": "xy" if mask >> i & 1 else "magnitude"} for i in range(BINS)]

    def bins(self, which=None, phase=False, sensor=1, seconds=1, mask=None):
        """Firmware bin values of the last frame for the bins in which (default all 8). Magnitude mode: coil 1 and
        coil 2 signed magnitude (about 1/mm for a wire bin at unit gain); xy mode: x and y in mm. With phase: one
        capture of seconds and, per bin that is on, the amplitude and phase per coil (tones())."""
        which = list(range(BINS)) if which is None else list(which)
        setup = self.bin_setup()
        lanes = self.ask("stm bins", "STMBINS", count=2)
        vals = {}
        for s in lanes:
            p = s.split()
            vals[p[1]] = [tuple(int(v) for v in x.split(",")) for x in p[2:]]
        out = {"bins": []}
        for i in which:
            e = dict(setup[i])
            for lane, v in vals.items():
                a, b = v[i]
                if e["mode"] == "xy":
                    e[lane] = {"x_mm": a * 0.001, "y_mm": b * 0.001}
                else:
                    e[lane] = {"coil1": a / 2 ** 24, "coil2": b / 2 ** 24}
            out["bins"].append(e)
        if phase:
            freqs = [e["hz"] for e in out["bins"] if e["hz"]]
            cap = self.samples(sensor=sensor, seconds=seconds, mask=mask)
            out["capture"] = summary(cap)
            out["tones"] = tones(cap, freqs)
        return out

    def set_bin(self, index, hz, timeout=3.0):
        """Bin index 0-7 to hz (46 800-62 800 Hz; 0 = back to its default use). Not saved. Returns the frequency
        in effect after the Rx answers. The wire bin of each sensor is refused."""
        s = self.ask(f"bin {int(index)} {float(hz):.1f}", "CFG ")[0]
        if not s.startswith("CFG 0 "):
            raise RxError(f"bin {index} {hz}: {s}")
        t0 = time.time()
        while time.time() - t0 < timeout:
            v = self.get(f"dsp_bin_hz_{int(index)}_eff")
            if v and (hz == 0 or abs(float(v) - hz) < 0.5):
                return {"bin": int(index), "hz": float(v)}
            time.sleep(0.2)
        raise RxError(f"bin {index}: the Rx did not confirm {hz} Hz in {timeout} s (in effect: {v or 'none'})")

    def pga(self, hold=None):
        """The PGA step in effect per lane, from the frame status (bits 2-3 lane 1, bits 7-8 lane 2, pin code 3 = step 2), and the hold
        (Rx STM32 CID 29: -1 auto, 0-2 a fixed step, one value for both sensors). hold given: write CID 29 first.
        CID 29 is RAM only (the Rx STM32 writes flash only on CID 3); an Rx STM32 restart returns it to -1 (auto)."""
        if hold is not None:
            if int(hold) not in (-1, 0, 1, 2):
                raise RxError(f"PGA hold {hold}: -1 (auto), 0, 1 or 2")
            s = self.cid_answer(f"stm cmd 29 {int(hold)} 0", "cid=29w")
            if "cid=29w" not in s or "status=1" not in s:
                raise RxError(f"PGA hold {hold}: {s}")
            time.sleep(0.3)
        s = self.cid_answer("stm read 29 0", "cid=29r")
        m = re.search(r"cid=29r .*status=1 value=(-?\d+)", s)
        if not m:
            raise RxError(f"PGA hold read: {s}")
        st = re.search(r"status=0x([0-9a-f]+)", self.ask("stm v2", "STMV2 ver=", count=1)[0])
        status = int(st.group(1), 16)
        step = {0: 0, 1: 1, 3: 2}  # the status bits copy the two PGA pins: step 0 = 00, 1 = 01, 2 = 11 (TT Rx STM32)
        return {"hold": int(m.group(1)), "step_lane1": step.get(status >> 2 & 3, -1),
                "step_lane2": step.get(status >> 7 & 3, -1)}

    # Raw sample stream --------------------------------------------------------------------------------------------
    def samples(self, sensor=1, seconds=1, shift=0, mask=None, keep_raw=False):
        """The raw sample stream of one sensor (mask 3 or 12) or of the coils in mask (rank bits 0-3). Fixed rate
        27 397.26 complex samples a second per coil. Returns a dict: iq (complex64 [coil, sample], z >> shift),
        coils, fs_hz, mixer_hz, stretch_starts (sample index of each gap-free stretch), and the stream checks."""
        mask = SENSOR_MASK[int(sensor)] if mask is None else int(mask)
        if mask not in (1, 2, 3, 4, 8, 12):
            raise RxError(f"coil mask {mask}: the coils of one sensor only (1, 2, 3, 4, 8, 12); both sensors: two captures")
        if not 1 <= int(seconds) <= 600 or not 0 <= int(shift) <= 9:
            raise RxError("seconds 1-600, shift 0-9")
        self._send(f"stm raw {mask} {int(shift)} {int(seconds)}")
        buf, t0, start = bytearray(), time.time(), None
        while time.time() - t0 < int(seconds) + 15:
            chunk = self.ser.read(65536)
            if chunk:
                buf += chunk
            if start is None:
                i = buf.find(b"RAW start")
                j = buf.find(b"\n", i) if i >= 0 else -1
                if j >= 0:
                    start = j + 1
                    continue
                for bad in (b"STM raw refused", b"STM raw usage", b"RAW error", b"STM refused"):
                    k = buf.find(bad)
                    if k >= 0 and buf.find(b"\n", k) >= 0:
                        raise RxError(buf[k:buf.find(b"\n", k)].decode(errors="replace"))
                if time.time() - t0 > 5:
                    raise RxError("stm raw: no RAW start line in 5 s")
            else:
                k = buf.find(b"\nRAW end ", start)
                if k >= 0 and buf.find(b"\n", k + 1) >= 0:
                    break
        if start is None:
            raise RxError("stm raw: no RAW start line")
        k = buf.find(b"\nRAW end ", start)
        end_line = buf[k + 1:buf.find(b"\n", k + 1)].decode(errors="replace") if k >= 0 else None
        cap = parse_stream(bytes(buf[start:k if k >= 0 else len(buf)]), mask)
        cap["end_line"] = end_line
        cap["seconds"], cap["shift"] = int(seconds), int(shift)
        if keep_raw:
            cap["raw"] = bytes(buf[start:k if k >= 0 else len(buf)])
        return cap


def parse_stream(data, mask):
    """Packets of a stream capture (bytes after the RAW start line). Reports breaks, never fills them."""
    n_coils = bin(mask & 15).count("1")
    size = 6 + 8 * n_coils * 4 + 2
    res = {"mask": mask, "coils": [COIL_NAMES[r] for r in range(4) if mask >> r & 1], "fs_hz": FS, "mixer_hz": MIX,
           "packets": 0, "crc_fail": 0, "seq_breaks": 0, "missing": 0, "skipped_bytes": 0, "saturated_packets": 0,
           "gaps": 0, "end": None, "stm_line": None}
    blocks, starts, prev, i, n_samp = [], [], None, 0, 0
    while i < len(data) - 1:
        if data[i] == 0xA5 and data[i + 1] == 0x5B and i + 14 <= len(data):
            sent, ovr, sat, drop = struct.unpack_from("<IHHH", data, i + 2)
            if struct.unpack_from("<H", data, i + 12)[0] == _crc16(data[i + 2:i + 12]):
                res["end"] = {"sent": sent, "overruns": ovr, "saturated": sat, "dropped": drop}
                i += 14
                m = data.find(b"RAW stm ", i)
                if m >= 0:
                    res["stm_line"] = data[m:data.find(b"\n", m) if data.find(b"\n", m) >= 0 else len(data)].decode(errors="replace")
                break
        if data[i] == 0xA5 and data[i + 1] == 0x5A and i + size <= len(data) and data[i + 4] == mask:
            if struct.unpack_from("<H", data, i + size - 2)[0] == _crc16(data[i + 2:i + size - 2]):
                seq, flags = struct.unpack_from("<H", data, i + 2)[0], data[i + 5]
                brk = prev is not None and seq != (prev + 1) & 0xFFFF
                if brk:
                    res["seq_breaks"] += 1
                    res["missing"] += (seq - prev - 1) & 0xFFFF
                if flags & 8:
                    res["gaps"] += 1
                if flags & 1:
                    res["saturated_packets"] += 1
                if prev is None or brk or flags & 0x0E:  # an overrun, a gap, a break or dropped packets: a new stretch
                    starts.append(n_samp)
                prev = seq
                res["packets"] += 1
                blocks.append(data[i + 6:i + size - 2])
                n_samp += 8
                i += size
                continue
            res["crc_fail"] += 1
        res["skipped_bytes"] += 1
        i += 1
    a = np.frombuffer(b"".join(blocks), dtype="<i2").reshape(-1, n_coils, 2) if blocks else np.zeros((0, n_coils, 2), "<i2")
    res["iq"] = (a[..., 0] + 1j * a[..., 1]).astype(np.complex64).T.copy()  # [coil, sample]
    res["samples"] = int(a.shape[0])
    res["stretch_starts"] = starts
    e = res["end"]
    res["ok"] = bool(e and e["sent"] == res["packets"] and not res["crc_fail"] and not res["seq_breaks"]
                     and not e["overruns"] and not e["dropped"])
    return res


def summary(cap):
    """The capture without its sample array (for JSON)."""
    return {k: v for k, v in cap.items() if k not in ("iq", "raw", "stretch_starts")}


def tones(cap, freqs_hz, min_len=64):
    """Amplitude and phase per coil at each frequency (Hz at the coil, inside 41 096-68 493 Hz) from a capture.
    Per gap-free stretch (settling samples dropped) a Hann-window DFT; amplitude = mean over stretches of the tone
    peak; phase = angle of the sum over stretches of X_coil * conj(X_ref) / |X_ref|, with the first coil the
    reference (0 deg). Returns a list of {hz, stretches, coils: {name: {amp_z, amp_counts, phase_deg}}}."""
    iq, n = cap["iq"], cap["iq"].shape[1]
    bounds = list(cap["stretch_starts"]) + [n]
    segs = [(a + SETTLE, b) for a, b in zip(bounds[:-1], bounds[1:]) if b - a - SETTLE >= min_len]
    out = []
    for f in freqs_hz:
        fb = f - MIX
        if abs(fb) >= FS / 2:
            raise RxError(f"{f} Hz is outside the sample band {MIX - FS / 2:.0f}-{MIX + FS / 2:.0f} Hz")
        xs = []
        for a, b in segs:
            m = b - a
            w = np.hanning(m)
            ph = np.exp(-2j * np.pi * fb * np.arange(m) / FS)
            xs.append(iq[:, a:b] @ (w * ph) / w.sum())  # complex tone amplitude per coil
        xs = np.array(xs)  # [stretch, coil]
        ent = {"hz": f, "stretches": len(segs), "coils": {}}
        if len(segs):
            ref = xs[:, 0]
            for c, name in enumerate(cap["coils"]):
                amp = float(np.mean(np.abs(xs[:, c])))
                rel = np.sum(xs[:, c] * np.conj(ref) / np.maximum(np.abs(ref), 1e-12))
                ent["coils"][name] = {"amp_z": round(amp, 3), "amp_counts": round(amp / Z_PER_COUNT, 4),
                                      "phase_deg": round(float(np.degrees(np.angle(rel))), 2)}
        out.append(ent)
    return out


# One open per call --------------------------------------------------------------------------------------------------
def info(port):
    with Rx(port) as rx:
        return rx.info()


def polarity(port):
    with Rx(port) as rx:
        return rx.polarity()


def bins(port, which=None, phase=False, sensor=1, seconds=1, mask=None):
    with Rx(port) as rx:
        return rx.bins(which, phase, sensor, seconds, mask)


def set_bin(port, index, hz):
    with Rx(port) as rx:
        return rx.set_bin(index, hz)


def pga(port, hold=None):
    with Rx(port) as rx:
        return rx.pga(hold)


def samples(port, sensor=1, seconds=1, shift=0, mask=None, keep_raw=False):
    with Rx(port) as rx:
        return rx.samples(sensor, seconds, shift, mask, keep_raw)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("polarity")
    b = sub.add_parser("bins")
    b.add_argument("--bins", help="bin indices, e.g. 0,1,5 (default all)")
    b.add_argument("--phase", action="store_true", help="also amplitude and phase from a sample capture")
    s = sub.add_parser("set-bin")
    s.add_argument("index", type=int)
    s.add_argument("hz", type=float)
    g = sub.add_parser("pga")
    g.add_argument("--hold", type=int, choices=(-1, 0, 1, 2), help="Rx STM32 CID 29, RAM only: -1 auto, 0-2 fixed step")
    c = sub.add_parser("samples")
    c.add_argument("--out", help="write the capture: .npz (iq, stretch_starts, fs_hz, mixer_hz) or .bin (stream bytes)")
    c.add_argument("--freqs", help="also amplitude and phase at these Hz, e.g. 53333.3,55315.5")
    c.add_argument("--shift", type=int, default=0)
    for p in (b, c):
        p.add_argument("--sensor", type=int, default=1, choices=(1, 2))
        p.add_argument("--mask", type=int, help="coil mask 1, 2, 3, 4, 8 or 12 (one sensor) instead of --sensor (rank 0-3 = s1c1, s1c2, s2c1, s2c2)")
        p.add_argument("--seconds", type=int, default=1)
    a = ap.parse_args()
    try:
        if a.cmd == "info":
            out = info(a.port)
        elif a.cmd == "polarity":
            out = polarity(a.port)
        elif a.cmd == "bins":
            which = [int(x) for x in a.bins.split(",")] if a.bins else None
            out = bins(a.port, which, a.phase, a.sensor, a.seconds, a.mask)
        elif a.cmd == "set-bin":
            out = set_bin(a.port, a.index, a.hz)
        elif a.cmd == "pga":
            out = pga(a.port, a.hold)
        else:
            cap = samples(a.port, a.sensor, a.seconds, a.shift, a.mask, keep_raw=bool(a.out and a.out.endswith(".bin")))
            out = summary(cap)
            if a.freqs:
                out["tones"] = tones(cap, [float(x) for x in a.freqs.split(",")])
            if a.out:
                if a.out.endswith(".bin"):
                    open(a.out, "wb").write(cap["raw"])
                else:
                    np.savez(a.out, iq=cap["iq"], stretch_starts=np.array(cap["stretch_starts"]), fs_hz=FS, mixer_hz=MIX,
                             coils=np.array(cap["coils"]))
                out["file"] = a.out
    except RxError as e:
        print(json.dumps({"port": a.port, "error": str(e)}))
        return 1
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
