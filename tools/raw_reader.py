r"""Reader for the ESP32 raw stream tests (firmware 0.5.73, USB commands "raw usb", "raw pass", "raw loop"; 0.5.74 "stm raw").

  python raw_reader.py <port> "<raw command>" [--out capture.bin]

Opens the port (pyserial), sends the raw command, reads until the "RAW end" line or 150 s, and prints:
the bytes received per second, the total, and for "raw usb" the RAWU block sequence check (missing blocks,
bad blocks). The ESP32's own "RAW end" line follows (bytes in and out, bytes dropped, kB/s). --out keeps every
byte between "RAW start" and "RAW end" for an offline check. The port is closed at the end.
"""
import argparse
import struct
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("command")
    ap.add_argument("--out")
    a = ap.parse_args()
    with serial.Serial(a.port, 115200, timeout=0.05) as ser:
        ser.reset_input_buffer()
        ser.write((a.command.strip() + "\n").encode())
        buf = bytearray()
        start = None
        t_start = time.time()
        per_s = []
        last_n, last_t = 0, time.time()
        while time.time() - t_start < 150:
            chunk = ser.read(65536)
            if chunk:
                buf += chunk
            if start is None:
                i = buf.find(b"RAW start")
                if i >= 0:
                    j = buf.find(b"\n", i)
                    if j >= 0:
                        print(buf[i:j].decode(errors="replace"))
                        start = j + 1
                        last_n, last_t = len(buf), time.time()
                elif b"usage" in buf or b"refused" in buf:
                    print(buf.decode(errors="replace").strip())
                    return 1
            else:
                now = time.time()
                if now - last_t >= 1.0:
                    per_s.append((len(buf) - last_n) / (now - last_t))
                    last_n, last_t = len(buf), now
                k = buf.find(b"RAW end", start)
                if k >= 0 and buf.find(b"\n", k) >= 0:
                    break
        if start is None:
            print("no RAW start line")
            return 1
        k = buf.find(b"RAW end", start)
        data = bytes(buf[start:k if k >= 0 else len(buf)])
        m = data.rfind(b"\nRAW stm ")  # stm raw: the packet count line before RAW end
        stm_line = ""
        if m >= 0:
            stm_line = data[m + 1:].split(b"\n")[0].decode(errors="replace")
            data = data[:m]
        if data.endswith(b"\n"):
            data = data[:-1]
        end_line = buf[k:buf.find(b"\n", k)].decode(errors="replace") if k >= 0 else "(no RAW end line)"
    if a.out:
        open(a.out, "wb").write(data)
    print(f"received {len(data)} B; per second kB: " + " ".join(f"{v / 1000:.1f}" for v in per_s))
    if per_s:
        print(f"mean {sum(per_s) / len(per_s) / 1000:.1f} kB/s over {len(per_s)} s")
    if a.command.split()[1:2] == ["usb"]:
        n_blocks, bad, missing, prev = len(data) // 512, 0, 0, None
        for b in range(n_blocks):
            blk = data[512 * b:512 * (b + 1)]
            if blk[:4] != b"RAWU":
                bad += 1
                continue
            seq = struct.unpack_from("<I", blk, 4)[0]
            if prev is not None and seq != prev + 1:
                missing += seq - prev - 1 if seq > prev else 1
            prev = seq
        print(f"RAWU blocks {n_blocks}, bad {bad}, missing {missing}, partial tail {len(data) % 512} B")
    if stm_line:
        print(stm_line)
    print(end_line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
