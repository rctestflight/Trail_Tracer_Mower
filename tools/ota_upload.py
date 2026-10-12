"""Update a rover's firmware over USB (OTA), on a COM port: python tools/ota_upload.py COM7 firmware.bin
--expect rover-firmware (needs pyserial, tools/directport.py and tools/benchlog.py). The PlatformIO Upload
button runs it for a board with this firmware (tools/upload_auto.py).
No ROM download mode, no button, no power cycle. Needs firmware with the "ota" commands (lib/usb_ota).
  1 checks: ESP image magic, chip ID 2 (ESP32-S2), app descriptor; the --expect text is in the image
  2 "ota begin <size> <sha256> confirm", then "ota w <offset> <hex>" lines of 512 bytes, one answer
    each. A lost or bad answer: "ota status" gives the offset, and the upload continues from it; a
    port reopen (USB re-enumeration) is waited for, then the same.
  3 "ota end": the board checks size and SHA-256, selects the new slot, restarts with the USB detach
  4 back on USB: "ota status", then "ota confirm" (proves the link both ways on the new image), then
    "ota status" again. An image with the confirm gate that gets no confirm in 120 s returns to the
    previous image by itself.
Usage: python ota_upload.py <COM port> <firmware.bin> [--expect rover-firmware]"""
import argparse
import hashlib
import zlib
import json
import time
import urllib.request
from pathlib import Path

from benchlog import post

ap = argparse.ArgumentParser()
ap.add_argument("board")
ap.add_argument("fw")
ap.add_argument("--expect", default=None, help="text that must be in the image, e.g. rover-firmware")
ap.add_argument("--no-confirm", action="store_true", help="rollback test: send no confirm, wait for the gate's rollback")
ap.add_argument("--quiet-check", action="store_true",
                help="10 s heartbeat check before the update (off by default: update mode keeps the board quiet)")
a = ap.parse_args()
board, fw = a.board, Path(a.fw)
data = fw.read_bytes()
sha = hashlib.sha256(data).hexdigest()


def image_error():
    if data[0] != 0xE9:
        return "not an ESP image (magic)"
    if int.from_bytes(data[12:14], "little") != 2:
        return "not an ESP32-S2 image (chip ID)"
    if int.from_bytes(data[0x20:0x24], "little") != 0xABCD5432:
        return "no app descriptor"
    if a.expect and a.expect.encode() not in data:
        return f"text {a.expect!r} not in the image"
    return None


# The image checks come before any port is opened: a refused image never touches the board.
err = image_error()
if err:
    post(f"{board}: OTA {fw.name} refused: {err}")
    raise SystemExit(1)
from directport import DirectPort  # noqa: E402

direct = DirectPort(board.upper() if board.lower().startswith("com") else board)  # the COM port, opened and closed with the safe rules of directport.py
url = None
import contextlib  # noqa: E402
risky = lambda b, act: contextlib.nullcontext()  # noqa: E731


def chunk_line(off, chunk):
    line = f"ota w {off} {chunk.hex()}"
    return line + f" {zlib.crc32(chunk):08x}" if use_crc else line


use_crc = False


def api(path, body=None):
    if direct is not None:
        return direct.api(path, body)
    with urllib.request.urlopen(urllib.request.Request(url + path, data=body), timeout=3) as r:
        return json.loads(r.read())


nxt = 0


def send_wait(text, timeout=5.0):
    """Send one line; return the board's first answer line that starts with "OTA", or None.
    None also when the daemon cannot write (port reopen): the caller asks "ota status" later."""
    global nxt
    try:
        nxt = api("/status")["next"] if nxt == 0 else nxt
        api("/write?nl=1", text.encode())
    except Exception:
        return None
    end = time.time() + timeout
    while time.time() < end:
        try:
            r = api(f"/lines?since={nxt}&max=200")
        except Exception:
            return None
        for _, _, x in r["lines"]:
            if x.startswith("OTA"):
                nxt = r["next"]
                return x
        nxt = r["next"]
        time.sleep(0.005)
    return None


def wait_board(sec):
    """Wait until the daemon is open with fresh data; True if so within sec."""
    t1 = time.time()
    while time.time() - t1 < sec:
        try:
            st = api("/status")
            if st["state"] == "open" and st["last_rx_age_s"] is not None and st["last_rx_age_s"] < 1.5:
                return True
        except Exception:
            pass
        time.sleep(0.2)
    return False


def written_now():
    """The board's written count from "ota status", after a lost answer or a USB re-enumeration."""
    for _ in range(3):
        wait_board(15)
        r = send_wait("ota status")
        if r and r.startswith("OTA status") and "active=1" in r:
            return int(r.split("written=")[1].split()[0])
        if r and "active=0" in r:
            return None  # the board lost the upload (restart): start again
    return None


def quiet_check(sec=10.0):
    """The board must be regular before an update: at least 9 heartbeat lines (rover "HB", monitor
    "beat") in 10 s, no gap over 1.6 s. A busy board (flash erases after a boot) lost the upload and
    then the host lost the board (bench test)."""
    n0 = api("/status")["next"]
    time.sleep(sec)
    lines = api(f"/lines?since={n0}&max=5000")["lines"]
    import datetime as dt
    ts = [dt.datetime.strptime(t, "%Y-%m-%d %H:%M:%S.%f").timestamp() for _, t, x in lines
          if x.startswith("HB ") or " beat " in x]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    ok = len(ts) >= 9 and (max(gaps) if gaps else 99) < 1.6
    post(f"{board}: quiet check: {len(ts)} heartbeats in {sec:.0f} s, longest gap "
         f"{max(gaps) if gaps else 0:.2f} s: {'ok' if ok else 'NOT QUIET, no update'}")
    return ok


if a.quiet_check and not quiet_check():
    raise SystemExit(1)
with risky(board, f"OTA {fw.name}"):
    if not wait_board(15):  # a COM port opens 2 s after it appears; the board must send lines
        raise SystemExit(f"stop: {board} sends no lines (port {api('/status')['state']})")
    t0 = time.time()
    nxt = api("/status")["next"]
    # A line can lose bytes on the USB input and still be valid hex: the board then writes a short,
    # wrong chunk (bench test: written counts off a chunk end, then a sha256 mismatch). So the
    # upload goes on only when the board's count is exactly at a chunk end that this tool sent:
    # count = off + chunk (answer lost) goes on; count = off (line lost) sends the chunk again;
    # any other count aborts, and the upload starts again from 0 (at most 3 attempts).
    for attempt in range(1, 4):
        r = send_wait(f"ota begin {len(data)} {sha} confirm crc")  # older usb_ota ignores "crc"
        post(f"{board}: OTA {fw.name} ({len(data)} B, sha256 {sha[:12]}) attempt {attempt}: {r}")
        if not r or not r.startswith("OTA begin ok"):
            raise SystemExit("stop")
        use_crc = "crc=1" in r  # usb_ota with a CRC-32 per line: the board refuses a damaged line
        off, resends, bad = 0, 0, False
        while off < len(data):
            chunk = data[off:off + 512]
            r = send_wait(chunk_line(off, chunk))
            if r == f"OTA ok {off + len(chunk)}":
                off += len(chunk)
                if off % (64 * 1024) < 512:
                    post(f"{board}: OTA {100 * off // len(data)} % ({time.time() - t0:.0f} s)")
                continue
            w = written_now()
            if w == off + len(chunk):
                off = w
                continue
            resends += 1
            if w == off and resends <= 20:
                post(f"{board}: OTA line at {off} not taken ({r!r}); sent again")
                continue
            post(f"{board}: OTA answer at {off} was {r!r}; board count {w}; not a chunk end: abort, start again")
            send_wait("ota abort")
            bad = True
            break
        if not bad:
            break
    else:
        raise SystemExit("stop: 3 attempts failed")
    resumes = resends
    r = send_wait("ota end", timeout=15)
    post(f"{board}: {r} ({time.time() - t0:.0f} s, {resumes} resumes)")
    if r and not r.startswith("OTA done"):
        raise SystemExit("stop")
    # No answer: the board can restart before its "OTA done" line reaches the PC (bench test).
    # "ota status" after the restart tells: state=1 gate=1 is the new image waiting for confirm.
    t1 = time.time()
    # Back means: the new boot line, then fresh data. Fresh data alone can still come from the old
    # image before its restart (bench test: "back" after 1.0 s, the status came from the old image).
    n_boot = nxt
    booted = False
    while time.time() - t1 < 30 and not booted:
        try:
            r = api(f"/lines?since={n_boot}&max=2000")
            n_boot = r["next"]
            booted = any(x.startswith("BOOT rover-firmware") or "Trail-Tracer-Rx-Monitor v" in x for _, _, x in r["lines"])
        except Exception:
            pass
        time.sleep(0.2)
    back = booted and wait_board(15)
    post(f"{board}: {'back on USB' if back else 'NOT back on USB'} {time.time() - t1:.1f} s after restart")
    nxt = api("/status")["next"]
    time.sleep(1.0)
    # A USB guard reconnect right after the restart can lose an answer: ask again, up to 6 times
    # in about 30 s (the gate rolls an unconfirmed image back at 120 s).
    st = None
    for _ in range(6):
        st = send_wait("ota status")
        if st:
            break
        time.sleep(4)
    post(f"{board}: {st}")
    if not st or "state=1" not in st:
        post(f"{board}: no new image waits for confirm; stop")
        raise SystemExit("stop")
    if a.no_confirm:
        # Rollback test: no confirm. The gate restarts the image at 120 s of running, and the
        # bootloader starts the previous slot.
        slot = st.split("running=")[1].split()[0]
        post(f"{board}: rollback test: no confirm; waiting for the gate (120 s of running)")
        time.sleep(110)
        t2 = time.time()
        while time.time() - t2 < 60:
            nxt = api("/status")["next"]
            s2 = send_wait("ota status") if wait_board(5) else None
            if s2 and f"running={slot}" not in s2:
                post(f"{board}: rollback done: {s2} ({time.time() - t1:.0f} s after the update restart)")
                break
            time.sleep(2)
        else:
            post(f"{board}: rollback test FAILED: still {slot} after {time.time() - t1:.0f} s")
            raise SystemExit("stop")
    else:
        post(f"{board}: {send_wait('ota confirm')}")
        post(f"{board}: {send_wait('ota status')}")
    for _, t, x in api(f"/lines?since={max(0, api('/status')['next'] - 60)}&max=60")["lines"]:
        if x.startswith(("BOOT", "SETUP", "FIRST_PACKET", "SAFE")) or "Monitor v" in x:
            post(f"{board}: {x[:100]}")
