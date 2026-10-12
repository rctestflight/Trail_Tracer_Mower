"""The PlatformIO Upload button (tools/pio_upload_hook.py runs this file with the port and the images).

1. The board on the port answers "boot" with "BOOT rover-firmware ..." (this firmware): update over USB
   with tools/ota_upload.py. The settings and the log stay; a new image with no confirm returns to the
   previous image. No power-off.
2. No answer (V1.0, another firmware, or a blank board): reset the board into its ROM download mode over
   USB; if the board ignores the reset (V1.0 on Windows), ask for the button way and wait for the ROM
   port; then erase the full flash, write the four images and ask for a full power-off.
3. The board is already in ROM download mode: as 2, with no reset.

Usage (normally from PlatformIO): python upload_auto.py --port COM7 --esptool <esptool.py>
                                    <offset> <image> [<offset> <image> ...]
Set FULL_INSTALL=1 in the environment to force path 2 on a board with this firmware."""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from serial.tools import list_ports

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
VID = 0x303A        # Espressif USB vendor ID
ROM_PID = 0x0002    # ESP32-S2 ROM download mode
POWER_OFF = "Full power-off now: USB and vehicle power off, 5 s, on."


def ports(pid=None):
    return [p.device for p in list_ports.comports() if p.vid == VID and (pid is None or p.pid == pid)]


def pid_of(dev):
    return next((p.pid for p in list_ports.comports() if p.device == dev), None)


def boot_line(dev, wait_s=4.0):
    """The BOOT line of this firmware, or None. The port opens and closes with the bench rules."""
    from directport import DirectPort
    d = DirectPort(dev)
    try:
        t0 = time.time()
        while d.api("/status")["state"] != "open" and time.time() - t0 < 5.0:
            time.sleep(0.2)
        if d.api("/status")["state"] != "open":
            return None
        for _ in range(3):  # a line written right after the open can be lost: ask up to 3 times
            n0 = d.api("/status")["next"]
            d.api("/write?nl=1", b"boot")
            t0 = time.time()
            while time.time() - t0 < wait_s / 2:
                time.sleep(0.2)
                for _, _, x in d.api(f"/lines?since={n0}&max=2000")["lines"]:
                    if x.startswith("BOOT rover-firmware"):
                        return x.strip()
        return None
    finally:
        d.close()


def run(cmd):
    print("> " + " ".join(str(c) for c in cmd), flush=True)
    return subprocess.call([str(c) for c in cmd])


def send_line(dev, text):
    from directport import DirectPort
    d = DirectPort(dev)
    try:
        t0 = time.time()
        while d.api("/status")["state"] != "open" and time.time() - t0 < 5.0:
            time.sleep(0.2)
        d.api("/write?nl=1", text.encode())
        time.sleep(0.5)
    finally:
        d.close()


def wait_rom(before, sec=15.0):
    t0 = time.time()
    while time.time() - t0 < sec:
        new = [p for p in ports(ROM_PID) if p not in before] or ports(ROM_PID)
        if new:
            time.sleep(1.0)
            return new[0]
        time.sleep(0.3)
    return None


def enter_rom(dev, esptool, this_firmware):
    """Reset the board into its ROM download port over USB, with no button.
    This firmware: its "dld" command (it ignores the USB reset sequences on purpose).
    Another Arduino firmware (V1.0): the esptool reset sequence, then the 1200 bps touch. esptool
    reports an error when the port drops: that is expected."""
    before = set(ports(ROM_PID))
    if this_firmware:
        send_line(dev, "dld")
        return wait_rom(before)
    run([sys.executable, esptool, "--chip", "esp32s2", "--port", dev, "--before", "default_reset",
         "--after", "no_reset", "chip_id"])
    rom = wait_rom(before, 5.0)
    if rom is None and dev in ports():
        import serial
        print(f"upload_auto: 1200 bps touch on {dev}", flush=True)
        try:
            serial.Serial(dev, 1200).close()
        except Exception as e:
            print(f"upload_auto: touch: {e}", flush=True)
        rom = wait_rom(before)
    return rom


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--port", required=True)
    a.add_argument("--esptool", required=True)
    a.add_argument("images", nargs="+")
    a = a.parse_args()
    if len(a.images) % 2:
        raise SystemExit("upload_auto: images must come as <offset> <file> pairs")
    pairs = list(zip(a.images[0::2], a.images[1::2]))
    firmware = next(f for off, f in pairs if int(off, 16) == 0x10000)
    if a.port not in ports():  # PlatformIO can pick another serial port: use the one Espressif USB port
        esp = ports()
        if len(esp) != 1:
            raise SystemExit(f"upload_auto: {len(esp)} ESP32-S2 USB ports (vendor 303A) found: {esp}. "
                             "Connect one rover, or set upload_port in platformio.ini.")
        print(f"upload_auto: {a.port} is not an ESP32-S2 USB port; using {esp[0]}", flush=True)
        a.port = esp[0]

    rom = a.port if pid_of(a.port) == ROM_PID else None
    if rom is None:
        line = boot_line(a.port)
        if line:
            print(f"upload_auto: {a.port}: {line}", flush=True)
        if line and os.environ.get("FULL_INSTALL") != "1":
            print("upload_auto: this firmware answers: update over USB (settings and log stay)", flush=True)
            return run([sys.executable, HERE / "ota_upload.py", a.port, firmware, "--expect", "rover-firmware"])
        print(f"upload_auto: {a.port}: full install (erase all, write all)", flush=True)
        rom = enter_rom(a.port, a.esptool, bool(line))
        if rom is None:  # V1.0 and other firmware: the button way
            msg = ("Download mode needs the button: unplug the USB cable, switch off the vehicle power, "
                   "hold the 0 button, plug the USB cable in, release 0. Waiting 120 s.")
            print("=" * 40, msg, "=" * 40, sep="\n", flush=True)
            rom = wait_rom(set(), 120.0)
        if rom is None:
            raise SystemExit("upload_auto: no ROM download port. Do the button steps, then press Upload again.")
    print(f"upload_auto: ROM download port {rom}", flush=True)
    args = [sys.executable, a.esptool, "--chip", "esp32s2", "--port", rom, "--baud", "921600",
            "--before", "no_reset", "--after", "no_reset", "write_flash", "--erase-all", "-z",
            "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "4MB"]
    for off, f in pairs:
        args += [off, f]
    code = run(args)
    if code:
        raise SystemExit(f"upload_auto: esptool exit {code}")
    print("\n" + "=" * len(POWER_OFF) + "\n" + POWER_OFF + "\n" + "=" * len(POWER_OFF), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
