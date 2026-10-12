r"""Rover settings over USB, for a PC with no station and no RxPortd (needs pyserial; tools/directport.py
opens the COM port with the bench's safe rules). The same commands as the station's settings page.

  python tools/rover_settings.py COM7 list                 all keys and values
  python tools/rover_settings.py COM7 get rover_enabled
  python tools/rover_settings.py COM7 set rover_enabled true
  python tools/rover_settings.py COM7 save                 write the settings to the rover's flash

Each answer is "CFG <result> <text>": result 0 ok, 1 unknown key, 2 bad value, 4 refused (for example
a stationary-only key while the rover moves). docs/settings_def.json lists every key, its unit,
limits and default. A set acts at once; save keeps it over a restart."""
import sys
import time

from directport import DirectPort


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    com, cmd = sys.argv[1].upper(), " ".join(sys.argv[2:])
    port = DirectPort(com)
    t0 = time.time()
    while port.api("/status")["state"] != "open":
        if time.time() - t0 > 10:
            sys.exit(f"{com}: not open ({port.api('/status')['state']})")
        time.sleep(0.2)
    time.sleep(0.5)
    n = port.api("/status")["next"]
    port.api("/write?nl=1", cmd.encode())
    t0, done = time.time(), False
    while time.time() - t0 < 5 and not done:
        r = port.api(f"/lines?since={n}&max=500")
        n = r["next"]
        for _, _, x in r["lines"]:
            if x.startswith("CFG "):
                print(x)
                done = not cmd.startswith("list") or " end " in x
        time.sleep(0.1)
    port.close()
    if not done:
        sys.exit("no answer in 5 s")


if __name__ == "__main__":
    main()
