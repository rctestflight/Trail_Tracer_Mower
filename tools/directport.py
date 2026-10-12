"""A board's COM port opened directly, for a PC with no bench daemon. It answers the
same calls as the daemon's HTTP API (/status, /write?nl=1, /lines), so a tool runs unchanged on it.
The rules (from bench tests):
  - open 2 s after the port appears; one open only: a failed open sets "fault" until the port
    leaves and returns (no retry runs: they made Windows stop enumerating the port);
  - after a port error (a board restart), one new open when the port is back;
  - close with RTS off, then DTR off (arduino-esp32 USBCDC line-state machine);
  - one write at a time.
Usage:  from directport import DirectPort; p = DirectPort("COM7"); p.api("/status")"""
import datetime as dt
import threading
import time

import serial
from serial.tools import list_ports


class DirectPort:
    def __init__(self, com):
        self.com = com
        self.ser = None
        self.state = "searching"
        self.last_rx = None
        self.lines = []
        self.partial = b""
        self.lock = threading.Lock()
        self.wlock = threading.Lock()
        self.appear_t = None
        self.stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _present(self):
        return any(p.device == self.com for p in list_ports.comports())

    def _close(self):
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

    def _add(self, text):
        with self.lock:
            self.lines.append([len(self.lines), dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3], text])

    def _run(self):
        while not self.stop:
            if self.ser is None:
                present = self._present()
                if not present:
                    self.appear_t = None
                    if self.state == "fault":
                        self.state = "searching"  # the port left: a new device may open
                    time.sleep(0.2)
                    continue
                if self.state == "fault":
                    time.sleep(0.5)
                    continue
                if self.appear_t is None:
                    self.appear_t = time.time()
                if time.time() - self.appear_t < 2.0:
                    time.sleep(0.2)
                    continue
                try:
                    self.ser = serial.Serial(self.com, 115200, timeout=0.05, write_timeout=1)
                    self.state = "open"
                    self._add(f"[directport] open {self.com}")
                except Exception as e:
                    self.state = "fault"
                    self._add(f"[directport] open of {self.com} failed: {e}; no retry until the port leaves and returns")
                continue
            try:
                data = self.ser.read(4096)
            except Exception as e:
                self._close()
                self.appear_t = None
                self.state = "searching"
                self._add(f"[directport] port lost: {type(e).__name__}; one new open when it is back")
                continue
            if data:
                self.last_rx = time.time()
                *done, self.partial = (self.partial + data).split(b"\n")
                for raw in done:
                    self._add(raw.rstrip(b"\r").decode("utf-8", "replace"))

    def close(self):
        self.stop = True
        time.sleep(0.1)
        self._close()

    def api(self, path, body=None):
        if path.startswith("/status"):
            with self.lock:
                n = len(self.lines)
            return {"state": self.state, "com": self.com, "next": n,
                    "last_rx_age_s": None if self.last_rx is None else round(time.time() - self.last_rx, 1)}
        if path.startswith("/write"):
            if self.ser is None or self.state != "open":
                raise IOError(f"port not open (state {self.state})")
            payload = (body or b"") + (b"\n" if "nl=1" in path else b"")
            with self.wlock:
                self.ser.write(payload)
            return {"ok": True}
        if path.startswith("/lines"):
            q = dict(kv.split("=") for kv in path.split("?", 1)[1].split("&"))
            since, mx = int(q.get("since", 0)), int(q.get("max", 500))
            with self.lock:
                out = self.lines[since:since + mx]
            return {"next": out[-1][0] + 1 if out else since, "lines": out}
        raise ValueError(path)
