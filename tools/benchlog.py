"""Prints one time-stamped line. The bench copy of this module (RxPortd/benchlog.py) also writes the
bench log; a PC with no bench daemon has no bench log."""
import datetime as dt


def post(msg):
    print(f"{dt.datetime.now():%Y-%m-%d %H:%M:%S}  {msg}", flush=True)
