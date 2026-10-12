"""Prints one time-stamped line (the progress lines of ota_upload.py)."""
import datetime as dt


def post(msg):
    print(f"{dt.datetime.now():%Y-%m-%d %H:%M:%S}  {msg}", flush=True)
