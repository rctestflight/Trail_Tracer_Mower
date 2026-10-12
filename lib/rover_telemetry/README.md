# rover_telemetry

Packs the record bodies of the rover telemetry log: the 100 Hz sample block (0x20), the status record (0x21) and the event record (0x30). The layouts are in `docs/espnow_protocol.md`, "Telemetry log". The numbering, the RAM and flash rings and the radio are in `lib/telemlog` (from Trail-Tracer-Rx-Monitor; copy it from there, do not edit it here).

No Arduino dependency: the same code builds on a PC for the tests.

## Where the firmware calls it

`src/main.cpp`, section "Telemetry log":

| Function | When | What |
|---|---|---|
| `telemetryBegin()` | `setup()`, after the settings load | boot_id from NVS, `telemlog_init()`, flash ring attach, boot event |
| `telemetrySample()` | each STM32 packet that passes its CRC check | one sample; every 7th sample stores a block |
| `telemetryService()` | each `loop()` | status record each second, events on changes, flash ring service, USB dump |
| `telemlog_poll()` | each `loop()` while ESP-NOW runs | live frames, INFO, FLASHINFO, backfill |

## Tests

| Test | Command (from the repository root) | Result at 9e968af |
|---|---|---|
| Host unit test: half and bfloat16 conversion against Python, known sample, 120 s stream through telemlog, dump equal to the live frames | `test\telemetry_host\build_host.cmd`, then `python test/telemetry_host/check.py` | 13 of 13 pass |
| rover-sim data end to end: every packet of a 450 s garden run with noise, quantisation, rate, dead reckoning from the logged wheel speeds, recovery after outages | `node test/sim_harness/run.mjs '{..., "packetLog": "<prefix>"}'`, then `python test/telemetry_host/sim_check.py <prefix>_cw_70.txt --outage-y 2.0` | 0.5 mm, coil 0.39 %, 27.8 kbit/s, 0.040 m dead reckoning error over 450 s, 82 % of 7-10 s outages recovered with 100 FAST slots |

## Quantisation

| Field | Step | Range |
|---|---|---|
| x, y | 1 mm | +-32.767 m |
| radius | 1 mm | 0 to 65.535 m |
| drive, steer | 1/10000 of full | +-3.2767 |
| wheel speed | 0.001 rev/s | +-32.767 rev/s |
| gyro | 0.02 deg/s | +-655 deg/s |
| beacon radius | 4 mm | 0 to 1020 mm |
| coil values | bfloat16, relative error 0.4 % at most | the float32 range |
| crossing point | 1 mm | +-32.767 m |
