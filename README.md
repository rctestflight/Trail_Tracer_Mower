# Trail Tracer Rx firmware (ESP32-S2)

This firmware runs on the ESP32-S2 of the Trail Tracer Rx board. One image has two roles. Every boot starts in the rover role, which drives the vehicle. The USB command `monitor` gives the monitor role for one boot. In the monitor role, the board is the PC radio link to the vehicles.

## Branches

| Branch | Content |
|---|---|
| `main` | The baseline (b601c1b). It ran on a real rover. |
| `feature/rx-v2-firmware` | The baseline, plus rover mode, telemetry, settings commands, updates over USB and the monitor role. |

Outside rover mode, the vehicle moves as on `main`. The guide lists each difference.

## Documents

| File | Content |
|---|---|
| `docs/test_guide.html` | First load over V1.0 with PlatformIO, field steps over USB, the way back to V1.0 |
| `docs/rover_guide.md` | Use of the firmware: settings, rover mode, telemetry, updates, commands, limits, test results |
| `docs/espnow_protocol.md` | The radio messages between a rover and a monitor |
| `docs/settings_def.json` | The settings, with their ranges and defaults |
| `lib/rover_autopilot/README.md` | The rover mode autopilot and its simulator results |
| `lib/rover_telemetry/README.md` | The telemetry records |
| `test/` | Host tests, simulator test bench and bench results |
| `archive/` | Old files and old bench results. History only |

## Build

Use PlatformIO with the environment `lolin_s2_mini`. An update over USB puts the image on the board (`tools/ota_upload.py`, see the guide).
