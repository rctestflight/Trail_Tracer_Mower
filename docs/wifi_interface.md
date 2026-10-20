# Wi-Fi telemetry link

The vehicle can send its telemetry over a Wi-Fi network instead of ESP-NOW. The same frames go to the PC
telemetry server, one frame per UDP datagram. The drive loop (sensor frame, control, motor commands) never
waits for the network.

Status: stable from ESP32 firmware 0.5.78 (bench test on 2026-10-13: 0 of 1600 datagrams lost, 0 decode errors in
the telemetry app). A change to this interface gets a new header version.

## Settings and commands

| Item | Value |
|---|---|
| `radio_mode` | `espnow` (default) or `wifi`; applies at the next start (`save`, then restart) |
| Network | USB command `wifi set <ssid> <password>` (a quoted SSID can hold spaces); `wifi clear`. Kept in the board's own memory, never in `config.json`; refused while the vehicle moves |
| Read-only keys | `wifi_ssid` (the network name), `wifi_password` (`set` or `not set`; the password itself is never printed, logged or listed) |
| Status | USB `wifi`: mode, network, state, signal strength, channel, IP, server, counters |
| Scan | USB `wifi scan`: networks with signal strength, channel, security |
| Events | Log event 19: associated, association lost, server found, no server found within 30 s |

## Protocol

| Part | Value |
|---|---|
| Server discovery | UDP broadcast beacon to port 47700, text `TTSRV1 <ip> <port>`; or mDNS service `_trailtracer._udp` (TXT `v=1`). The last address learned wins |
| Data | Unicast UDP to the server, from local port 47701. Each datagram: `"TT"`, version 1, 3 zero bytes, the vehicle's 6-byte MAC, then the ESP-NOW payload unchanged (`docs/espnow_protocol.md`) |
| Server to vehicle | From firmware 0.5.86: one datagram per frame to the vehicle's address, port 47701, with the same 12-byte header (the vehicle's MAC), then the ESP-NOW frame unchanged: CMD 0x10, STOP 0x12, RESTART 0x7F, REQ 0x41. Only datagrams from the known server address count; the fleet key, one-CMD-at-a-time and retry rules are those of ESP-NOW |
| Vehicle to server | ACK 0x11 and backfill answers go to the server as telemetry datagrams do |

A receiver decodes Wi-Fi frames with its ESP-NOW decoder after it strips the 12-byte header.

## Limits

- Rates: the frame rates of the log (`telemetry_rate_hz` up to 100 Hz, `telemetry_bins_hz`). The raw sample
  stream goes over USB, not Wi-Fi.
- One radio at a time: in Wi-Fi mode the vehicle sends no ESP-NOW frames, and a Monitor Rx does not hear it.
- Commands over Wi-Fi (0.5.86+): the same set as over ESP-NOW. A command that would cut the Wi-Fi link (radio_mode espnow, espnow_enabled or telemetry_enabled false) is refused; wifi set and wifi clear go over USB only.
