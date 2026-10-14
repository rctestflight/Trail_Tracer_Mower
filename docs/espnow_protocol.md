# ESP-NOW protocol: Trail Tracer Rx boards

Firmware 0.5.24 (`Trail_Tracer_Mower`, branch `feature/rover-autopilot`). The rover role, the monitor role and the PC telemetry program (`Trail-Tracer-PC-Telemetry`) use this protocol.

## Radio

- Both roles run `WIFI_STA` with no access point, on channel 1. The monitor role sets channel 1 at its start (0.5.23). Do not change the channel.
- Frames are not encrypted. Any ESP32 in range can send a frame to the mower.
- No MAC is fixed anywhere: every Rx board has its own MAC. The mower answers each frame to the MAC it came from, and a collector names a vehicle by its vehicle ID and the MAC seen on air.
- CMD 0x10, STOP 0x12 and RESTART 0x7F carry the fleet key (`cmd_key`, a u32 in the mower's settings). The mower acts on them only when the key matches. With the default `cmd_key` 0, the mower accepts these messages when their key is 0. The mower refuses RESTART while it moves. A frame with a wrong or missing key gets no answer and no effect; the mower counts it and writes event 10. The user puts the same `cmd_key` in every vehicle and in the PC program; a new dongle or PC needs no change on the vehicles. The key goes in clear: it guards against mistakes and other monitors in range, not against an attack. `cmd_key` can be set over USB or in `config.json` only, not over ESP-NOW.
- REQ 0x41 (read only) and PING are always answered.
- One frame carries at most 250 bytes.
- The mower answers from `loop()`, never from a receive callback. The callbacks run in the Wi-Fi task (priority 23) and interrupt `loop()`.

## Frame types

Byte 0 is the type. A frame whose byte 0 is `-` (0x2D) is the text status message.

| Type | Name | Direction | Layout, little-endian |
|---|---|---|---|
| `-` | status text | mower broadcast | ASCII line, see "Status text" |
| 0x01 | THRU | mower broadcast | u8 type, u32 seq, u32 sender_us, zero padding to the test size |
| 0x02 | PING | monitor to mower, unicast | u8 type, u32 seq, u32 rxb_us |
| 0x03 | PONG | mower to source, unicast | the PING bytes with byte 0 set to 0x03 |
| 0x10 | CMD | PC to mower, unicast | u8 type, u16 seq, u32 fleet key, ASCII command text |
| 0x11 | ACK | mower to source, unicast | u8 type, u16 seq, u8 result, ASCII text |
| 0x12 | STOP | PC to mower, unicast or broadcast | u8 type, u16 seq, u32 fleet key |
| 0x40 | LOG | mower broadcast (live), or unicast to a requester (backfill) | one log record, see "Telemetry log" |
| 0x41 | REQ | collector to mower, unicast | request for log records, see "Telemetry log" |
| 0x42 | INFO | mower broadcast, once a second | the record ranges that the mower holds |
| 0x44 | MON | monitor broadcast, once a second | u8 type, u8 version (1), u32 monitor ms, u64 PC unix time in ms (0 = unknown), u8 n (0: no receipts yet) |
| 0x43 | FLASHINFO | mower broadcast, every 5 s | the SLOW record ranges in the mower's flash, per boot |
| 0x7F | RESTART | any to one board, unicast | u8 type, then 4 ASCII bytes: "RST!" restarts the ESP32; "DLD!" restarts it into ROM download mode (USB 303a:0002, for esptool); then, to the mower, u32 fleet key |

## RESTART

- Purpose: to recover a board whose USB stopped, without a person. The ROM download mode has its own USB code, so esptool can flash the board even when the application's USB stack has failed.
- Senders send RESTART unicast to one MAC only, never broadcast.
- Limit: in ESP-IDF 4.4 the receive callback does not report the destination address. A receiver therefore cannot tell a broadcast RESTART from a unicast one, and it acts on both. The unicast rule binds the senders only.
- Both the rover firmware and the monitor firmware (commit 966267b) act on RESTART.

## Status text

ASCII, space-separated, sent about once a second, and also when `charge_code` or `status_bits` changes. It goes out only when ESP-NOW is on (`espnow_enabled` or `telemetry_enabled` true). The format is unchanged from the vehicle firmware.

| Position | Field | Unit |
|---|---|---|
| 0 | `-` | prefix |
| 1 | vehicle ID | integer |
| 2 | vehicle status | 0 stuck, 1 driving, 2 charging |
| 3 | ESP32 `millis()` | ms |
| 4 | battery voltage | V, 2 decimals |
| 5 | charge voltage | V, 2 decimals |
| 6 | charge current | A, 2 decimals |
| 7 | charge state machine step | integer, -1 to 3 |
| 8 | STM32 status bits | integer |
| 9 | STM32 charge code | integer |
| 10 | STM32 temperature | deg C |
| 11 | wire radius (`s1_radius`) | mm |
| 12 | 5 V rail | V, 2 decimals |
| 13 | steer output | servo-equivalent pulse, us (centre 1500) |
| 14 | lap flag | 1 once after the vehicle reaches the charger |

## Shipped units (production V1.0)

A shipped Rx (`TrailTracer` `main`, d4f8135) sends the status text only, and only when `espnow_enabled` is true (default false). It ignores every received frame: it sends no answer, no LOG and no INFO, and has no PC time. MON and all other frames do not change its operation.

- The monitor prints every frame as an R line, with no sender filter. A status text from a shipped unit is byte-identical to the rover role's status text (bench test 2026-10-10: 512 frames of a board on V1.0 and 518 frames of a board on 0.5.22, all 15 tokens in the same format).
- The RSSI line covers every ESP-NOW sender, also one that sends only text.
- Every shipped unit has vehicle ID 1 unless the owner changed it. The PC tells vehicles apart by MAC.

## CMD and ACK

CMD text, one of:

- `get <key>`
- `set <key> <value>`
- `save`
- `list`
- `run start`, `run stop`, `get run` (section "Run and STOP")

`<key>` is the `config.json` key name. `<value>` is a JSON literal with no spaces: a number, `true` or `false`.

ACK result: 0 ok, 1 unknown key, 2 bad value, 3 busy, 4 refused. The ACK text is `key=value` with the value now in force, as a JSON literal. `save` answers `save=nvs`. `list` answers with one ACK per key, all with the same seq, and a last ACK with the text `end <n> def <version>`, where n is the number of keys sent, so that a lost key shows, and version is the version of `docs/settings_def.json` (types, units, limits, live or stationary, descriptions of every key) that the firmware matches.

- `set` applies the value at once, through the same path as a `config.json` edit (validation, then apply). A value that validation clamps comes back clamped in the ACK.
- `save` writes the settings to the ESP32's flash store (NVS) only. It does not write `config.json`, because the PC may have the USB drive mounted.
- The mower keeps one CMD at a time. A CMD that arrives before the previous ACK is sent gets result 3, busy.
- Keys that take effect only after a restart: `enable_sensor_2`. The ACK result is 0, and the text ends with ` restart`.
- Refused keys (result 4): `espnow_enabled` and `telemetry_enabled` with `false`, because the ACK and every later command would then be lost.
- A value is one JSON literal: a value that contains `,` `{` `}` `[` `]` `:` or `"` gets result 2.

## Run and STOP

Defined with the PC telemetry program (Trail-Tracer-PC-Telemetry `docs/control_api.md`, draft 3).

- `run start` (CMD): the mower enters mode 2, autonomous: the autopilot if `rover_enabled` is true, else the wire follower. Answer `run=running`. Refused (result 4, `run=stopped rc_manual`) when an RC transmitter is present with its aux switch in manual: the RC has priority.
- `run stop` (CMD) and STOP (frame 0x12): the mower leaves mode 2 for mode 0. With no RC signal, mode 0 gives neutral output; the autopilot does not run. Always accepted. Answer `run=stopped`; the STOP frame gets an ACK with the same seq and the text `stop`.
- `get run`: `run=running` in mode 2, else `run=stopped`.
- STOP takes no part in the one-CMD-at-a-time rule: the receive callback only sets a flag, and `loop()` acts on it at its next pass. It is never busy and never cached. A wrong or missing fleet key: no effect, counted (event 10). The PC sends it 5 times, 50 ms apart; broadcast is allowed for STOP only (RESTART stays unicast).
- The stop is held in RAM only: a restart with no RC signal enters mode 2 again, as before.
- Each run start, run stop and STOP is event 11 in the vehicle log.

## Telemetry log

The mower is the source of truth. It numbers every record and keeps the records in RAM. A collector on a PC receives the live records and requests the ones it missed. Neither side waits for the other: the mower never waits for an answer, and it sends live records as broadcast frames, so a monitor that is off costs the mower nothing.

### Records, streams and numbers

- **boot_id:** u32, read from NVS and increased by 1 at each boot of the ESP32. Boots therefore sort in order.
- **Streams:** each record belongs to one of two streams. Each stream has its own record number.
  - Stream 0, FAST: sample blocks, 7 samples a record (5 with the IMU). The sample rate is a setting, 1 to 100 Hz (see "Sample rate").
  - Stream 1, SLOW: one status record a second, plus event records.
- **seq:** u32 record number in its stream. The first record of a boot has seq 0. The mower never clears a stream during a boot, so (boot_id, stream, seq) names one record for good.
- **ms:** ESP32 `millis()` when the record was made. For a sample block it is the time of the first sample.

### LOG 0x40

| Offset | Field | Type |
|---|---|---|
| 0 | type, 0x40 | u8 |
| 1 | stream, 0 or 1 | u8 |
| 2 | boot_id | u32 |
| 6 | seq | u32 |
| 10 | ms | u32 |
| 14 | body type | u8 |
| 15 | body | see below |

Live: the mower broadcasts each record once, when it makes it. Backfill: the mower sends the records that a REQ asks for, unicast to the sender of the REQ. A backfill frame is the same bytes as the live frame.

### Body 0x20: sample block (stream 0)

u8 body type 0x20, u8 n (1 to 7), then n samples of 32 bytes. One sample is taken for each STM32 packet that passes its CRC check, so about 100 a second. A packet that fails the CRC check gives no sample; the status record counts them.

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | time after the record's ms | u16 | ms |
| 2 | sensor 1 x (`s1_x`) | i16 | mm, clamped to +-32767 |
| 4 | sensor 1 y (`s1_y`) | i16 | mm, clamped |
| 6 | sensor 1 radius (`s1_radius`) | u16 | mm, 65535 = 65.535 m or more |
| 8 | flags | u8 | bit 0 signal lost, 1 overload, 2 signal disabled, 3 signal weak (sensor 1 status bits); 4 entry beacon, 5 exit beacon (below the threshold); 6 charging; 7 wheel speeds fresh (under 60 ms old) |
| 9 | autopilot | u8 | high 4 bits state (0 roam, 1 seek, 2 acquire, 3 follow, 4 depart, 5 halted), low 4 bits sub-step; 0xFF autopilot off |
| 10 | drive command to the ODrives | i16 | 1/10000 of full, forward positive |
| 12 | steer command to the ODrives | i16 | 1/10000 of full, as `odrive_steer_command` |
| 14 | wheel speed, node 0 (even side) | i16 | 0.001 rev/s, ODrive encoder estimate as received |
| 16 | wheel speed, node 1 (odd side) | i16 | 0.001 rev/s |
| 18 | gyro z | i16 | 0.02 deg/s; -32768 = no gyro. Logged only: the autopilot does not use the gyro |
| 19 | Wi-Fi link (`radio_mode` `wifi`, `lib/wifi_telem`) | 1 associated, 2 association lost, 3 server address learned, 4 no server beacon and no mDNS (multicast DNS) answer 30 s after the association | 1: received signal strength, dBm; 3: server port; 4: 30 | 1: `ssid=<ssid> ip=<ip> ch=<channel>`; 2: `ssid=<ssid>`; 3: `server <ip>:<port> via beacon` or `via mdns`; 4: `no telemetry server on this network`. The text never holds the password. |
| 20 | beacon 1 (`s1_btx_radius[0]`) | u8 | 4 mm, 255 = 1020 mm or more |
| 21 | beacon 2 (`s1_btx_radius[1]`) | u8 | 4 mm |
| 22 | sensor 1 coil 1, real | bf16 | `coil_data[0][0]`, bfloat16: the top 16 bits of the float32, rounded to nearest even (relative error 0.4 % at most; the float32 range, because the Goertzel output's scale is not fixed) |
| 24 | sensor 1 coil 1, imaginary | bf16 | `coil_data[0][1]` |
| 26 | sensor 1 coil 2, real | bf16 | `coil_data[0][2]` |
| 28 | sensor 1 coil 2, imaginary | bf16 | `coil_data[0][3]` |
| 30 | crossing point X, filtered (autopilot) | i16 | mm, left positive; -32768 = no reading |

Size: 14 + 2 + 7 x 32 = 240 bytes a frame, about 14.3 frames a second, about 27 kbit/s.

With the STM32 link version 2 (firmware 0.5.45 and later), bytes 22 to 29 hold sensor 1 coil 1, sensor 1 coil 2, sensor 2 coil 1 and sensor 2 coil 2 at the wire bin of each sensor, in STM32 magnitude units (2^-24 LSB). From 0.5.72 they are signed (the STM32 sign of each coil value); up to 0.5.71 they held the absolute value.

### Body 0x22: sample block with the six IMU axes (stream 0)

The mower sends 0x22 in place of 0x20 when it has an MPU6050 (or in a bench build with `BENCH_FAKE_IMU`). u8 body type 0x22, u8 n (1 to 5), then n samples of 42 bytes. Bytes 0 to 31 of a sample are the 0x20 sample above (gyro z at byte 18). Then:

| Offset | Field | Type | Unit |
|---|---|---|---|
| 32 | accelerometer x | i16 | mg, clamped to +-32767 |
| 34 | accelerometer y | i16 | mg |
| 36 | accelerometer z | i16 | mg |
| 38 | gyro x | i16 | 0.02 deg/s, as gyro z |
| 40 | gyro y | i16 | 0.02 deg/s |

Axes and signs are those of the MPU6050 as mounted (`MPU6050_light` getAccX/Y/Z in g, getGyroX/Y/Z in deg/s). A block holds one type only: when the type changes, the mower stores the open block as it is (it can hold fewer samples), and the next block has the new type.

Size: 14 + 2 + 5 x 42 = 226 bytes a frame, 20 frames a second, about 36 kbit/s.

Bench fill (`BENCH_FAKE_IMU`): static values, different on each axis so that a swapped field shows: 0.125, -0.250, 1.000 g and 1.50, -2.50, 3.50 deg/s (x, y, z). The status record sets flag bit 6 for these values; a reader must not use them as measurements.

### Sample rate (bodies 0x24 and 0x25)

The setting `telemetry_rate_hz` sets the sample rate of the log: 100, 50, 25, 20, 10, 5, 4, 2 or 1 Hz. The default is 20 Hz. A change applies at once, with no restart. Each mower has its own rate.

At 100 Hz, the mower sends bodies 0x20 and 0x22 as above. Below 100 Hz, it sends body 0x24 in place of 0x20, and 0x25 in place of 0x22. These bodies have one more header byte: u8 body type, u8 n, u8 rate in Hz, then n samples. The samples have the same layout as in 0x20 and 0x22. A sample time is the block time plus the sample offset, at every rate.

Below 100 Hz, an anti-alias filter runs before the mower keeps a sample. The filter is two moving averages in series, each over one output period. It has no overshoot and no ringing. Every channel has the same delay, and the sample time already includes it. The filter does not average these fields:

| Field | Value in the output sample |
|---|---|
| Flags | A flag is on when any packet of the output period had it on. |
| Autopilot state | The last value of the period |
| Wire radius, gyro z, crossing point X | The average of the valid packets only. With no valid packet, the field shows "no reading". |
| Beacon distances | The average after a limit at 1020 mm |

At 20 Hz, the frames carry one fifth of the 100 Hz data.

### Body 0x2E: coil magnitudes of all 8 bins (stream 0)

From firmware 0.5.66, with the STM32 link version 2. The telemetry app draws one plot per bin from it. From 0.5.68 the setting `telemetry_bins_hz` sets the rate (default 5 Hz; 0 = off; the same rates as `telemetry_rate_hz`), and the same anti-alias filter as the samples runs first (see "Sample rate"). The record time is the filter output time. In 0.5.66 the record was a plain average over 200 ms, with the window start as its time.

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | body type 0x2E | u8 | |
| 1 | filter factor N (0.5.66: frames averaged) | u8 | 100 Hz / rate; 255 at most |
| 2 | output period (0.5.66: window) | u16 | ms |
| 4 | bin 0 to 7 frequency | 8 x u16 | Hz / 2; 0 = bin off |
| 20 | bin 0 to 7 coil values | 8 x 4 x i24 | per bin: sensor 1 coil 1, sensor 1 coil 2, sensor 2 coil 1, sensor 2 coil 2 |

A coil value is the STM32 magnitude in LSB of 2^-24 (about 1/mm for the wire bin at unit gain): the filtered absolute value, with the sign of the filtered value (0.5.66: the mean of the absolute values, with the sign of their sum, as the USB `STMCOIL8` line). 0x800000 (-8388608) means no data: the sensor is on no STM32 lane, or the bin is off.

Size: 14 + 116 = 130 bytes a frame; at 5 Hz about 5 kbit/s.

### Body 0x21: status (stream 1, once a second)

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | body type 0x21 | u8 | |
| 1 | drive mode | u8 | 0 manual, 1 cruise, 2 autonomous |
| 2 | charge state machine step | i8 | `charge_status` |
| 3 | vehicle status | u8 | 0 stuck, 1 driving, 2 charging |
| 4 | battery voltage | u16 | mV |
| 6 | charge voltage | u16 | mV |
| 8 | charge current | i16 | mA |
| 10 | 5 V rail | u16 | mV |
| 12 | STM32 temperature | i8 | deg C |
| 13 | autopilot state | u8 | as in the sample, 0xFF off |
| 14 | autopilot sub-step | u8 | |
| 15 | flags | u8 | bit 0 go home, 1 gyro present, 2 CAN running, 3 ESP-NOW started, 4 `rover_enabled`, 5 speed lock, 6 IMU values are bench fill (`BENCH_FAKE_IMU`), not measurements, 7 moving: not standing still by the mower's own test (the test that refuses stationary-only settings and `save`) |
| 16 | STM32 status bits | u32 | |
| 20 | STM32 charge code | u32 | |
| 24 | CRC errors, total | u16 | wraps |
| 26 | UART overruns, total | u16 | wraps |
| 28 | packets in the last second | u16 | |
| 30 | longest packet handling in the last second | u16 | us |
| 32 | bounces | u16 | since mode 2 started |
| 34 | turnarounds | u16 | |
| 36 | charges | u16 | |
| 38 | ODrive node 0 bus voltage | u16 | mV |
| 40 | ODrive node 0 bus current | i16 | mA |
| 42 | ODrive axis states, nodes 0 to 5 | 6 x u8 | ODrive AxisState |
| 48 | ODrive axis errors, OR of all nodes | u32 | |
| 52 | nodes with an axis error | u8 | bit n = node n |
| 53 | nodes with a heartbeat in the last second | u8 | bit n = node n |
| 54 | free heap | u32 | bytes |
| 58 | ESP-NOW send failures, total | u16 | wraps |
| 60 | backfill frames sent in the last second | u16 | |
| 62 | crossing point X, filtered | i16 | mm; -32768 = none |
| 64 | wire distance, filtered | u16 | mm |
| 66 | wire angle, filtered | u8 | deg |
| 67 | seconds since the last MON frame from a monitor (0.5.21 and later) | u8 | s; 255 = none, or 255 s and more |
| 68 | PC time at this record, seconds (0.5.22 and later; from MON frames) | u32 | unix s; 0 = unknown |
| 72 | PC time at this record, milliseconds part | u16 | ms |
| 74 | firmware version major, minor, patch (0.5.24 and later) | 3 x u8 | for example 0, 5, 24 |

### Body 0x31: vehicle name (stream 1)

From firmware 0.5.69: every 10 s, and at once after a change of the setting `vehicle_name`.

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | body type 0x31 | u8 | |
| 1 | vehicle number (`vehicleID`) | u8 | |
| 2 | name length n | u8 | 16 at most |
| 3 | name | n bytes | ASCII: A-Z, a-z, 0-9, space, `_`, `-` |

### Body 0x30: event (stream 1)

u8 body type 0x30, u16 code, i32 a, i32 b, then ASCII text to the end of the frame (no terminator).

| Code | Event | a | b | Text |
|---|---|---|---|---|
| 1 | boot (always record 0 of stream 1) | reset reason (`esp_reset_reason()`) | previous boot_id (boot_id - 1; 0 on the first boot) | `fw=<version> cfg=<crc32 of the settings, hex>` |
| 2 | drive mode change | old | new | |
| 3 | autopilot state change | old | new | state names |
| 4 | stuck detector on | | | |
| 5 | charge step change | old | new | |
| 6 | ODrive axis error | node | error bits | |
| 7 | autopilot halted (wire lost too long) | | | |
| 8 | setting command (`set` or `save`), applied or refused | result (0 ok, 1 unknown key, 2 bad value, 4 refused) | source: 0 USB, 1 ESP-NOW | `<command> -> <reply>` |
| 9 | stall stop: the autopilot halted because the wheels did not turn under a drive command | | | `stalled: halted` |
| 10 | CMD or RESTART refused for a wrong or missing fleet key | count since boot | | |
| 11 | run start, run stop or STOP | 1 start, 0 stop | result (0 ok, 4 refused) | source and answer, e.g. `STOP frame -> run=stopped` |
| 12 | IMU at boot | accelerometer range, g (0 = no IMU) | gyro range, deg/s | gyro offsets, filter, or `absent (init <code>)` |
| 18 | STM32 command link (0.5.25 and later, `lib/stm_settings`) | kind: 1 no answer after the last send, 2 wrong answer, 3 a read-back differs (all five settings sent again), 4 packets back after a gap of 0.5 s or more (all five sent again), 5 command queue full, 6 HELLO/BOOT sent after 1 s without packets | kinds 1, 2, 3, 5: command ID (bit 31 the write bit); 4: the gap, ms; 6: 0 boot ok, 1 no HELLO answer, 2 BOOT failed, 3 no nDATA_READY, -1 line not silent, no HELLO sent | for example `read-back cmd 6 got 0.000 want 11.900: send all again`, `silent 1004 ms, 13 B: boot ok; hello 7 B: 00 00 01 00 00 01 00` |

### REQ 0x41

| Offset | Field | Type |
|---|---|---|
| 0 | type, 0x41 | u8 |
| 1 | stream | u8 |
| 2 | boot_id | u32 |
| 6 | from_seq | u32 |
| 10 | mask | u64 |

Bit i of the mask asks for record from_seq + i. The mower sends each asked record that it still holds, oldest first, as a LOG frame unicast to the REQ's sender. It skips records that it no longer holds; INFO shows what it holds. It sends at most 50 backfill frames a second, from `loop()`. It keeps up to 8 REQs. A REQ that arrives when 8 wait is dropped (counted as req_dropped); the collector sends it again after 2 s.

### INFO 0x42

Broadcast once a second while ESP-NOW is on, also when a stream is empty.

| Offset | Field | Type |
|---|---|---|
| 0 | type, 0x42 | u8 |
| 1 | boot_id | u32 |
| 5 | previous boot_id (0 = none held) | u32 |
| 9 | stream 0, this boot: oldest seq, newest seq | 2 x u32 |
| 17 | stream 1, this boot: oldest, newest | 2 x u32 |
| 25 | stream 0, previous boot: oldest, newest | 2 x u32 |
| 33 | stream 1, previous boot: oldest, newest | 2 x u32 |

A boot that made no record in a stream has oldest 0xFFFFFFFF and newest 0xFFFFFFFF for it. A boot that made records but holds none of them any more has oldest = newest + 1, where newest is the last seq it made (telemlog 312d827): a collector then knows how far a crashed boot went.

### FLASHINFO 0x43

Broadcast every 5 s once the flash store is attached. The flash keeps SLOW records over a power cycle and over many boots; INFO shows only two boots.

| Offset | Field | Type |
|---|---|---|
| 0 | type, 0x43 | u8 |
| 1 | n, at most 20 | u8 |
| 2 | n entries, newest boot first: boot_id, oldest seq, newest seq | n x (3 x u32) |

A collector treats each entry as a SLOW range that REQ can fetch. It asks for the boots that INFO does not cover after the gaps that INFO covers, oldest data first.

### Storage on the mower

- Each stream has its own ring of fixed slots in RAM with `__NOINIT_ATTR`. The ring keeps its contents through a software restart, a panic and a watchdog reset. It does not keep them through a power cycle (the mower checks a magic word and a CRC per slot, and starts empty after a power-on reset).
- A new boot writes after the previous boot's newest record. The previous boot's records stay readable until new records overwrite them. INFO shows the part that is left. These are the records from just before a crash.
- Depth (build flags `TELEMLOG_FAST_SLOTS`, `TELEMLOG_SLOW_SLOTS` in `platformio.ini`; slots of 244 B and 96 B, `lib/telemlog`):
  - FAST: 10 slots, 70 samples: 3.5 s at 20 Hz. Older FAST records are gone. A collector that was off longer shows the span as "not available".
  - SLOW: 24 slots, about 24 s at 1 status record a second, less the event records. The flash copy of SLOW keeps about 1.5 h.
  - Since 0.5.21 the rings are a tenth of their earlier size (design choice: the RAM is worth more than the backfill). The firmware uses 73.3 KB of static RAM.
- A collector removes duplicates by (MAC, boot_id, stream, seq).

### USB serial commands for the log

`log info` prints the INFO fields. `log dump <stream> <from_seq> <count>` prints each held record as one line `LOG <hex of the LOG frame>`, paced so that no line is lost. `restart` restarts the ESP32 (software reset: the rings survive).
