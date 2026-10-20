# Drive task: design note

Status: design only, no code. The base is ESP32 firmware 0.5.102. Code work starts after the acceptance run passes.

## Goal

The drive path gets the CPU before the data path, at a set control rate of 100 Hz. This follows the run model: only driving and data run together, and driving has priority.

## Today (0.5.102)

One task, `loopTask` (priority 1), does all the work in one pass: frame read, decode, answer, control, CAN output, radio, USB, log, bench input and the USB drive. The Wi-Fi task (priority 23) and the network task `tiT` (priority 18) interrupt it.

Measured on a bench Rx, Wi-Fi link, vehicle driving, 60 s per case. CPU load comes from the 2 kHz task sampler. Latency is the time from the frame edge to the CAN output.

| Frame period | CPU load | Latency p50 | Latency p99 | Longest pass |
|---|---|---|---|---|
| 10 ms | 46 % | 1.7 ms | 3.7 ms | 15 ms |
| 2.35 ms | 77 % | 1.6 ms | 3.8 ms | 12.5 ms |

A long radio or bench part delays the next control pass by up to 12 ms. No frame is lost, because the catch-up reads every frame. Control, however, runs late in these passes.

## Design

Two tasks on the single core.

| Task | Priority | Work |
|---|---|---|
| Drive task | 5 (above `loopTask`, below the network tasks) | Frame read, decode, answer, control on a 100 Hz grid, CAN and servo output |
| Service task (`loopTask`) | 1 | Radio, USB, log, bench input, USB drive, saves, settings |

1. The frame-edge interrupt wakes the drive task. The drive task reads every frame and runs control on the 100 Hz grid.
2. The drive task alone owns the Rx STM32 serial port and the drive outputs.
3. The service task puts Rx STM32 commands into a queue. The drive task sends at most one command per frame and gives each answer back through a second queue.
4. After each frame, the drive task writes one snapshot: position, bins, status and outputs. A sequence counter guards the snapshot, so the drive task never waits for a lock. The service task reads the newest snapshot for telemetry and the log.
5. The drive task does not print. It counts events, and the service task reports the counts.
6. A save follows the run model. The service task asks the drive task to stop, and the drive task sets the outputs to stop and waits. Then the service task saves, and the drive task resumes.
7. The loop watchdog covers both tasks. The service task feeds the RTC watchdog only while the drive task shows a heartbeat.

## CPU budget of the drive task

This is an estimate from the 0.5.102 per-part times: read 61 us, decode 214 us and answer 87 us per frame; control 0.86 ms and CAN output 0.18 ms per control pass.

| Frame period | Frames per second | Drive task load |
|---|---|---|
| 10 ms | 100 | about 14 % |
| 2.35 ms | 425 | about 26 % |

## Open measurement: the priority

The Wi-Fi task and `tiT` take 9 % to 11 % of the CPU and interrupt priority 5. If the p99 latency with the drive task is not better than today, try priority 19 (above `tiT`, below the Wi-Fi task). The drive task stays short (0.4 ms per frame), so the network tasks keep enough CPU. Measure first, then decide.

## Room for the playground (horizon, no work now)

The open-source playground gives the user one file with `setup()`, `loop()` and a per-frame function. This design keeps room for that file:

- The per-frame function runs in the drive task, after decode and before control, with a measured time budget. An overrun gets a count and a report, not a stop.
- `loop()` runs in the service task.
- The user reads all data from the snapshot.

## Steps

1. Move the frame path into the drive task with no change in behaviour. Measure CPU load, latency and the control rate.
2. Add the command queue and the answer queue.
3. Add the save handshake.
4. Decide the priority from the measurement.

## Acceptance

- 0 lost frames and 0 lost answers at 10 ms and 2.35 ms frames, on ESP-NOW and on Wi-Fi.
- The control rate is 100 Hz, with no more than 1 pass a second too few.
- The p99 latency is no worse than today, and the longest control delay is under 5 ms.
- 20 saves while stopped stop no output and leave no stop of the USB link.
