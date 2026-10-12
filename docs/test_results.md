# Test results: v2.0.0-test.1

Release v2.0.0-test.1: ESP32 firmware 0.5.57 on the branch `feature/rx-v2-firmware`, tested with the STM32 sensor-board app 2.0.20 and with the shipped v1 STM32 app 0.1.0. All tests ran on two bench Rx boards with no vehicle, no ODrive motor controller and no IMU.

## ESP32 firmware 0.5.57

| Check | Result |
|---|---|
| Soak: 30 min at 10 ms frames, radio and log on | No reset. 1799 of 1800 heartbeats. 942 of 942 STM32 commands answered. Frame loss 0.1 %. Log: 916 flash pages written, 0 write or erase failures; 7075 radio log frames sent, 0 send failures |
| Update over USB with the PlatformIO Upload button (firmware 0.5.55, the same upload code) | Pass (116 s, the new image confirmed) |
| Full install with the Upload button, from this firmware (0.5.55, the same upload code) | Pass (download mode with no button, full erase in 15 s, 4 images verified) |
| Full install from a main-branch build, Linux PC (0.5.56, the same upload code) | Pass: no answer to `boot`, download mode with no button, erase and write, then a full power-off |
| STM32 app update through the ESP32 (0.5.56 and 0.5.57) | Pass (2.0.20 and 0.1.0, each with its power-on step) |
| Shipped v1 STM32 app 0.1.0, cold start | 58 of 58 v1 commands answered; 100 packets a second |
| Fast STM32 frames left from before an ESP32 restart | The ESP32 sets the 10 ms period before its main loop starts; then 100 packets a second |
| Fastest STM32 period (2336 us, 2 sensors, battery measurement off), 10 min | No reset; the ESP32 handles 401 to 417 of 428 frames a second |

## STM32 app 2.0.20 (sensor board DSP)

Set-ups: S2H = 2 sensors with the battery and charge measurement (the default); S1H = 1 sensor with it; S2N and S1N = the same with it off.

| Set-up | Result |
|---|---|
| S2H (default) | Pass. Fastest frame period 2354 us. CPU load 60.1 %. Position noise at 10 ms: 0.031 mm at 2 Vpp, 2.47 mm at 0.27 Vpp. Rejection of other frequencies 80 dB or more |
| S1H | Pass on timing, CPU load, noise, rejection and gain tests. Fastest frame period 2354 us; 10 min there with 0 overruns. CPU load 63.1 % |
| S2N | Signal tests pass: noise 0.085 mm at 10 ms and 0.014 mm at 100 ms (2 Vpp); rejection 75.8 dB or more; overload recovery pass. CPU load 59.8 %. Long overrun test in progress |
| S1N | Signal tests ran; no-signal tests in progress |

## Not tested on the bench

- An ODrive motor controller and a real rover.
- The full install from V1.0 with the 0 button steps on a Windows PC, and the way back to V1.0.
