# STM32 app update

Encrypted and signed update of the STM32 sensor-board app 2.0.47, for the update through the ESP32 (USB bridge to the boot checker on the board). Files: `.commit_header` (52 B) and `.bin.encrypted`. The board accepts only signed images; this folder holds no STM32 source.

App 2.0.39 and later restart into the boot checker by command, so ESP32 firmware 0.5.65 and later updates the STM32 app with no power cycle (`stm restart`).

App 2.0.44 makes the bins narrower at reading times from about 25 ms to 105 ms (-3 dB at +/-19 Hz at 100 ms), with the same magnitude scale. At 10 ms (the default) and 20 ms the output is the same as 2.0.39.

App 2.0.47 adds the raw sample stream (command 42): the samples of one sensor (both coils) at the full rate, through the ESP32 to USB (ESP32 firmware 0.5.76 and later, USB command `stm raw`).
