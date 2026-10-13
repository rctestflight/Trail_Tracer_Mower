# STM32 app update

Encrypted and signed update of the STM32 sensor-board app 2.0.39, for the update through the ESP32 (USB bridge to the boot checker on the board). Files: `.commit_header` (52 B) and `.bin.encrypted`. The board accepts only signed images; this folder holds no STM32 source.

App 2.0.39 adds the restart into the boot checker by command, so ESP32 firmware 0.5.65 and later updates the STM32 app with no power cycle (`stm restart`).
