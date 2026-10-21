# Rx STM32 app update

Encrypted and signed update of the Rx STM32 app 2.2.2, for the update through the ESP32 (USB bridge to the boot checker on the board). Files: `.commit_header` (52 B) and `.bin.encrypted`. The board accepts only signed images; this folder holds no Rx STM32 source.

App 2.2.2 sends the 144 B frame (version 3) and, in every frame, the polarity per sensor and Tx channel (extension item 12): +1 when the dual tone field points along the field that gives x and y, -1 when opposite, 0 unknown. Polarity uses both coils, so the sensor angle has no effect. The field sign (item 9) and the sign spread (item 10) stay. The sign spread limit is a command (54, index 14). ESP32 firmware 0.6.21 and later reads these items and gives inside or outside and the heading round the loop from them with the current direction setting of the Tx channel. The default update period is 10 ms. The app restarts into the update loader on the ESP32 command (command 41), so the ESP32 updates it with no power cycle.

ESP32 firmware 0.5.65 and later updates the Rx STM32 app with no power cycle (`stm restart`).
