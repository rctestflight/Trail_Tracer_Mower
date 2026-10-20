# Rx STM32 app update

Encrypted and signed update of the Rx STM32 app 2.1.222, for the update through the ESP32 (USB bridge to the boot checker on the board). Files: `.commit_header` (52 B) and `.bin.encrypted`. The board accepts only signed images; this folder holds no Rx STM32 source.

App 2.1.222 sends the 144 B frame (version 3) and decodes dual tone polarity per Tx channel (side of the wire, sign and spread per sensor) and fills the sensor 2 coil data in the frame. Side detection also works with one sensor; the bin sweep runs low to high in one pass, also across a period change. ESP32 firmware 0.6.11 and later reads these items. The default update period is 10 ms.

ESP32 firmware 0.5.65 and later updates the Rx STM32 app with no power cycle (`stm restart`).
