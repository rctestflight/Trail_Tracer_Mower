// Update page on the ESP32 (0.5.116): firmware updates from a browser, through the web server (web_server.h).
//   GET  /update            the page: two file inputs (Rx STM32 file, ESP32 image), one button, a result line
//   GET  /update/status     JSON: versions, the update in progress, the last result
//   POST /update            form post (multipart/form-data), fields "rxstm32" and "esp32"; either may be empty
//   POST /update/rxstm32    raw body: one Rx STM32 file (52 B header + encrypted image); optional ?sha256=<64 hex>
//   POST /update/esp32      raw body: one ESP32 image; optional ?sha256=<64 hex>
// The same writers, checks and refusal texts as "stm url" and "ota url": the vehicle must be stationary, one update at
// a time, the Rx STM32 file first, then the ESP32 image and its restart. The BOOT button and the boot checkers stay
// the recovery; this page is a convenience.
#pragma once
#include <stdint.h>

namespace webupd {

void setup();        // from setup(): adds the page to the web server
void loopService();  // from loop(), every pass in every branch: grants an upload start in loop() context

}  // namespace webupd
