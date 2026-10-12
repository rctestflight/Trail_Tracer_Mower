// usb_guard: USB CDC protection for ESP32-S2 Arduino firmware (arduino-esp32 2.0.17, TinyUSB 0.16).
// 1. USBCDC::_onLineState runs a reboot state machine on DTR/RTS changes: a host close that leaves
//    DTR off / RTS on is step 1 and clears Serial's "connected"; steps 3 and 4 enter the bootloader
//    and left the USB dead (bench tests). begin() turns the machine off.
// 2. restart(): a restart with a 300 ms USB detach first, so the host sees the board leave.
// 3. service(): the USB output stop. The board runs and takes PC input, the port is open (DTR on),
//    but no USB transmit transfer completes, so the 64-byte TX FIFO stays full and all output is
//    lost (monitor diagnostics: no TX-complete events for many seconds; a
//    healthy board has about 200 a second). Detection: TX FIFO full, port open, mounted, not
//    suspended, no TX-complete event for STUCK_MS. Action: a USB reconnect with no restart (D+
//    pull-up off for DETACH_MS, then on; the host enumerates the board again). Non-blocking, so a
//    moving vehicle keeps its drive commands. A host that opens the port and stops reading also
//    fills the FIFO; it gets a reconnect each STUCK_MS + DETACH_MS + GRACE_MS.
// Removed 2026-10-10 (0.5.8): the self-restarts after a transmit stall and after a lost host. They
// restarted a monitor board during updates, and a parked vehicle after a USB unplug.
#pragma once
#include <Arduino.h>
#include "esp32-hal-tinyusb.h"
#include "tusb.h"

namespace usb_guard {

constexpr uint32_t DETACH_MS = 300;   // the PC sees the removal
constexpr uint32_t STUCK_MS = 3000;   // TX FIFO full with no completed transfer this long
constexpr uint32_t GRACE_MS = 6000;   // after a reconnect: the host enumerates and opens the port

void begin();  // before Serial.begin(): reboot machine off, TX-complete counter on

// Restart with the USB detach first. flush: hands queued text to the host before the detach.
inline void restart(void (*flush)() = nullptr) {
  if (flush) flush();
  tud_disconnect();
  delay(DETACH_MS);
  ESP.restart();
}

void service(uint32_t now_ms, bool stationary);  // once per loop() pass; never blocks
bool reconnecting(uint32_t now_ms);  // a guard reconnect is in progress (USB down on purpose)
inline void progress() {}
uint32_t restarts();     // USB reconnects by the guard
inline uint32_t full_resets() { return 0; }
uint32_t stalls();       // output stops detected
uint32_t tx_events();    // TX-complete events since begin()

}  // namespace usb_guard
