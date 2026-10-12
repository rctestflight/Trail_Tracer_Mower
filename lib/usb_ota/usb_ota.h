// usb_ota: firmware self-update over a text line link (USB serial), for any ESP32-S2 Arduino
// firmware with two OTA slots. No ROM download mode, no button, no power cycle.
// The firmware passes each line that starts with "ota " to usb_ota::command(); every command line
// is answered with exactly one line that starts with "OTA".
//   ota begin <size> <sha256 hex> [confirm] [crc]   only when hooks.may_start() is true; "crc":
//                                             every "ota w" line must carry the chunk's CRC-32
//   ota w <offset> <hex> [crc32 hex]          at most 512 bytes; offset = bytes written so far;
//                                             with the zlib CRC-32 of the chunk, a damaged line is refused
//                                             (after a lost answer, "ota status" gives the offset)
//   ota end                                   checks size and SHA-256, selects the slot, restarts
//   ota abort | ota status | ota confirm
// Image checks before any flash write: ESP image magic 0xE9, chip ID 2 (ESP32-S2) at offset 12,
// app descriptor magic 0xABCD5432 at offset 0x20, size not over the next slot.
// Rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE): the new image starts "pending verify".
//   Without "confirm": it marks itself valid after 30 s of running.
//   With "confirm": it is valid only after "ota confirm" arrives over the same link, which proves
//   that the link works in both directions on the new image. With no confirm in 120 s it restarts,
//   and the bootloader starts the previous image. A crash before either also returns to it.
#pragma once
#include <Arduino.h>

namespace usb_ota {

struct Hooks {
  Print* out;             // answers ("OTA ..."); must not block
  bool (*may_start)();    // e.g. vehicle stationary; flash writes stop the CPU for tens of ms
  void (*restart)();      // restart with the USB detach first
};

void begin(const Hooks& hooks);  // early in setup(): reads the confirm flag of a new image
void command(const char* arg);   // the text after "ota "
void service();                  // each loop(): marks the image valid, or rolls back

}  // namespace usb_ota
