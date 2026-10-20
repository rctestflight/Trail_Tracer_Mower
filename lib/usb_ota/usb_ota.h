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
//   ota url <http://a.b.c.d[:port]/path> <size> <sha256 hex> [confirm]
//                                             the same update, fetched over Wi-Fi: an HTTP/1.0 GET on a plain
//                                             lwIP socket (no TLS, IP address only) in its own task at the
//                                             caller's priority, through the same writer and image checks as
//                                             "ota w". Only when may_start() and net_ready() are true and the
//                                             running image is valid. Answer "OTA url started ..."; then
//                                             "OTA progress <bytes>/<size>" each second from service(), and
//                                             one last line: "OTA done; restarting ..." (then as "ota end"),
//                                             or "OTA url failed: <reason>; ..." with the boot slot unchanged.
//                                             30 s with no byte from the server ends it; "ota abort" stops it.
//   ota status                                one line: "OTA status state=<idle|download|done|failed> got=<n> size=<n>"
//                                             (the "ota url" download); after a failure error="<reason>"; then the
//                                             running slot, img_state (2 valid, 1 pending verify), gate and the writer.
// Image checks before any flash write: ESP image magic 0xE9, chip ID 2 (ESP32-S2) at offset 12,
// app descriptor magic 0xABCD5432 at offset 0x20, size not over the next slot.
// Rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE): the new image starts "pending verify".
//   Without "confirm": it marks itself valid after 30 s of running.
//   With "confirm": it is valid only after "ota confirm" arrives over the same link, which proves
//   that the link works in both directions on the new image. With no confirm in 120 s it restarts,
//   and the bootloader starts the previous image. A crash before either also returns to it.
//   A hang: begin() starts the RTC watchdog (60 s, system reset) on a new image, for a hang in setup(); the firmware
//   must set it up again or feed it (rtc_wdt_feed) in loop(). In loop() the firmware's own watchdogs act.
#pragma once
#include <Arduino.h>
#include "esp_partition.h"

namespace usb_ota {

struct Hooks {
  Print* out;             // answers ("OTA ..."); must not block
  bool (*may_start)();    // e.g. vehicle stationary; flash writes stop the CPU for tens of ms
  void (*restart)();      // restart with the USB detach first
  bool (*net_ready)();    // "ota url": the network is up (nullptr: "ota url" is refused)
};

void begin(const Hooks& hooks);  // early in setup(): reads the confirm flag of a new image
void command(const char* arg);   // the text after "ota "
void service();                  // each loop(): marks the image valid, or rolls back; "ota url" lines and restart
bool busy();                     // a download runs ("ota url" or fetch_start), or it ended well and is not yet taken
                                 // (also an upload from the update page: push_begin)
bool fetching();                 // busy() for a download that the board fetches ("ota url", fetch_start), not an upload
Print* set_out(Print* out);      // another answer output (e.g. one that also keeps the line for a radio answer); returns the old

// A download into a data partition (the firmware's "stm url"): the same HTTP fetch, task and refusals as "ota url"
// (except the image state), no image check, no slot select, no restart. Each 4 KB sector is erased once, just before
// its first write. At the end the SHA-256 of partition bytes [0, size) is checked. False: refused, reason in err.
bool fetch_start(const char* url, unsigned long size, const char* sha256_hex, const esp_partition_t* part, char* err, size_t cap);
enum FetchResult : uint8_t { FETCH_NONE, FETCH_RUN, FETCH_OK, FETCH_FAILED };
// From loop(): the partition download's state and bytes received. FETCH_OK and FETCH_FAILED come once (reason set).
FetchResult fetch_poll(uint32_t* got, const char** reason);

// An upload that the caller feeds (the firmware's update page, from the web server task): the same writers and checks
// as "ota url" (part == nullptr: the ESP32 image into the next slot) and fetch_start (part: a data partition).
// size 0: not known before the end (a form post); the ESP32 image is then checked against the slot size as it comes.
// sha256_hex nullptr or "": no expected value; the ESP32 image is still checked by its own SHA-256 digest at the slot
// select, and a partition upload by a read-back of the flash against the SHA-256 of the bytes received.
// push_begin from loop() only (it runs may_start: update mode); push_write from the uploading task; push_end from
// either (the firmware runs it in loop(), for the stack of the image check at the slot select).
// A partition upload ends as a fetch_start download (fetch_poll). The ESP32 image, after push_end, waits for
// push_restart (the caller answers its client first); then service() restarts as after "ota url".
bool push_begin(const esp_partition_t* part, unsigned long size, const char* sha256_hex, char* err, size_t cap);
bool push_write(const uint8_t* b, size_t n, char* err, size_t cap);
bool push_end(char* err, size_t cap);
void push_abort(const char* why);  // the client went away: the writer stops, the boot slot stays
void push_restart();
uint32_t push_got();

}  // namespace usb_ota
