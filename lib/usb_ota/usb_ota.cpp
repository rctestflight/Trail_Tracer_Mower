#include "usb_ota.h"

#include <Preferences.h>
#include <Update.h>

#include "esp_ota_ops.h"
#include "mbedtls/sha256.h"

// arduino-esp32 (C): true = the app marks the image valid itself (usb_ota::service), not initArduino()
extern "C" bool verifyRollbackLater() { return true; }

namespace usb_ota {
namespace {

Hooks h;
bool active = false, want_confirm = false, need_crc = false;  // need_crc: "begin ... crc", every line has a CRC-32
uint32_t size_ = 0, written = 0;
uint8_t sha_expected[32];
mbedtls_sha256_context sha;
bool pending = false;  // this image runs "pending verify"
bool gate = false;     // and it waits for "ota confirm"
bool done = false;     // valid, or rollback started

int hexval(char c) {
  return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}

bool hex_to(const char* hex, uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    const int a = hexval(hex[2 * i]), b = hexval(hex[2 * i + 1]);
    if (a < 0 || b < 0) return false;
    buf[i] = (uint8_t)(a << 4 | b);
  }
  return true;
}

uint32_t crc32(const uint8_t* p, size_t n) {  // zlib CRC-32 (Python zlib.crc32)
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

void set_confirm_flag(bool on) {
  Preferences p;
  p.begin("usb_ota", false);
  p.putUChar("cfm", on ? 1 : 0);
  p.end();
}

// First chunk only: an image for this chip, with an app descriptor, before any flash write.
const char* image_error(const uint8_t* b, size_t n) {
  if (n < 0x24) return "first chunk shorter than 36 bytes";
  if (b[0] != 0xE9) return "not an ESP image (magic)";
  if ((b[12] | b[13] << 8) != 2) return "not an ESP32-S2 image (chip ID)";
  const uint32_t m = b[0x20] | b[0x21] << 8 | b[0x22] << 16 | (uint32_t)b[0x23] << 24;
  if (m != 0xABCD5432) return "no app descriptor";
  return nullptr;
}

void mark_valid(const char* why) {
  esp_ota_mark_app_valid_cancel_rollback();
  set_confirm_flag(false);
  done = true;
  h.out->printf("OTA image marked valid (%s)\n", why);
}

}  // namespace

void begin(const Hooks& hooks) {
  h = hooks;
  esp_ota_img_states_t st;
  pending = esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
  Preferences p;
  p.begin("usb_ota", true);
  const bool flag = p.getUChar("cfm", 0);
  p.end();
  gate = pending && flag;
  if (!pending && flag) set_confirm_flag(false);  // a rollback happened: the flag belonged to the failed image
}

void command(const char* arg) {
  Print& o = *h.out;
  unsigned long size = 0, off = 0;
  char hex[1100];  // on the stack: static RAM is short on the rover (dram0_0_seg)
  char word[16] = {0}, word2[16] = {0};
  const int nb = sscanf(arg, "begin %lu %64s %15s %15s", &size, hex, word, word2);
  if (nb >= 2) {
    if (!h.may_start()) { o.println("OTA error not now (may_start)"); return; }
    if (strlen(hex) != 64 || !hex_to(hex, sha_expected, 32)) { o.println("OTA error sha256 must be 64 hex"); return; }
    const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
    if (!next || size > next->size) { o.printf("OTA error size %lu over slot %lu\n", size, next ? (unsigned long)next->size : 0UL); return; }
    if (active) Update.abort();
    if (!Update.begin(size, U_FLASH)) { o.printf("OTA error begin %s\n", Update.errorString()); active = false; return; }
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);
    active = true; size_ = size; written = 0;
    want_confirm = !strcmp(word, "confirm") || !strcmp(word2, "confirm");
    need_crc = !strcmp(word, "crc") || !strcmp(word2, "crc");
    o.printf("OTA begin ok size=%lu slot=%s confirm=%d crc=%d\n", size, next->label, (int)want_confirm, (int)need_crc);
    return;
  }
  unsigned long crc = 0;
  const int nw = sscanf(arg, "w %lu %1099s %lx", &off, hex, &crc);
  if (nw >= 2) {
    if (!active) { o.println("OTA error not begun"); return; }
    if (off != written) { o.printf("OTA error offset %lu expected %lu\n", off, (unsigned long)written); return; }
    const size_t n = strlen(hex) / 2;
    if (n == 0 || n > 512 || strlen(hex) % 2 || written + n > size_) { o.println("OTA error chunk"); return; }
    uint8_t buf[512];
    if (!hex_to(hex, buf, n)) { o.println("OTA error hex"); return; }
    // A line that lost bytes on the USB input can still be valid hex: the CRC-32 of the chunk
    // (optional third field) refuses it, and nothing is written.
    if (need_crc && nw != 3) { o.printf("OTA error crc missing at %lu\n", off); return; }
    if (nw == 3 && crc32(buf, n) != (uint32_t)crc) { o.printf("OTA error crc at %lu\n", off); return; }
    if (written == 0) {
      const char* e = image_error(buf, n);
      if (e) { Update.abort(); active = false; o.printf("OTA error image: %s\n", e); return; }
    }
    if (Update.write(buf, n) != n) { o.printf("OTA error write %s\n", Update.errorString()); return; }
    mbedtls_sha256_update_ret(&sha, buf, n);
    written += n;
    o.printf("OTA ok %lu\n", (unsigned long)written);
    return;
  }
  if (!strcmp(arg, "end")) {
    if (!active || written != size_) { o.printf("OTA error end at %lu of %lu\n", (unsigned long)written, (unsigned long)size_); return; }
    uint8_t got[32];
    mbedtls_sha256_finish_ret(&sha, got);
    mbedtls_sha256_free(&sha);
    if (memcmp(got, sha_expected, 32)) { Update.abort(); active = false; o.println("OTA error sha256 mismatch"); return; }
    if (!Update.end(false)) { active = false; o.printf("OTA error end %s\n", Update.errorString()); return; }
    active = false;
    set_confirm_flag(want_confirm);
    o.printf("OTA done; restarting into the new image (confirm=%d)\n", (int)want_confirm);
    h.restart();
    return;
  }
  if (!strcmp(arg, "abort")) { if (active) Update.abort(); active = false; o.println("OTA aborted"); return; }
  if (!strcmp(arg, "confirm")) {
    if (pending && !done) mark_valid("confirm");
    else o.printf("OTA confirm: nothing pending (pending=%d done=%d)\n", (int)pending, (int)done);
    return;
  }
  if (!strcmp(arg, "status")) {
    const esp_partition_t* run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(run, &st);
    o.printf("OTA status running=%s state=%d gate=%d active=%d written=%lu of %lu\n", run->label, (int)st, (int)(gate && !done),
             (int)active, (unsigned long)written, (unsigned long)size_);
    return;
  }
  o.println("OTA usage: ota begin <size> <sha256> [confirm] | ota w <offset> <hex> | ota end | ota abort | ota status | ota confirm");
}

void service() {
  if (!pending || done) return;
  const uint32_t ms = millis();
  if (!gate && ms >= 30000) { mark_valid("30 s"); return; }
  if (gate && ms >= 120000) {
    done = true;
    h.out->println("OTA no confirm in 120 s: restart; the bootloader starts the previous image");
    h.restart();
  }
}

}  // namespace usb_ota
