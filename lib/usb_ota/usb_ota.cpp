#include "usb_ota.h"

#include <Preferences.h>
#include <Update.h>

#include "esp_flash.h"
#include "esp_ota_ops.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "mbedtls/sha256.h"
#include "soc/rtc_wdt.h"

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

// "ota url": the download task writes dl_state, dl_got and dl_err; loop() (service) reports them.
// DL_FAIL: failed, not yet reported; DL_FAILED: reported (dl_err stays for "ota status").
enum : uint8_t { DL_IDLE, DL_RUN, DL_OK, DL_FAIL, DL_FAILED };
volatile uint8_t dl_state = DL_IDLE;
volatile uint32_t dl_got = 0;      // body bytes received
volatile bool dl_abort = false;    // "ota abort" during a download
uint32_t dl_t0 = 0, dl_ms = 0, dl_print_ms = 0;
uint32_t dl_ip = 0;                // network order
uint32_t dl_size = 0;              // body bytes asked (the command's size)
// What the download feeds: the ESP32 image writer ("ota url"), or a data partition (fetch_start, the firmware's
// "stm url"). A partition download erases each 4 KB sector once, just before its first write.
enum : uint8_t { K_ESP, K_PART };
uint8_t dl_kind = K_ESP;
const esp_partition_t* dl_part = nullptr;
uint32_t part_written = 0, part_erased = 0;
uint16_t dl_port = 80;
char dl_host[24], dl_path[128], dl_err[96];
unsigned dl_stack_free = 0;        // least free stack of the download task, bytes
constexpr size_t DL_BUF = 2048;    // receive buffer (Update keeps its own 4 KB flash buffer); 2 KB: heap for Wi-Fi
size_t dl_buf_len = DL_BUF;         // the buffer in use: 2048 B, or 1024 B when the heap has no 2 KB block
constexpr int DL_RCVBUF = 2 * 1436; // the socket keeps at most 2 TCP segments waiting (Wi-Fi receive buffers)
uint32_t dl_heap[5];                // free heap: command, after Update.begin, after the task start, connected, buffer
constexpr uint32_t DL_STALL_MS = 30000;
// An upload fed by the caller (push_begin): the same writers; dl_got counts the bytes, dl_size 0 = not known yet.
bool dl_push = false;              // the download state belongs to an upload, not to "ota url" or fetch_start
bool sha_given = true;             // sha_expected holds a value from the client
uint8_t push_head[64];             // ESP32 image: the first bytes, held until the image check can run on 36 B or more
size_t push_hn = 0;
const char URL_ERR[] = "OTA error usage: ota url http://a.b.c.d[:port]/path <size> <sha256> [confirm]";
const char DL_RUNS[] = "OTA error a download runs (ota abort stops it)";

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

// The writer of "ota begin / w / end" and "ota url". Each returns false with the reason in err.
bool start_image(unsigned long size, char* err, size_t cap, const char** slot) {
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  if (!next || size > next->size) {
    snprintf(err, cap, "size %lu over slot %lu", size, next ? (unsigned long)next->size : 0UL);
    return false;
  }
  // An earlier writer that stopped part way (a page upload cut off) can leave the Update writer open with active
  // false: Update.begin then refuses with no error until a restart. Close it first.
  if (active || Update.isRunning()) Update.abort();
  if (!Update.begin(size, U_FLASH)) { snprintf(err, cap, "begin %s", Update.errorString()); active = false; return false; }
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts_ret(&sha, 0);
  active = true; size_ = size; written = 0;
  *slot = next->label;
  return true;
}

bool write_chunk(const uint8_t* buf, size_t n, char* err, size_t cap) {
  if (written == 0) {
    const char* e = image_error(buf, n);
    if (e) { Update.abort(); active = false; snprintf(err, cap, "image: %s", e); return false; }
  }
  if (Update.write((uint8_t*)buf, n) != n) { snprintf(err, cap, "write %s", Update.errorString()); return false; }
  mbedtls_sha256_update_ret(&sha, buf, n);
  written += n;
  return true;
}

// Size and SHA-256, then the slot select (Update.end checks the image again); the confirm flag for the new image.
// at_received: an upload of unknown size; Update.begin had the slot size, the image ends at the bytes received.
bool finish_image(char* err, size_t cap, bool at_received = false) {
  if (!active || written != size_) {
    snprintf(err, cap, "end at %lu of %lu", (unsigned long)written, (unsigned long)size_);
    return false;
  }
  uint8_t got[32];
  mbedtls_sha256_finish_ret(&sha, got);
  mbedtls_sha256_free(&sha);
  if (sha_given && memcmp(got, sha_expected, 32)) { Update.abort(); active = false; snprintf(err, cap, "sha256 mismatch"); return false; }
  if (!Update.end(at_received)) { active = false; snprintf(err, cap, "end %s", Update.errorString()); return false; }
  active = false;
  set_confirm_flag(want_confirm);
  return true;
}

void mark_valid(const char* why) {
  esp_ota_mark_app_valid_cancel_rollback();
  set_confirm_flag(false);
  done = true;
  h.out->printf("OTA image marked valid (%s)\n", why);
}

// ── ota url ─────────────────────────────────────────────────────────────────

// Connect with a 10 s limit (a blocking lwIP connect retries for minutes). lwip_fcntl and lwip_select directly:
// the firmware links them already; the VFS select and fcntl cost 1.5 KB of flash.
bool dl_connect(int fd, const sockaddr_in& a) {
  const int fl = lwip_fcntl(fd, F_GETFL, 0);
  lwip_fcntl(fd, F_SETFL, fl | O_NONBLOCK);
  if (connect(fd, (const sockaddr*)&a, sizeof(a)) != 0 && errno != EINPROGRESS) return false;
  fd_set w;
  FD_ZERO(&w);
  FD_SET(fd, &w);
  timeval tv = {10, 0};
  if (lwip_select(fd + 1, nullptr, &w, nullptr, &tv) <= 0) return false;
  int e = 0;
  socklen_t el = sizeof(e);
  getsockopt(fd, SOL_SOCKET, SO_ERROR, &e, &el);
  lwip_fcntl(fd, F_SETFL, fl);
  return e == 0;
}

// The HTTP header ends in buf[0..he): status 200, and Content-Length (if sent) equal to the size asked.
const char* dl_header(char* buf, size_t he, char* err, size_t cap) {
  buf[he - 1] = 0;
  if (strncmp(buf, "HTTP/1.", 7) || he < 12) return "no HTTP answer";
  const int status = atoi(buf + 9);
  if (status != 200) { snprintf(err, cap, "HTTP status %d", status); return err; }
  for (char* l = strchr(buf, '\n'); l; l = strchr(l, '\n')) {
    l++;
    if (!strncasecmp(l, "Content-Length:", 15)) {
      const unsigned long cl = strtoul(l + 15, nullptr, 10);
      if (cl != dl_size) { snprintf(err, cap, "the server file is %lu bytes, the command says %lu", cl, (unsigned long)dl_size); return err; }
    }
  }
  return nullptr;
}

// A partition download: erase each sector once, before its first byte; then write.
bool part_write(const uint8_t* buf, size_t n, char* err, size_t cap) {
  while (part_erased < part_written + n) {
    if (esp_partition_erase_range(dl_part, part_erased, 4096) != ESP_OK) { snprintf(err, cap, "flash erase at %lu", (unsigned long)part_erased); return false; }
    part_erased += 4096;
  }
  if (esp_partition_write(dl_part, part_written, buf, n) != ESP_OK) { snprintf(err, cap, "flash write at %lu", (unsigned long)part_written); return false; }
  part_written += n;
  return true;
}

// The SHA-256 of the bytes in flash (what the caller reads back later), against the command's value.
bool part_finish(char* err, size_t cap) {
  if (part_written != dl_size) { snprintf(err, cap, "end at %lu of %lu", (unsigned long)part_written, (unsigned long)dl_size); return false; }
  uint8_t b[256], got[32];
  mbedtls_sha256_context c;
  mbedtls_sha256_init(&c);
  mbedtls_sha256_starts_ret(&c, 0);
  bool ok = true;
  for (uint32_t off = 0; off < dl_size && ok; off += sizeof(b)) {
    const size_t k = dl_size - off < sizeof(b) ? dl_size - off : sizeof(b);
    ok = esp_partition_read(dl_part, off, b, k) == ESP_OK;
    if (ok) mbedtls_sha256_update_ret(&c, b, k);
  }
  mbedtls_sha256_finish_ret(&c, got);
  mbedtls_sha256_free(&c);
  if (!ok) { snprintf(err, cap, "flash read"); return false; }
  if (memcmp(got, sha_expected, 32)) { snprintf(err, cap, "sha256 mismatch"); return false; }
  return true;
}

bool sink_write(const uint8_t* buf, size_t n, char* err, size_t cap) {
  return dl_kind == K_PART ? part_write(buf, n, err, cap) : write_chunk(buf, n, err, cap);
}

// The download: header, then the body in DL_BUF blocks through write_chunk. nullptr = all bytes written.
const char* dl_receive(int fd, uint8_t* buf, char* err, size_t cap) {
  size_t fill = 0;
  bool head = true;
  uint32_t last_rx = millis();
  for (;;) {
    if (dl_abort) return "stopped by ota abort";
    const int k = recv(fd, buf + fill, dl_buf_len - fill, 0);
    if (k < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        snprintf(err, cap, "receive error %d at %lu of %lu bytes", errno, (unsigned long)dl_got, (unsigned long)dl_size);
        return err;
      }
      if (millis() - last_rx >= DL_STALL_MS) {
        snprintf(err, cap, "no data from the server for 30 s at %lu of %lu bytes", (unsigned long)dl_got, (unsigned long)dl_size);
        return err;
      }
      continue;
    }
    if (k == 0) break;  // HTTP/1.0: the server closes at the end of the body
    last_rx = millis();
    fill += k;
    if (head) {
      size_t he = 0;
      for (size_t i = 3; i < fill && !he; i++)
        if (!memcmp(buf + i - 3, "\r\n\r\n", 4)) he = i + 1;
      if (!he) {
        if (fill == dl_buf_len) return "HTTP header longer than the receive buffer";
        continue;
      }
      if (const char* e = dl_header((char*)buf, he, err, cap)) return e;
      head = false;
      memmove(buf, buf + he, fill - he);
      fill -= he;
      dl_got = fill;
    } else {
      dl_got += k;
    }
    if (dl_got > dl_size) { snprintf(err, cap, "the file is longer than %lu bytes", (unsigned long)dl_size); return err; }
    if (fill == dl_buf_len) {
      if (!sink_write(buf, fill, err, cap)) return err;
      fill = 0;
    }
  }
  if (head) return "the server closed before the end of the HTTP header";
  if (fill && !sink_write(buf, fill, err, cap)) return err;
  if (dl_got < dl_size) {
    snprintf(err, cap, "the file ended at %lu of %lu bytes (truncated)", (unsigned long)dl_got, (unsigned long)dl_size);
    return err;
  }
  return nullptr;
}

const char* dl_run(char* err, size_t cap) {
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(dl_port);
  a.sin_addr.s_addr = dl_ip;
  const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) return "no socket";
  const int rcv = DL_RCVBUF;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
  const char* e = nullptr;
  uint8_t* buf = nullptr;
  timeval tv = {1, 0};  // recv returns each second: the stall and abort checks
  char req[200];
  const int n = snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\nHost: %s\r\n\r\n", dl_path, dl_host);
  if (!dl_connect(fd, a)) e = "no connection to the server in 10 s";
  else if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) || setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)))
    e = "socket options";
  else if (send(fd, req, n, 0) != n) e = "request not sent";
  else {
    dl_heap[3] = ESP.getFreeHeap();
    // 4 KB first; a smaller block when the heap is cut up (bench 2026-10-16: no 4 KB block after a stalled download)
    for (dl_buf_len = DL_BUF; dl_buf_len >= 1024 && !(buf = (uint8_t*)malloc(dl_buf_len)); dl_buf_len /= 2) {}
    dl_heap[4] = ESP.getFreeHeap();
    if (!buf) {
      snprintf(err, cap, "no memory for the receive buffer (free heap %lu B, largest block %lu B)",
               (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
      e = err;
    }
  }
  if (buf && !e)
    e = dl_receive(fd, buf, err, cap);
  free(buf);
  close(fd);
  if (e) return e;
  if (dl_kind == K_PART) return part_finish(err, cap) ? nullptr : err;
  return finish_image(err, cap) ? nullptr : err;
}

void dl_task(void*) {
  char err[96];
  const char* e = dl_run(err, sizeof(err));
  dl_ms = millis() - dl_t0;
  if (e) {
    if (dl_kind == K_ESP && active) Update.abort();  // the boot slot stays: only Update.end selects the new slot
    if (dl_kind == K_ESP) active = false;
    snprintf(dl_err, sizeof(dl_err), "%s", e);
  }
  dl_stack_free = uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  dl_state = e ? DL_FAIL : DL_OK;
  vTaskDelete(nullptr);
}

// http://a.b.c.d[:port]/path into dl_ip, dl_port, dl_host, dl_path. False: not this form.
bool url_parse(char* url) {
  unsigned q[4] = {256, 256, 256, 256};
  unsigned long port = 80;
  int pos = 0;
  const char* slash = nullptr;
  if (sscanf(url, "http://%u.%u.%u.%u%n", &q[0], &q[1], &q[2], &q[3], &pos) == 4 && pos > 0) {
    char* r = url + pos;
    if (*r == ':') port = strtoul(r + 1, &r, 10);
    if (*r == '/') slash = r;
  }
  if (!slash || (q[0] | q[1] | q[2] | q[3]) > 255 || port == 0 || port > 65535 || strlen(slash) >= sizeof(dl_path)) return false;
  dl_ip = q[0] | q[1] << 8 | q[2] << 16 | (uint32_t)q[3] << 24;  // network order
  dl_port = (uint16_t)port;
  snprintf(dl_host, sizeof(dl_host), "%u.%u.%u.%u", q[0], q[1], q[2], q[3]);
  snprintf(dl_path, sizeof(dl_path), "%s", slash);
  return true;
}

// The checks both downloads share, in this order. nullptr: passed (the URL is parsed, sha_expected set).
const char* dl_check(char* url, unsigned long size, const char* hex) {
  if (dl_state == DL_RUN || dl_state == DL_OK) return "a download runs (ota abort stops it)";
  if (!h.net_ready || !h.net_ready()) return "Wi-Fi not connected";
  if (strlen(hex) != 64 || !hex_to(hex, sha_expected, 32)) return "sha256 must be 64 hex";
  if (!url_parse(url)) return "usage";
  if (size == 0) return "size 0";
  return nullptr;
}

// The download task. False: no memory for it (dl_err holds the reason).
bool dl_launch() {
  dl_got = 0; dl_abort = false; dl_err[0] = 0;
  dl_t0 = dl_print_ms = millis();
  dl_state = DL_RUN;
  if (xTaskCreate(dl_task, "ota_dl", 5120, nullptr, uxTaskPriorityGet(nullptr), nullptr) == pdPASS) return true;
  dl_state = DL_FAILED;
  snprintf(dl_err, sizeof(dl_err), "no memory for the download task");
  return false;
}

__attribute__((noinline)) void url_command(const char* arg) {
  Print& o = *h.out;
  char url[160], hex[72], word[16] = {0};
  unsigned long size = 0;
  if (sscanf(arg, "%159s %lu %71s %15s", url, &size, hex, word) < 3) { o.println(URL_ERR); return; }
  if (dl_state == DL_RUN || dl_state == DL_OK) { o.println(DL_RUNS); return; }
  if (pending && !done) { o.println("OTA error this image is not marked valid yet"); return; }
  if (const char* e = dl_check(url, size, hex)) {
    if (!strcmp(e, "usage")) o.println(URL_ERR);
    else o.printf("OTA error %s\n", e);
    return;
  }
  if (!h.may_start()) { o.println("OTA error Stop the vehicle first"); return; }
  char err[64];
  const char* slot = "";
  memset(dl_heap, 0, sizeof(dl_heap));
  dl_heap[0] = ESP.getFreeHeap();
  if (!start_image(size, err, sizeof(err), &slot)) { o.printf("OTA error %s\n", err); return; }
  dl_heap[1] = ESP.getFreeHeap();
  want_confirm = !strcmp(word, "confirm");
  need_crc = false;
  dl_kind = K_ESP;
  dl_push = false;
  sha_given = true;
  dl_size = size;
  if (!dl_launch()) {
    Update.abort();
    active = false;
    o.println("OTA error no memory for the download task");
    return;
  }
  dl_heap[2] = ESP.getFreeHeap();
  o.printf("OTA url started size=%lu slot=%s confirm=%d\n", size, slot, (int)want_confirm);
}

// From service(): progress each second, then the one result line (and the restart after a good download).
void dl_service() {
  Print& o = *h.out;
  const uint32_t now = millis();
  if (dl_state == DL_RUN) {
    if (now - dl_print_ms >= 1000) {
      dl_print_ms = now;
      o.printf("OTA progress %lu/%lu heap=%lu maxblk=%lu minheap=%lu\n", (unsigned long)dl_got, (unsigned long)dl_size,
               (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)ESP.getMinFreeHeap());
      static bool heap_said = false;
      if (dl_heap[4] && !heap_said) {
        heap_said = true;
        o.printf("OTA heap command=%lu begin=%lu task=%lu connected=%lu buffer=%lu buf_len=%u\n", (unsigned long)dl_heap[0],
                 (unsigned long)dl_heap[1], (unsigned long)dl_heap[2], (unsigned long)dl_heap[3], (unsigned long)dl_heap[4],
                 (unsigned)dl_buf_len);
      }
    }
  } else if (dl_state == DL_FAIL) {
    dl_state = DL_FAILED;
    o.printf("OTA %s failed: %s; running firmware and boot slot unchanged (ms=%lu stack_free=%u)\n", dl_push ? "upload" : "url", dl_err,
             (unsigned long)dl_ms, dl_stack_free);
  } else if (dl_state == DL_OK) {  // stays DL_OK through the restart: "ota status" reads done
    o.printf("OTA progress %lu/%lu heap=%lu maxblk=%lu minheap=%lu\n", (unsigned long)dl_got, (unsigned long)dl_size,
               (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)ESP.getMinFreeHeap());
    o.printf("OTA done; restarting into the new image (confirm=%d ms=%lu stack_free=%u)\n", (int)want_confirm,
             (unsigned long)dl_ms, dl_stack_free);
    h.restart();
  }
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
  if (pending) {
    // A new image that hangs in setup(): no watchdog runs there (the app start code stops the bootloader's RTC
    // watchdog; the task watchdog checks loop() only). The RTC watchdog resets it after 60 s, and the bootloader then
    // starts the previous image. The firmware's own RTC watchdog set-up replaces this; loop() feeds it.
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
    rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
    rtc_wdt_set_time(RTC_WDT_STAGE0, 60000);
    rtc_wdt_enable();
    rtc_wdt_protect_on();
  }
}

bool busy() { return dl_state == DL_RUN || dl_state == DL_OK; }

Print* set_out(Print* out) {
  Print* old = h.out;
  h.out = out;
  return old;
}

bool fetch_start(const char* url_in, unsigned long size, const char* sha256_hex, const esp_partition_t* part, char* err, size_t cap) {
  char url[160];
  snprintf(url, sizeof(url), "%s", url_in);
  if (const char* e = dl_check(url, size, sha256_hex)) {
    snprintf(err, cap, "%s", !strcmp(e, "usage") ? "url must be http://a.b.c.d[:port]/path" : e);
    return false;
  }
  if (!part || size > part->size) {
    snprintf(err, cap, "size %lu over the partition (%lu)", size, part ? (unsigned long)part->size : 0UL);
    return false;
  }
  if (!h.may_start()) { snprintf(err, cap, "Stop the vehicle first"); return false; }
  dl_kind = K_PART;
  dl_part = part;
  dl_size = size;
  dl_push = false;
  sha_given = true;
  part_written = part_erased = 0;
  if (!dl_launch()) { snprintf(err, cap, "%s", dl_err); return false; }
  return true;
}

FetchResult fetch_poll(uint32_t* got, const char** reason) {
  *got = dl_got;
  *reason = dl_err;
  if (dl_kind != K_PART) return FETCH_NONE;
  if (dl_state == DL_RUN) return FETCH_RUN;
  if (dl_state == DL_OK) { dl_state = DL_IDLE; return FETCH_OK; }
  if (dl_state == DL_FAIL) { dl_state = DL_FAILED; return FETCH_FAILED; }
  return FETCH_NONE;
}

// ── push: an upload fed by the caller ───────────────────────────────────────

namespace {
void push_fail(const char* why) {
  if (dl_kind == K_ESP && active) Update.abort();  // the boot slot stays: only Update.end selects the new slot
  if (dl_kind == K_ESP) active = false;
  if (why != dl_err) snprintf(dl_err, sizeof(dl_err), "%s", why);
  dl_ms = millis() - dl_t0;
  dl_state = DL_FAIL;
}
bool push_sink(const uint8_t* b, size_t n, char* err, size_t cap) {
  if (dl_kind == K_PART) {
    mbedtls_sha256_update_ret(&sha, b, n);  // the bytes as received; push_end compares the flash with them
    return part_write(b, n, err, cap);
  }
  return write_chunk(b, n, err, cap);
}
}  // namespace

bool push_begin(const esp_partition_t* part, unsigned long size, const char* sha256_hex, char* err, size_t cap) {
  if (busy()) { snprintf(err, cap, "a download runs (ota abort stops it)"); return false; }
  if (!part && pending && !done) { snprintf(err, cap, "this image is not marked valid yet"); return false; }
  const bool given = sha256_hex && *sha256_hex;
  if (given && (strlen(sha256_hex) != 64 || !hex_to(sha256_hex, sha_expected, 32))) { snprintf(err, cap, "sha256 must be 64 hex"); return false; }
  if (part && size > part->size) { snprintf(err, cap, "size %lu over the partition (%lu)", size, (unsigned long)part->size); return false; }
  if (!h.may_start()) { snprintf(err, cap, "Stop the vehicle first"); return false; }
  sha_given = given;
  if (part) {
    dl_kind = K_PART;
    dl_part = part;
    part_written = part_erased = 0;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);
  } else {
    const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
    const char* slot = "";
    // Size not known: the whole slot; Update.write refuses bytes past it, and push_end ends at the bytes received.
    if (!start_image(size ? size : (next ? next->size : 0), err, cap, &slot)) return false;
    want_confirm = false;
    need_crc = false;
    dl_kind = K_ESP;
  }
  dl_push = true;
  dl_size = size;
  push_hn = 0;
  dl_got = 0; dl_abort = false; dl_err[0] = 0;
  dl_t0 = dl_print_ms = millis();
  dl_state = DL_RUN;
  return true;
}

bool push_write(const uint8_t* b, size_t n, char* err, size_t cap) {
  if (!dl_push || dl_state != DL_RUN) { snprintf(err, cap, "no upload runs"); return false; }
  if (dl_abort) { snprintf(err, cap, "stopped by ota abort"); push_fail(err); return false; }
  dl_got += n;
  if (dl_size && dl_got > dl_size) { snprintf(err, cap, "the file is longer than %lu bytes", (unsigned long)dl_size); push_fail(err); return false; }
  if (dl_kind == K_ESP && written == 0 && push_hn < sizeof(push_head)) {  // hold the head for the image check
    const size_t k = n < sizeof(push_head) - push_hn ? n : sizeof(push_head) - push_hn;
    memcpy(push_head + push_hn, b, k);
    push_hn += k;
    b += k;
    n -= k;
    if (push_hn < sizeof(push_head)) return true;
    if (!push_sink(push_head, push_hn, err, cap)) { push_fail(err); return false; }
  }
  if (n && !push_sink(b, n, err, cap)) { push_fail(err); return false; }
  return true;
}

bool push_end(char* err, size_t cap) {
  if (!dl_push || dl_state != DL_RUN) { snprintf(err, cap, "%s", dl_err[0] ? dl_err : "no upload runs"); return false; }
  bool ok = true;
  if (dl_kind == K_ESP && written == 0) {  // the whole file is in the head buffer
    if (!push_hn) { snprintf(err, cap, "empty file"); ok = false; }
    else ok = push_sink(push_head, push_hn, err, cap);
  }
  if (ok && dl_size && dl_got != dl_size) {
    snprintf(err, cap, "the file ended at %lu of %lu bytes (truncated)", (unsigned long)dl_got, (unsigned long)dl_size);
    ok = false;
  }
  if (ok && dl_kind == K_PART) {
    uint8_t got[32];
    mbedtls_sha256_finish_ret(&sha, got);
    mbedtls_sha256_free(&sha);
    if (sha_given && memcmp(got, sha_expected, 32)) { snprintf(err, cap, "sha256 mismatch"); ok = false; }
    if (ok) {
      memcpy(sha_expected, got, 32);
      dl_size = part_written;
      ok = part_finish(err, cap);  // read back from flash: equal to the bytes received
    }
    if (ok) { dl_ms = millis() - dl_t0; dl_state = DL_OK; return true; }  // fetch_poll reports it
  } else if (ok) {
    const bool known = dl_size != 0;
    if (!known) size_ = written;  // the bytes received are the image
    ok = finish_image(err, cap, !known);
    if (ok) { dl_ms = millis() - dl_t0; return true; }  // DL_RUN until push_restart
  }
  push_fail(err);
  return false;
}

void push_abort(const char* why) {
  if (dl_push && dl_state == DL_RUN) push_fail(why);
}

void push_restart() {
  if (dl_push && dl_kind == K_ESP && dl_state == DL_RUN && !active) dl_state = DL_OK;  // service() restarts
}

uint32_t push_got() { return dl_got; }

void command(const char* arg) {
  Print& o = *h.out;
  if (!strncmp(arg, "url ", 4)) { url_command(arg + 4); return; }
  if (busy() && strcmp(arg, "status") && strcmp(arg, "abort")) { o.println(DL_RUNS); return; }
  unsigned long size = 0, off = 0;
  char hex[1100];  // on the stack: static RAM is short on the rover (dram0_0_seg)
  char word[16] = {0}, word2[16] = {0};
  char err[64];
  const int nb = sscanf(arg, "begin %lu %64s %15s %15s", &size, hex, word, word2);
  if (nb >= 2) {
    if (!h.may_start()) { o.println("OTA error Stop the vehicle first"); return; }
    if (strlen(hex) != 64 || !hex_to(hex, sha_expected, 32)) { o.println("OTA error sha256 must be 64 hex"); return; }
    const char* slot = "";
    if (!start_image(size, err, sizeof(err), &slot)) { o.printf("OTA error %s\n", err); return; }
    sha_given = true;
    dl_push = false;
    want_confirm = !strcmp(word, "confirm") || !strcmp(word2, "confirm");
    need_crc = !strcmp(word, "crc") || !strcmp(word2, "crc");
    o.printf("OTA begin ok size=%lu slot=%s confirm=%d crc=%d\n", size, slot, (int)want_confirm, (int)need_crc);
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
    if (!write_chunk(buf, n, err, sizeof(err))) { o.printf("OTA error %s\n", err); return; }
    o.printf("OTA ok %lu\n", (unsigned long)written);
    return;
  }
  if (!strcmp(arg, "end")) {
    if (!finish_image(err, sizeof(err))) { o.printf("OTA error %s\n", err); return; }
    o.printf("OTA done; restarting into the new image (confirm=%d)\n", (int)want_confirm);
    h.restart();
    return;
  }
  if (!strcmp(arg, "abort")) {
    if (busy()) { dl_abort = true; o.println("OTA abort: the download stops"); return; }  // the task aborts the writer
    if (active) Update.abort();
    active = false;
    o.println("OTA aborted");
    return;
  }
  if (!strcmp(arg, "confirm")) {
    if (pending && !done) mark_valid("confirm");
    else o.printf("OTA confirm: nothing pending (pending=%d done=%d)\n", (int)pending, (int)done);
    return;
  }
  if (!strcmp(arg, "status")) {
    const esp_partition_t* run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(run, &st);
    // table: the partition table this firmware is built for (partitions.csv); table_crc: CRC-32 of the table
    // in flash (0x8000, 3072 B), equal to the CRC-32 of the release partitions.bin
    uint8_t buf[256];
    uint32_t crc = 0;
    for (uint32_t off = 0; off < 3072; off += sizeof(buf)) {
      if (esp_flash_read(NULL, buf, 0x8000 + off, sizeof(buf)) != ESP_OK) { crc = 0; break; }
      crc = esp_rom_crc32_le(crc, buf, sizeof(buf));
    }
    // state, got, size: the "ota url" download (a partition download for "stm url" is not shown here); error: its
    // reason after a failure. img_state: esp_ota_img_states_t of the running image (2 valid, 1 pending verify).
    static const char* const DL[] = {"idle", "download", "done", "failed", "failed"};
    const bool esp = dl_kind == K_ESP;
    const uint8_t ds = esp ? dl_state : (uint8_t)DL_IDLE;
    char e[112] = "";
    if (ds == DL_FAIL || ds == DL_FAILED) snprintf(e, sizeof(e), " error=\"%s\"", dl_err);
    o.printf("OTA status state=%s got=%lu size=%lu%s running=%s img_state=%d gate=%d active=%d written=%lu of %lu"
             " table=rover-4mb-1 table_crc=%08lx\n",
             DL[ds], esp ? (unsigned long)dl_got : 0UL, esp ? (unsigned long)dl_size : 0UL, e, run->label, (int)st,
             (int)(gate && !done), (int)active, (unsigned long)written, (unsigned long)size_, (unsigned long)crc);
    return;
  }
  o.println("OTA usage: ota begin <size> <sha256> [confirm] | ota w <offset> <hex> | ota end | ota abort | ota status | ota confirm"
            " | ota url <url> <size> <sha256> [confirm]");
}

void service() {
  if (dl_kind == K_ESP && dl_state != DL_IDLE && dl_state != DL_FAILED) dl_service();
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
