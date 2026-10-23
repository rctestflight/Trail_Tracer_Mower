// Monitor role: PC-tethered ESP-NOW transceiver (from Trail-Tracer-Rx-Monitor 0.4.3, a633bff).
// Entered by the USB command "monitor" in the vehicle role, for one boot: every restart (including
// "restart", the USB guard and RST!) starts the vehicle role again, and the PC sends "monitor" again.
// Monitor to PC, one line per record, first token = type:
//   R <rx_us> <src_mac> <len> <hex>   each received frame
//   Q <us> <dst_mac> <len> <err>      at the esp_now_send call (err 0 = ESP_OK)
//   X <us> <dst_mac> <ok|fail>        from the send callback
//   I <us> <text>                     banner, 1 s heartbeat, errors
//   I <us> rssi <mac> n= avg= min= max=   each second, per sender heard: signal strength in dBm
//   CFG <result> <text>               answer to get, set, save (as the vehicle role)
// PC to monitor:
//   S <dst_mac> <hex> [*<crc32>]      send one frame (ff:ff:ff:ff:ff:ff = broadcast); with the
//                                     CRC-32 of "<dst_mac> <hex>", a damaged line is refused
//   V                                 print the banner again
//   ota <args>                        firmware update over USB (lib/usb_ota; answers "OTA ...")
//   restart                           restart (USB detach first); the board starts as a vehicle
//   dld                               restart into ROM download mode (USB detach first)
//   get <key>, set <key> <value>, save   settings (as the vehicle role)
//   monitor                           no action (the role is already on)
//   time <unix ms>                    PC time; the MON broadcast (0x44, once a second) carries it
//   P                                 host ping, once a second (no answer). After the first P of a
//                                     connection, 10 s with no PC byte while DTR is on restarts
//                                     the board (USB detach first).
//   P0                                the host stops pinging on purpose
// The board sends a MON broadcast (0x44) once a second: the rovers learn that a monitor hears them.
// Update mode (as the vehicle role): from "ota begin" the board prints no R, Q or X lines (the
// frames are counted as updskip) and serves only USB and the update, until the restart into the
// new image or 120 s after the last "ota" line. A monitor restarted in the middle of updates while it
// printed about 10 KB/s of radio lines (bench test).
// Back to the vehicle role by itself: a restart when the PC goes (USB not mounted or suspended for
// 10 s, or USB never up within 10 s of the monitor start).
// LED (GPIO1, active LOW): a 20 ms flash per received frame while a PC has the board enumerated;
// with no PC, two short blinks every 1.5 s until the return to the vehicle role.
// USB output never blocks: a line that does not fit within USB_WRITE_MS is cut, the next line
// starts with a newline, and usbdrop counts the cut lines. Output is gated on DTR
// (tud_cdc_connected()) through Serial, which holds the Arduino USB lock.
// Shipped units (production V1.0, d4f8135): they send only the '-' status text, broadcast on channel 1,
// and ignore every received frame. The monitor therefore stays on channel 1, prints every frame as an
// R line with no sender filter, and expects no answer, no MON receipt and no time base from a sender.
// The PC tells senders apart by MAC: the vehicle ID is 1 on every shipped unit.
#include "cpu_prof.h"
#include <Arduino.h>
#include <memory>  // 0.6.33: report buffers from the heap (static RAM is full)
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include "esp32-hal-tinyusb.h"
#include "usb_guard.h"
#include "usb_ota.h"
#include "monitor_role.h"
#include "usb_msc_fat.h"

void restartIntoRom();                                                   // src/platform/updates.inc
void applyTxPower();                                                     // src/platform/radio.inc (setting espnow_tx_power_dbm)
uint8_t configCommandRun(const char* t, bool over_espnow, String& reply);  // src/platform/commands.inc

namespace {

constexpr uint32_t HOST_SILENT_MS = 10000;  // after a ping, no PC byte this long with DTR on: restart
constexpr int RX_LINE_MAX = 1200;           // "ota w <offset> <512 bytes as hex>" needs about 1040
constexpr int LED_PIN = 1;                  // GPIO1, active-LOW (nLED)
constexpr int EVT_QUEUE_LEN = 64;
constexpr int TEXT_LINE_MAX = 2 * ESP_NOW_MAX_DATA_LEN + 64;
constexpr uint32_t USB_WRITE_MS = 20;       // longest time one line may wait for the host
constexpr uint32_t UPDATE_IDLE_MS = 120000; // update mode ends this long after the last "ota" line

enum EvtKind : uint8_t { EVT_RX, EVT_TX };

struct Evt {
  EvtKind kind;
  bool ok;
  uint8_t mac[6];
  int64_t us;
  int len;
  uint8_t data[ESP_NOW_MAX_DATA_LEN];
};

const char* fw_version = "";
QueueHandle_t evt_queue;
volatile uint32_t rx_count = 0;
volatile uint32_t drop_count = 0;
volatile bool restart_requested = false;
uint32_t tx_count = 0;
uint32_t tx_fail_count = 0;
uint32_t usb_drop_count = 0;
uint32_t upd_skip_count = 0;  // frames not printed in update mode
bool usb_cut = false;
volatile uint32_t usb_rx_overflow = 0;
uint32_t s_crc_bad = 0;
bool host_pinged = false;
uint32_t host_rx_ms = 0;
bool update_mode = false;
uint32_t update_last_ms = 0;

// USB diagnostics that survive a software restart. In the USB output stop no USB path is left for
// them, so the next monitor start prints the values of the boot before ("I ... prev ...").
struct UsbDiag {
  uint32_t magic;
  uint32_t loop_ms;        // last loop() pass
  uint32_t out_ok_ms;      // last USB write that took bytes
  uint32_t host_rx_ms;     // last byte from the PC
  uint32_t s_ms;           // last S line run
  uint32_t evt_rx, evt_tx; // Arduino USB event task: RX and TX events handled
  uint32_t gate_skip;      // lines not written: tud_cdc_connected() false
  uint32_t usb_drop;
  uint32_t afw_max_ms;     // longest Serial.availableForWrite() call (the TX lock waits up to 250 ms)
  uint16_t fifo;           // tud_cdc_n_write_available(0) at the last pass
  uint8_t mounted, suspended, connected, line_state;
  uint8_t step;            // 0.6.6: the loop() step running now (a stop names its step at the next start)
  uint8_t in_write;        // 0.6.6: 1 while usb_write runs, 2 while it waits for FIFO space
  uint32_t step_ms;        // 0.6.6: when that step began
};
constexpr uint32_t USB_DIAG_MAGIC = 0x55534245;  // "USBE" (0.6.6: step fields)
RTC_NOINIT_ATTR UsbDiag usb_diag;
char* prev_diag = nullptr;  // text of the boot before, from the heap at monitor start

bool host_open() { return tud_cdc_connected(); }

// 0.5.113: a Linux PC opens the port with echo on and switches it off a moment later; board output in that moment came
// back as input in front of the first command ("unknown command"; bench: 3 in 900 Monitor Rx starts, 0 in 900 with the
// echo off). The Monitor Rx holds its output for OPEN_QUIET_MS after the PC opens the port: with nothing sent there is
// nothing to echo. Input is kept (a command sent at once is answered after the hold).
constexpr uint32_t OPEN_QUIET_MS = 100;
bool host_ready() {
  static bool open_prev = false;
  static uint32_t open_edge_ms = 0;
  const bool o = tud_cdc_connected();
  if (o && !open_prev) open_edge_ms = millis();
  open_prev = o;
  return o && millis() - open_edge_ms >= OPEN_QUIET_MS;
}

void usb_diag_pass(uint32_t now) {
  usb_diag.loop_ms = now;
  usb_diag.mounted = tud_mounted();
  usb_diag.suspended = tud_suspended();
  usb_diag.connected = tud_cdc_connected();
  usb_diag.line_state = tud_cdc_n_get_line_state(0);
  usb_diag.fifo = (uint16_t)tud_cdc_n_write_available(0);
}

void on_usb_rx_tx(void*, esp_event_base_t, int32_t id, void*) {
  if (id == ARDUINO_USB_CDC_RX_EVENT) usb_diag.evt_rx++;
  else usb_diag.evt_tx++;
}

void on_recv(const uint8_t* mac, const uint8_t* data, int len) {
  Evt e;
  e.us = esp_timer_get_time();
  e.kind = EVT_RX;
  memcpy(e.mac, mac, 6);
  e.len = min(len, ESP_NOW_MAX_DATA_LEN);
  memcpy(e.data, data, e.len);
  rx_count++;
  if (len == 5 && data[0] == 0x7F && memcmp(data + 1, "RST!", 4) == 0) restart_requested = true;
  if (xQueueSend(evt_queue, &e, 0) != pdTRUE) drop_count++;
}

// MON broadcast (docs/espnow_protocol.md, type 0x44), once a second: the rovers learn that a monitor
// hears them, and the PC time. The PC gives its time with the USB line "time <unix ms>".
volatile uint8_t mon_inflight = 0;  // MON frames sent and not yet reported by on_sent (no X line for them)
bool pc_time_known = false;
int64_t pc_offset_ms = 0;           // PC unix ms - esp_timer ms

void on_sent(const uint8_t* mac, esp_now_send_status_t status) {
  if (mon_inflight && mac[0] == 0xFF && mac[1] == 0xFF && mac[5] == 0xFF) { mon_inflight--; return; }
  Evt e;
  e.us = esp_timer_get_time();
  e.kind = EVT_TX;
  e.ok = (status == ESP_NOW_SEND_SUCCESS);
  memcpy(e.mac, mac, 6);
  e.len = 0;
  if (xQueueSend(evt_queue, &e, 0) != pdTRUE) drop_count++;
}

int timed_afw() {
  const uint32_t t = millis();
  const int a = Serial.availableForWrite();
  const uint32_t d = millis() - t;
  if (d > usb_diag.afw_max_ms) usb_diag.afw_max_ms = d;
  return a;
}

// Stall diagnostics (USB output stop search, 2026-10-12): per-second buckets of the last 5 s; at a guard stall the
// sums are kept and printed once USB is back: "I <us> usbstall n=<k> ..." (R lines, bytes written, write waits of
// 1 ms, cut lines, STM32 UART bytes drained, the longest loop pass).
struct DiagSec { uint32_t r_lines, out_bytes, waits, cuts, uart_bytes, loop_max_us; };
DiagSec diag_sec[5];
uint32_t diag_sec_t = 0;
int diag_i = 0;

void usb_write_inner(const char* p, int n);
void usb_write(const char* p, int n) {
  usb_diag.in_write = 1;
  usb_write_inner(p, n);
  usb_diag.in_write = 0;
}
void usb_write_inner(const char* p, int n) {
  diag_sec[diag_i].out_bytes += n > 0 ? n : 0;
  if (!host_open()) { usb_diag.gate_skip++; return; }
  while (!host_ready() && host_open()) delay(1);  // once per open, at most OPEN_QUIET_MS
  if (usb_cut) {
    if (timed_afw() < 1) { usb_drop_count++; usb_diag.usb_drop = usb_drop_count; return; }
    Serial.write('\n');
    usb_cut = false;
  }
  uint32_t t0 = millis();
  while (n > 0) {
    int space = timed_afw();
    if (space > 0) {
      int k = Serial.write((const uint8_t*)p, min(space, n));
      if (k > 0) usb_diag.out_ok_ms = millis();
      p += k;
      n -= k;
      continue;
    }
    if (millis() - t0 >= USB_WRITE_MS || !host_open()) {
      usb_drop_count++;
      usb_diag.usb_drop = usb_drop_count;
      usb_cut = true;
      diag_sec[diag_i].cuts++;
      return;
    }
    diag_sec[diag_i].waits++;
    usb_diag.in_write = 2;
    delay(1);
  }
}

void out(const char* fmt, ...) {
  char line[TEXT_LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  usb_write(line, min(n, (int)sizeof(line) - 1));
}

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(m) (m)[0], (m)[1], (m)[2], (m)[3], (m)[4], (m)[5]

// Signal strength (RSSI) per sender (design choice 2026-10-10: a monitor statistic; the link budget is
// taken as equal both ways). The ESP-NOW receive callback of ESP-IDF 4.4 gives no RSSI, so the
// Wi-Fi promiscuous receive runs for management frames: an ESP-NOW frame is an action frame (0xD0)
// with the Espressif vendor category (7F 18 FE 34). Each second, one line per sender heard:
//   I <us> rssi <mac> n=<frames> avg=<dBm> min=<dBm> max=<dBm>
struct RssiStat { uint8_t mac[6]; uint32_t n; int32_t sum; int8_t mn, mx; };
constexpr int RSSI_SLOTS = 16;  // a slot with no frame in the last second is freed
RssiStat* rssi_tab = nullptr;  // heap, at monitor start

void on_promisc(void* buf, wifi_promiscuous_pkt_type_t type) {
  RssiStat* t = rssi_tab;
  if (type != WIFI_PKT_MGMT || !t) return;
  const wifi_promiscuous_pkt_t* pk = (const wifi_promiscuous_pkt_t*)buf;
  const uint8_t* f = pk->payload;
  if (pk->rx_ctrl.sig_len < 28 || f[0] != 0xD0 || f[24] != 0x7F || f[25] != 0x18 || f[26] != 0xFE || f[27] != 0x34) return;
  const uint8_t* src = f + 10;
  const int8_t r = (int8_t)pk->rx_ctrl.rssi;
  for (int i = 0; i < RSSI_SLOTS; i++) {
    RssiStat& e = t[i];
    const bool empty = e.mac[0] == 0 && e.mac[1] == 0 && e.mac[2] == 0 && e.mac[3] == 0;
    if (!empty && memcmp(e.mac, src, 6)) continue;
    if (empty) memcpy(e.mac, src, 6);
    if (e.n == 0 || r < e.mn) e.mn = r;
    if (e.n == 0 || r > e.mx) e.mx = r;
    e.sum += r;
    e.n++;
    return;
  }
}

void print_rssi() {
  if (!rssi_tab) return;
  for (int i = 0; i < RSSI_SLOTS; i++) {
    RssiStat& e = rssi_tab[i];
    if (!e.n) { memset(e.mac, 0, 6); continue; }  // sender gone: free the slot
    const uint32_t n = e.n; const int32_t sum = e.sum; const int mn = e.mn, mx = e.mx;
    e.n = 0; e.sum = 0;
    out("I %lld rssi " MAC_FMT " n=%lu avg=%.1f min=%d max=%d\n", esp_timer_get_time(), MAC_ARG(e.mac),
        (unsigned long)n, (double)sum / n, mn, mx);
  }
}

bool quiet = false;  // bench "quiet on": R and X lines not printed

void print_event(const Evt& e) {
  static const char hex[] = "0123456789abcdef";
  if (quiet) { if (e.kind != EVT_RX && !e.ok) tx_fail_count++; return; }
  if (e.kind == EVT_RX) {
    char line[TEXT_LINE_MAX];
    int n = snprintf(line, sizeof(line), "R %lld " MAC_FMT " %d ", e.us, MAC_ARG(e.mac), e.len);
    for (int i = 0; i < e.len; i++) {
      line[n++] = hex[e.data[i] >> 4];
      line[n++] = hex[e.data[i] & 0x0f];
    }
    line[n++] = '\n';
    diag_sec[diag_i].r_lines++;
    usb_write(line, n);
  } else {
    if (!e.ok) tx_fail_count++;
    out("X %lld " MAC_FMT " %s\n", e.us, MAC_ARG(e.mac), e.ok ? "ok" : "fail");
  }
}

void print_banner() {
  uint8_t ch;
  wifi_second_chan_t ch2;
  esp_wifi_get_channel(&ch, &ch2);
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  out("I %lld Trail-Tracer-Rx-Monitor v%s MAC " MAC_FMT " channel %u reset %d role monitor (rover-firmware image)\n",
      esp_timer_get_time(), fw_version, MAC_ARG(mac), ch, (int)esp_reset_reason());
  if (prev_diag && prev_diag[0]) out("I %lld prev %s\n", esp_timer_get_time(), prev_diag);
}

bool parse_mac(const char* s, uint8_t* mac) {
  unsigned v[6];
  if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
  return true;
}

int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

int parse_hex(const char* s, uint8_t* o, int max) {
  int n = 0;
  while (s[0] && s[1]) {
    int hi = hex_nibble(s[0]), lo = hex_nibble(s[1]);
    if (hi < 0 || lo < 0 || n >= max) return -1;
    o[n++] = (uint8_t)((hi << 4) | lo);
    s += 2;
  }
  return s[0] ? -1 : n;
}

void print_error(const char* text) { out("I %lld error %s\n", esp_timer_get_time(), text); }

uint32_t crc32_zlib(const uint8_t* p, size_t n) {  // = Python zlib.crc32
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int i = 0; i < 8; i++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

void cmd_send(char* args) {
  uint8_t mac[6];
  uint8_t data[ESP_NOW_MAX_DATA_LEN];
  char* sp = strchr(args, ' ');
  if (!sp) { print_error("S: usage S <mac> <hex> [*<crc32>]"); return; }
  char* star = strchr(sp + 1, '*');
  if (star) {
    uint32_t want = strtoul(star + 1, nullptr, 16);
    if (star > sp + 1 && star[-1] == ' ') star[-1] = 0;
    *star = 0;
    *sp = ' ';
    uint32_t got = crc32_zlib((const uint8_t*)args, strlen(args));
    *sp = 0;
    if (got != want) { s_crc_bad++; print_error("S: crc mismatch, not sent"); return; }
  } else {
    *sp = 0;
  }
  if (!parse_mac(args, mac)) { print_error("S: bad mac"); return; }
  int len = parse_hex(sp + 1, data, sizeof(data));
  if (len <= 0) { print_error("S: bad hex"); return; }
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peer);
  }
  int64_t us = esp_timer_get_time();
  usb_diag.s_ms = millis();
  esp_err_t err = esp_now_send(mac, data, len);
  tx_count++;
  out("Q %lld " MAC_FMT " %d %d\n", us, MAC_ARG(mac), len, (int)err);
}

// Print for usb_ota's answers: collects one line, then sends it with usb_write (never blocks).
// Its buffer and the input line come from the heap at monitor start: static RAM is full with the
// vehicle role's buffers (1440 B over when static).
constexpr size_t OTA_LINE_MAX = 256;
class LineOut : public Print {
 public:
  size_t write(uint8_t c) override {
    if (!buf) return 1;
    if (n < OTA_LINE_MAX) buf[n++] = (char)c;
    if (c == '\n') { usb_write(buf, (int)n); n = 0; }
    return 1;
  }
  char* buf = nullptr;
 private:
  size_t n = 0;
};
LineOut ota_out;

void restart_with_detach() {
  msc_fat_eject();  // the PC stops its drive reads first (a PC that read the drive at the detach lost USB ports)
  delay(500);  // also: the last answer ("OTA done") reaches the host before the detach
  usb_guard::restart();
}

bool ota_may_start() {  // a monitor has no motion to protect; enter update mode
  if (!update_mode) {
    out("I %lld UPDATE mode on: USB and the update only; loop watchdog paused\n", esp_timer_get_time());
    disableLoopWDT();  // as the vehicle role: the image check at "ota end" ran past 5 s once
  }
  update_mode = true;
  update_last_ms = millis();
  return true;
}

void handle_line(char* line) {
  if (line[0] == 'S' && line[1] == ' ') cmd_send(line + 2);
  else if (line[0] == 'V' && line[1] == 0) print_banner();
  else if (line[0] == 'P' && line[1] == 0) {
    host_pinged = true;
    usb_guard::set_reconnect(true);  // the PC program connected: an undrained FIFO now gets the USB reconnect
  }
  else if (!strcmp(line, "P0")) host_pinged = false;
  else if (!strncmp(line, "ota ", 4)) {
    if (update_mode) update_last_ms = millis();
    usb_ota::command(line + 4);
  } else if (!strncmp(line, "time ", 5)) {
    const long long u = atoll(line + 5);
    if (u > 1000000000000LL) {
      pc_offset_ms = (int64_t)u - esp_timer_get_time() / 1000;
      pc_time_known = true;
      out("I %lld time set: unix_ms=%lld\n", esp_timer_get_time(), u);
    } else {
      print_error("time: usage time <unix ms>");
    }
  } else if (!strcmp(line, "cpu")) {  // CPU share per FreeRTOS task since the last "cpu" (lib/cpu_prof)
    std::unique_ptr<char[]> b(new (std::nothrow) char[700]);
    if (b) {
      cpuprof::report(b.get(), 700, nullptr, 0);
      out("I %lld %s\n", esp_timer_get_time(), b.get());
    } else print_error("cpu: no heap");
  } else if (!strcmp(line, "monitor")) {
    usb_guard::set_reconnect(true);  // the PC program connected
    out("I %lld monitor role already on\n", esp_timer_get_time());
  } else if (!strcmp(line, "promisc off") || !strcmp(line, "promisc on")) {
    // bench (USB output stop search, 2026-10-12): the Wi-Fi promiscuous receive for the RSSI lines off or on
    const bool on = line[9] == 'n';
    esp_wifi_set_promiscuous(on);
    out("I %lld promisc %s\n", esp_timer_get_time(), on ? "on" : "off");
  } else if (!strcmp(line, "quiet on") || !strcmp(line, "quiet off")) {
    // bench (same search): R and X lines not printed (counted only); beats and rssi lines stay
    quiet = line[7] == 'n';
    out("I %lld quiet %s\n", esp_timer_get_time(), quiet ? "on" : "off");
  } else if (!strcmp(line, "restart")) {
    out("I %lld restart (USB detach first)\n", esp_timer_get_time());
    delay(50);
    restart_with_detach();
  } else if (!strcmp(line, "dld")) {
    out("I %lld restart into ROM download mode\n", esp_timer_get_time());
    delay(50);
    restartIntoRom();
  } else if (!strncmp(line, "get ", 4) || !strncmp(line, "set ", 4) || !strcmp(line, "save")) {
    String reply;
    const uint8_t r = configCommandRun(line, false, reply);
    out("CFG %u %s\n", (unsigned)r, reply.c_str());
  } else if (line[0]) print_error("unknown command");
}

void on_usb_event(void*, esp_event_base_t, int32_t id, void* data) {
  if (id == ARDUINO_USB_CDC_RX_OVERFLOW_EVENT)
    usb_rx_overflow += ((arduino_usb_cdc_event_data_t*)data)->rx_overflow.dropped_bytes;
}

char* rx_line = nullptr;  // RX_LINE_MAX bytes from the heap at monitor start

void poll_serial() {
  char* line = rx_line;
  static int n = 0;
  static bool overflow = false;
  static uint32_t seen_rx_overflow = 0;
  while (Serial.available()) {
    char c = Serial.read();
    host_rx_ms = millis();
    usb_diag.host_rx_ms = host_rx_ms;
    if (c == '\r') continue;
    if (usb_rx_overflow != seen_rx_overflow) {
      seen_rx_overflow = usb_rx_overflow;
      overflow = true;
    }
    if (c == '\n') {
      line[n] = 0;
      if (overflow) print_error("line damaged (USB input overflow or too long), not run");
      else handle_line(line);
      n = 0;
      overflow = false;
    } else if (n < RX_LINE_MAX - 1) {
      line[n++] = c;
    } else {
      overflow = true;
    }
  }
}

// LED: with no PC (not enumerated), two 80 ms blinks every 1.5 s; else a flash per frame.
void led_service(uint32_t now, uint32_t frame_flash_until) {
  if (!tud_mounted()) {
    const uint32_t t = now % 1500;
    digitalWrite(LED_PIN, (t < 80 || (t >= 240 && t < 320)) ? LOW : HIGH);
    return;
  }
  digitalWrite(LED_PIN, (int32_t)(now - frame_flash_until) < 0 ? LOW : HIGH);
}

}  // namespace

namespace monitor_role {

// 0.5.109: the radio starts from loop(), once the PC has configured the USB device (plus 300 ms), or 3 s after the
// start with no PC. The Monitor Rx boot started Wi-Fi within about 100 ms of the USB start (the core starts USB before
// setup()), while the PC enumerated it; about 1 in 100 of these boots left USB dead with protocol errors (-71) until
// a power cycle (bench 2026-10-15: 3 in about 290). The vehicle role starts its radio after setup(), 2.8 s in, and
// showed 0 in about 190 boots. (0.5.108 waited for this in setup(); every boot then ended in a task watchdog reset.)
bool radio_on = false;
uint32_t uart_clears = 0;  // 0.6.8: Rx STM32 input cleared (a read returned nothing while bytes waited)
void radio_start() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // shipped units send on channel 1 only
  if (esp_now_init() != ESP_OK) {
    print_error("ESP-NOW init failed");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(on_recv);
  esp_now_register_send_cb(on_sent);
  applyTxPower();  // espnow_tx_power_dbm, as the vehicle role
  {
    esp_now_peer_info_t peer = {};
    memset(peer.peer_addr, 0xFF, 6);
    peer.ifidx = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(peer.peer_addr)) esp_now_add_peer(&peer);  // MON broadcast
  }
  rssi_tab = (RssiStat*)calloc(RSSI_SLOTS, sizeof(RssiStat));
  if (rssi_tab) {
    wifi_promiscuous_filter_t flt = {};
    flt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&flt);
    esp_wifi_set_promiscuous_rx_cb(on_promisc);
    esp_wifi_set_promiscuous(true);
  }
  radio_on = true;
}

void setup(const char* version) {
  fw_version = version;
  // The STM32 UART as the vehicle role starts it, so the ESP32 TX line idles high. Left floating,
  // the STM32 read noise for the whole monitor run, and after the return to the vehicle role it
  // sent no packets until a power cycle (bench test 2026-10-10). Its packets are discarded.
  Serial1.setRxBufferSize(2048);
  Serial1.begin(2000000);
  pinMode(GPIO_NUM_8, INPUT_PULLUP);  // STM_nDATA_READY
  rx_line = (char*)malloc(RX_LINE_MAX);
  ota_out.buf = (char*)malloc(OTA_LINE_MAX);
  prev_diag = (char*)malloc(300);
  if (!rx_line || !ota_out.buf || !prev_diag) ESP.restart();
  prev_diag[0] = 0;
  if (usb_diag.magic == USB_DIAG_MAGIC) {  // values of the last monitor boot, until its restart
    const UsbDiag& d = usb_diag;
    snprintf(prev_diag, 300,
             "loop_ms=%lu out_ok_ms=%lu host_rx_ms=%lu s_ms=%lu evrx=%lu evtx=%lu gskip=%lu usbdrop=%lu "
             "afwmax=%lu fifo=%u mnt=%u susp=%u conn=%u ls=%u step=%u step_ms=%lu in_write=%u",
             (unsigned long)d.loop_ms, (unsigned long)d.out_ok_ms, (unsigned long)d.host_rx_ms, (unsigned long)d.s_ms,
             (unsigned long)d.evt_rx, (unsigned long)d.evt_tx, (unsigned long)d.gate_skip, (unsigned long)d.usb_drop,
             (unsigned long)d.afw_max_ms, (unsigned)d.fifo, (unsigned)d.mounted, (unsigned)d.suspended,
             (unsigned)d.connected, (unsigned)d.line_state, (unsigned)d.step, (unsigned long)d.step_ms, (unsigned)d.in_write);
  }
  memset(&usb_diag, 0, sizeof(usb_diag));
  usb_diag.magic = USB_DIAG_MAGIC;
  Serial.setRxBufferSize(16384);
  usb_guard::begin();
  usb_guard::set_before_detach(msc_fat_eject);  // a guard reconnect ejects the drive first
  Serial.onEvent(ARDUINO_USB_CDC_RX_OVERFLOW_EVENT, on_usb_event);
  Serial.onEvent(ARDUINO_USB_CDC_RX_EVENT, on_usb_rx_tx);
  Serial.onEvent(ARDUINO_USB_CDC_TX_EVENT, on_usb_rx_tx);
  Serial.begin(115200);
  usb_ota::begin(usb_ota::Hooks{&ota_out, ota_may_start, restart_with_detach});
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  evt_queue = xQueueCreate(EVT_QUEUE_LEN, sizeof(Evt));
  enableLoopWDT();
  print_banner();
}

#define STEP(n) do { usb_diag.step = (n); usb_diag.step_ms = millis(); } while (0)  // 0.6.6
void loop() {
  static uint32_t last_beat = 0;
  static uint32_t flash_until = 0;
  static bool host_was_open = false;
  STEP(1);
  if (!radio_on) {
    static uint32_t mounted_ms = 0;
    if (tud_mounted() && !mounted_ms) mounted_ms = millis();
    if ((mounted_ms && millis() - mounted_ms >= 300) || millis() >= 3000) radio_start();
  }
  STEP(2);
  const bool open_now = host_open();
  if (open_now && !host_was_open) {
    print_banner();
    std::unique_ptr<char[]> saved(new (std::nothrow) char[900]);
    if (saved && usb_guard::take_saved(saved.get(), 900))
      out("I %lld %s (input stop, before the last restart)\n", esp_timer_get_time(), saved.get());
  }
  STEP(3);
  if (!open_now) host_pinged = false;
  if (open_now && host_pinged && millis() - host_rx_ms >= HOST_SILENT_MS) {
    out("I %lld restart: no PC input for %lu s after pings (USB input stopped?)\n", esp_timer_get_time(),
        (unsigned long)(HOST_SILENT_MS / 1000));
    {
      std::unique_ptr<char[]> regs(new (std::nothrow) char[1200]);
      usb_guard::snapshot();
      if (regs) {
        usb_guard::format_snapshot(regs.get(), 1200);
        usb_guard::save_for_next_boot(regs.get());  // the line below can be lost in the restart
        out("I %lld %s (input stop)\n", esp_timer_get_time(), regs.get());
      }
      delay(200);
    }
    delay(100);
    restart_with_detach();
  }
  STEP(4);
  host_was_open = open_now;
  // Back to the vehicle role when the PC goes: USB not mounted or suspended for 10 s (also 10 s if
  // it never came up after the monitor start). The restart clears the role. 10 s, not 2 s: a
  // supply-fed monitor rides through a host USB power cycle (Pi: 3 s off, about 5 s with enumeration).
  {
    static bool usb_seen = false;
    static uint32_t usb_lost_since = 0;
    const uint32_t now = millis();
    if ((tud_mounted() && !tud_suspended()) || usb_guard::reconnecting(now)) {  // a guard reconnect is not a PC loss
      usb_seen = true;
      usb_lost_since = 0;
    } else if (!usb_lost_since) {
      usb_lost_since = now ? now : 1;
    } else if (now - usb_lost_since >= 10000u) {
      ESP.restart();  // no USB host to detach from
    }
  }
  {
    static uint32_t loop_t = 0;
    const uint32_t us = micros();
    if (loop_t && us - loop_t > diag_sec[diag_i].loop_max_us) diag_sec[diag_i].loop_max_us = us - loop_t;
    loop_t = us;
    const uint32_t ms = millis();
    if (ms - diag_sec_t >= 1000) { diag_sec_t = ms; diag_i = (diag_i + 1) % 5; diag_sec[diag_i] = DiagSec{}; }
  }
  STEP(5);
  {  // Rx STM32 frames: not used in the monitor role, read and dropped
    // 0.6.8: in blocks, at most 4 KB a pass; a read that returns nothing while bytes wait clears the input. The byte loop
    // of 0.5.60-0.6.7 never ended on the first Monitor Rx start after a USB load (5 of 5 such starts on one board): loop() stopped
    // in it with USB alive, and the PC saw no line after the banner.
    uint8_t tmp[256];
    size_t got = 0;
    int n = Serial1.available();
    while (n > 0 && got < 4096) {
      const size_t k = Serial1.read(tmp, n < (int)sizeof(tmp) ? (size_t)n : sizeof(tmp));
      if (k == 0) { Serial1.flush(false); uart_clears++; break; }  // false: the receive side too
      got += k;
      n = Serial1.available();
    }
    diag_sec[diag_i].uart_bytes += got;
  }
  STEP(6);
  usb_guard::service(millis(), true);
  {
    static uint32_t seen_stalls = 0;
    static bool pending = false;
    static DiagSec snap;
    if (usb_guard::stalls() != seen_stalls) {
      seen_stalls = usb_guard::stalls();
      snap = DiagSec{};
      for (const DiagSec& d : diag_sec) {
        snap.r_lines += d.r_lines; snap.out_bytes += d.out_bytes; snap.waits += d.waits; snap.cuts += d.cuts;
        snap.uart_bytes += d.uart_bytes; if (d.loop_max_us > snap.loop_max_us) snap.loop_max_us = d.loop_max_us;
      }
      pending = true;
    }
    static uint32_t diag_line_ms = 0;
    if (millis() - diag_line_ms >= 60000) {  // once a minute: the guard counters (bench: USB output stop search)
      diag_line_ms = millis();
      out("I %lld usbdiag kicks=%lu stalls=%lu reconnects=%lu\n", esp_timer_get_time(), (unsigned long)usb_guard::kicks(),
          (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts());
    }
    static uint8_t repeats = 0;
    static uint32_t repeat_ms = 0;
    if (pending) { repeats = 3; repeat_ms = millis() - 5000; pending = false; }
    if (repeats && host_open() && !usb_guard::reconnecting(millis()) && millis() - repeat_ms >= 5000) {
      repeats--;  // bench (0.5.67): three copies 5 s apart; one copy was lost in the output after a reconnect
      repeat_ms = millis();
      out("I %lld usbstall n=%lu last5s r_lines=%lu out_bytes=%lu waits=%lu cuts=%lu uart_bytes=%lu loop_max_us=%lu heap=%lu kicks=%lu\n",
          esp_timer_get_time(), (unsigned long)seen_stalls, (unsigned long)snap.r_lines, (unsigned long)snap.out_bytes,
          (unsigned long)snap.waits, (unsigned long)snap.cuts, (unsigned long)snap.uart_bytes, (unsigned long)snap.loop_max_us,
          (unsigned long)ESP.getFreeHeap(), (unsigned long)usb_guard::kicks());
      std::unique_ptr<char[]> regs(new (std::nothrow) char[1200]);
      if (regs) {
        usb_guard::format_snapshot(regs.get(), 1200);
        out("I %lld %s (output stop)\n", esp_timer_get_time(), regs.get());
      }
    }
  }
  STEP(7);
  if (update_mode && millis() - update_last_ms >= UPDATE_IDLE_MS) {
    update_mode = false;
    enableLoopWDT();
    out("I %lld UPDATE mode off: 120 s without an ota line; loop watchdog on\n", esp_timer_get_time());
  }
  STEP(8);
  Evt e;
  while (xQueueReceive(evt_queue, &e, 0) == pdTRUE) {
    if (update_mode) upd_skip_count++;
    else print_event(e);
    poll_serial();
    if (e.kind == EVT_RX) flash_until = millis() + 20;
  }
  STEP(9);
  led_service(millis(), flash_until);
  STEP(10);
  if (restart_requested) {
    out("I %lld restart requested\n", esp_timer_get_time());
    delay(100);
    restart_with_detach();
  }
  STEP(11);
  poll_serial();
  STEP(12);
  usb_ota::service();
  STEP(13);
  usb_diag_pass(millis());
  STEP(14);
  msc_fat_media_service();
  STEP(15);
  if (millis() - last_beat >= 1000) {
    last_beat = millis();
    out("I %lld beat rx=%lu drop=%lu tx=%lu txfail=%lu usbdrop=%lu usbrxdrop=%lu scrcbad=%lu "
        "grdrst=%lu grdfull=%lu grdstall=%lu update=%d updskip=%lu evrx=%lu evtx=%lu gskip=%lu afwmax=%lu fifo=%u ls=%u uclr=%lu\n",
        esp_timer_get_time(), (unsigned long)rx_count, (unsigned long)drop_count, (unsigned long)tx_count,
        (unsigned long)tx_fail_count, (unsigned long)usb_drop_count, (unsigned long)usb_rx_overflow,
        (unsigned long)s_crc_bad, (unsigned long)usb_guard::restarts(), (unsigned long)usb_guard::full_resets(),
        (unsigned long)usb_guard::stalls(), (int)update_mode, (unsigned long)upd_skip_count,
        (unsigned long)usb_diag.evt_rx, (unsigned long)usb_diag.evt_tx, (unsigned long)usb_diag.gate_skip,
        (unsigned long)usb_diag.afw_max_ms, (unsigned)usb_diag.fifo, (unsigned)usb_diag.line_state, (unsigned long)uart_clears);
    if (!update_mode) {
      print_rssi();
      uint8_t f[15];
      const int64_t now_ms = esp_timer_get_time() / 1000;
      const uint32_t up = (uint32_t)now_ms;
      const uint64_t unix_ms = pc_time_known ? (uint64_t)(now_ms + pc_offset_ms) : 0;
      f[0] = 0x44; f[1] = 1;
      memcpy(f + 2, &up, 4);
      memcpy(f + 6, &unix_ms, 8);
      f[14] = 0;  // n: no per-rover receipts yet
      static const uint8_t bc[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (radio_on) {
        mon_inflight++;
        if (esp_now_send(bc, f, sizeof(f)) != ESP_OK) mon_inflight--;
      }
    }
  }
  delay(1);
}

}  // namespace monitor_role
