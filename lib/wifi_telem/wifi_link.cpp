// wifi_link: see wifi_link.h.
#include "wifi_link.h"

#include <WiFi.h>
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "wifi_telem_proto.h"

namespace wifilink {

static Hooks s_h = {nullptr, nullptr, nullptr};
static bool s_started = false;
static char s_ssid[WIFI_SSID_MAX + 1] = "";
static char s_pass[WIFI_PASS_MAX + 1] = "";  // RAM only; never printed
static uint8_t s_mac[6] = {0};

// Association
#define RETRY_MS 8000u  // not associated for this long after a start or a loss: start a new attempt
static bool s_conn = false;
static uint32_t s_try_ms = 0;
static uint32_t s_assoc_ms = 0;
static bool s_heard = false;           // a beacon or an mDNS answer since the association
static bool s_no_server_said = false;  // "no telemetry server" said for this association
static uint32_t s_reconnects = 0;

// Server
static bool s_server_known = false;
static uint8_t s_server_ip[4];
static uint16_t s_server_port = 0;
static const char* s_server_via = "none";
static uint32_t s_beacon_ms = 0;       // millis() of the last good beacon; 0 = none
static struct sockaddr_in s_server_addr;

// Sockets: beacons in on TELEM_BEACON_PORT; telemetry out (and REQ in) on TELEM_UDP_PORT
static int s_rx = -1, s_tx = -1;

// mDNS
#define MDNS_PERIOD_MS 15000u
#define MDNS_TIMEOUT_MS 2000u
static bool s_mdns_ok = false;
static mdns_search_once_t* s_search = nullptr;
static uint32_t s_mdns_ms = 0;

// Counters
static uint32_t s_sent = 0, s_failed = 0, s_unsent = 0, s_beacons = 0, s_bad_beacons = 0, s_mdns_answers = 0, s_reqs = 0;

// Scan
static bool s_scan_on = false;
static int s_scan_i = -1;  // next network to print; -1 = not done yet

static void line(const char* fmt, ...) {
  if (!s_h.out) return;
  char b[420];  // the "wifi" status line is about 260 characters; 200 cut off mdns, reqs, reconnects and the line end
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  s_h.out->print(b);
}

static void event(int32_t a, int32_t b, const char* text) {
  if (s_h.event) s_h.event(a, b, text);
}

static int openUdp(uint16_t port) {
  const int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return -1;
  int one = 1;
  lwip_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  lwip_setsockopt(s, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  struct sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (lwip_bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) { lwip_close(s); return -1; }
  lwip_fcntl(s, F_SETFL, O_NONBLOCK);  // never wait on a receive
  return s;
}

// A server address from a beacon or an mDNS answer. The last one learned wins.
static void learn(const uint8_t ip[4], uint16_t port, const char* via) {
  s_heard = true;
  const bool same = s_server_known && !memcmp(ip, s_server_ip, 4) && port == s_server_port;
  s_server_via = via;
  if (same) return;
  memcpy(s_server_ip, ip, 4);
  s_server_port = port;
  s_server_known = true;
  memset(&s_server_addr, 0, sizeof(s_server_addr));
  s_server_addr.sin_family = AF_INET;
  s_server_addr.sin_port = htons(port);
  memcpy(&s_server_addr.sin_addr.s_addr, ip, 4);  // network order: first byte first
  char t[48];
  snprintf(t, sizeof(t), "server %u.%u.%u.%u:%u via %s", ip[0], ip[1], ip[2], ip[3], (unsigned)port, via);
  line("WIFI %s\n", t);
  event(EV_SERVER, port, t);
}

static void startAttempt(uint32_t now_ms) {
  s_try_ms = now_ms;
  if (!s_ssid[0]) return;
  WiFi.begin(s_ssid, s_pass[0] ? s_pass : nullptr);  // returns at once; the Wi-Fi task associates
}

void attach(const Hooks& hooks) { s_h = hooks; }

void begin(const char* ssid, const char* pass) {
  if (s_started) return;
  snprintf(s_ssid, sizeof(s_ssid), "%s", ssid ? ssid : "");
  snprintf(s_pass, sizeof(s_pass), "%s", pass ? pass : "");
  WiFi.persistent(false);  // the Wi-Fi driver keeps the password in RAM only, not in its own NVS area
  WiFi.mode(WIFI_STA);     // radio start, about 220 ms
  WiFi.setSleep(false);    // modem sleep off: no beacon-interval wake-up delay on the sends
  WiFi.setAutoReconnect(true);
  esp_wifi_get_mac(WIFI_IF_STA, s_mac);  // the same MAC that ESP-NOW frames carry
  s_mdns_ok = mdns_init() == ESP_OK;
  s_rx = openUdp(TELEM_BEACON_PORT);
  s_tx = openUdp(TELEM_UDP_PORT);
  s_started = true;
  line("WIFI start mac=%02x:%02x:%02x:%02x:%02x:%02x ssid=\"%s\" password=%s mdns=%d sockets=%d,%d\n", s_mac[0], s_mac[1],
       s_mac[2], s_mac[3], s_mac[4], s_mac[5], s_ssid, wifitelem::passwordState(s_pass[0] != 0), (int)s_mdns_ok, s_rx, s_tx);
  startAttempt(millis());
}

bool started() { return s_started; }
bool connected() { return s_conn; }
bool ready() { return s_conn && s_server_known && s_tx >= 0; }

void setCredentials(const char* ssid, const char* pass) {
  snprintf(s_ssid, sizeof(s_ssid), "%s", ssid ? ssid : "");
  snprintf(s_pass, sizeof(s_pass), "%s", pass ? pass : "");
  if (!s_started) return;
  WiFi.disconnect();  // returns at once
  s_conn = false;
  startAttempt(millis());
}

static void receive(uint32_t now_ms) {
  uint8_t b[300];
  struct sockaddr_in from;
  socklen_t fl;
  for (int i = 0; s_rx >= 0 && i < 4; i++) {  // beacons
    fl = sizeof(from);
    const int n = lwip_recvfrom(s_rx, b, sizeof(b), MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
    if (n <= 0) break;
    uint8_t ip[4];
    uint16_t port;
    if (!wifitelem::parseBeacon(b, (size_t)n, ip, &port)) { s_bad_beacons++; continue; }
    s_beacons++;
    s_beacon_ms = now_ms ? now_ms : 1;
    learn(ip, port, "beacon");
  }
  for (int i = 0; s_tx >= 0 && i < 4; i++) {  // from the server: REQ (backfill request) only is used
    fl = sizeof(from);
    const int n = lwip_recvfrom(s_tx, b, sizeof(b), MSG_DONTWAIT, (struct sockaddr*)&from, &fl);
    if (n <= 0) break;
    if (!s_server_known || memcmp(&from.sin_addr.s_addr, s_server_ip, 4)) continue;  // not the server
    const uint8_t* p;
    size_t m;
    if (!wifitelem::openDatagram(b, (size_t)n, &p, &m) || m == 0) continue;
    s_reqs++;
    if (s_h.frame) s_h.frame(p, (int)m);
  }
}

static void mdnsService(uint32_t now_ms) {
  if (!s_mdns_ok) return;
  if (!s_search) {
    if (!s_conn || (s_mdns_ms && now_ms - s_mdns_ms < MDNS_PERIOD_MS)) return;
    s_mdns_ms = now_ms ? now_ms : 1;
    s_search = mdns_query_async_new(nullptr, TELEM_MDNS_SERVICE, TELEM_MDNS_PROTO, MDNS_TYPE_PTR, MDNS_TIMEOUT_MS, 4, nullptr);
    return;
  }
  mdns_result_t* r = nullptr;
  if (!mdns_query_async_get_results(s_search, 0, &r)) return;  // timeout 0: no wait
  for (mdns_result_t* x = r; x; x = x->next) {
    bool v_ok = true;  // TXT "v=1"; a record with no "v" is taken
    for (size_t k = 0; k < x->txt_count; k++)
      if (x->txt[k].key && !strcmp(x->txt[k].key, TELEM_MDNS_TXT_KEY))
        v_ok = x->txt[k].value && !strcmp(x->txt[k].value, TELEM_MDNS_TXT_VALUE);
    if (!v_ok || !x->port) continue;
    for (mdns_ip_addr_t* a = x->addr; a; a = a->next) {
      if (a->addr.type != ESP_IPADDR_TYPE_V4) continue;
      uint8_t ip[4];
      memcpy(ip, &a->addr.u_addr.ip4.addr, 4);
      s_mdns_answers++;
      learn(ip, x->port, "mdns");
      break;
    }
  }
  if (r) mdns_query_results_free(r);
  mdns_query_async_delete(s_search);
  s_search = nullptr;
}

void service(uint32_t now_ms) {
  if (!s_started) return;
  const bool conn = WiFi.status() == WL_CONNECTED;  // associated, with an IP address
  if (conn && !s_conn) {
    s_conn = true;
    s_assoc_ms = now_ms;
    s_heard = false;
    s_no_server_said = false;
    s_mdns_ms = 0;  // ask mDNS at once
    const IPAddress ip = WiFi.localIP();
    char t[80];
    snprintf(t, sizeof(t), "ssid=%s ip=%u.%u.%u.%u ch=%d", s_ssid, ip[0], ip[1], ip[2], ip[3], (int)WiFi.channel());
    line("WIFI associated %s rssi=%d\n", t, (int)WiFi.RSSI());
    event(EV_ASSOCIATED, WiFi.RSSI(), t);
  } else if (!conn && s_conn) {
    s_conn = false;
    s_try_ms = now_ms;  // the driver tries first (auto reconnect); a new attempt after RETRY_MS
    char t[48];
    snprintf(t, sizeof(t), "ssid=%s", s_ssid);
    line("WIFI association lost %s\n", t);
    event(EV_LOST, 0, t);
  }
  if (!s_conn && s_ssid[0] && !s_scan_on && now_ms - s_try_ms >= RETRY_MS) {
    WiFi.disconnect();
    startAttempt(now_ms);
    s_reconnects++;
  }
  receive(now_ms);
  mdnsService(now_ms);
  if (s_conn && !s_heard && !s_no_server_said && now_ms - s_assoc_ms >= TELEM_NO_SERVER_MS) {
    s_no_server_said = true;
    line("WIFI no telemetry server on this network (no beacon and no mDNS answer in 30 s; a guest network can isolate clients)\n");
    event(EV_NO_SERVER, TELEM_NO_SERVER_MS / 1000, "no telemetry server on this network");
  }
  scanService();
}

bool send(const uint8_t* payload, size_t len) {
  if (!s_conn || !s_server_known || s_tx < 0) { s_unsent++; return false; }
  uint8_t d[TELEM_DGRAM_HDR + 256];
  const size_t n = wifitelem::buildDatagram(d, sizeof(d), s_mac, payload, len);
  if (!n) { s_failed++; return false; }
  // Non-blocking socket: lwIP queues the datagram for the Wi-Fi task, or refuses it (no buffer).
  const int r = lwip_sendto(s_tx, d, n, MSG_DONTWAIT, (struct sockaddr*)&s_server_addr, sizeof(s_server_addr));
  if (r == (int)n) { s_sent++; return true; }
  s_failed++;
  return false;
}

static const char* authName(wifi_auth_mode_t a) {
  switch (a) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-enterprise";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default: return "other";
  }
}

bool scanStart() {
  if (s_scan_on) { line("WIFI scan already runs\n"); return false; }
  // Async: returns at once. About 120 ms per channel, 13 channels.
  const int16_t r = WiFi.scanNetworks(true, false, false, 120);
  if (r == WIFI_SCAN_FAILED) {
    line("WIFI scan failed (the radio may be in an association attempt; try again in a few seconds)\n");
    return false;
  }
  s_scan_on = true;
  s_scan_i = -1;
  line("WIFI scan started\n");
  return true;
}

void scanService() {
  if (!s_scan_on) return;
  if (s_scan_i < 0) {
    const int16_t c = WiFi.scanComplete();
    if (c == WIFI_SCAN_RUNNING) return;
    if (c < 0) { s_scan_on = false; line("WIFI scan failed\n"); return; }
    line("WIFI scan n=%d\n", (int)c);
    s_scan_i = 0;
  }
  // A few lines per pass, so the USB text queue does not overrun.
  const int n = WiFi.scanComplete();
  for (int k = 0; k < 4 && s_scan_i < n; k++, s_scan_i++)
    line("WIFI net \"%s\" rssi=%d ch=%d sec=%s\n", WiFi.SSID(s_scan_i).c_str(), (int)WiFi.RSSI(s_scan_i),
         (int)WiFi.channel(s_scan_i), authName(WiFi.encryptionType(s_scan_i)));
  if (s_scan_i >= n) {
    WiFi.scanDelete();
    s_scan_on = false;
    line("WIFI scan end\n");
  }
}

void printStatus(const char* mode, const char* setting, const char* stored_ssid) {
  wifitelem::Status s = {};
  s.mode = mode;
  s.setting = setting;
  s.ssid = s_started ? s_ssid : stored_ssid;
  s.state = !s_started ? "off" : s_conn ? "connected" : !s_ssid[0] ? "no network" : "connecting";
  if (s_conn) {
    s.rssi_dbm = WiFi.RSSI();
    s.channel = WiFi.channel();
    const IPAddress ip = WiFi.localIP();
    for (int i = 0; i < 4; i++) s.ip[i] = ip[i];
  }
  s.server_known = s_server_known;
  memcpy(s.server_ip, s_server_ip, 4);
  s.server_port = s_server_port;
  s.server_via = s_server_via;
  s.beacon_age_s = s_beacon_ms ? (int32_t)((millis() - s_beacon_ms) / 1000) : -1;
  s.sent = s_sent; s.failed = s_failed; s.unsent = s_unsent; s.beacons = s_beacons; s.bad_beacons = s_bad_beacons;
  s.mdns_answers = s_mdns_answers; s.reqs = s_reqs; s.reconnects = s_reconnects;
  char b[400];
  wifitelem::formatStatus(b, sizeof(b), s);
  line("%s\n", b);
}

}  // namespace wifilink
