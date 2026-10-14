// wifi_telem_proto: see wifi_telem_proto.h.
#include "wifi_telem_proto.h"

#include <stdio.h>
#include <string.h>

namespace wifitelem {

// A decimal number of 1 to max_digits digits at *p, at most max; advances *p.
static bool readNumber(const char** p, const char* end, int max_digits, uint32_t max, uint32_t* out) {
  uint32_t v = 0;
  int n = 0;
  while (*p < end && **p >= '0' && **p <= '9') {
    if (++n > max_digits) return false;
    v = v * 10 + (uint32_t)(**p - '0');
    (*p)++;
  }
  if (n == 0 || v > max) return false;
  *out = v;
  return true;
}

bool parseBeacon(const uint8_t* data, size_t len, uint8_t ip[4], uint16_t* port) {
  const char* p = (const char*)data;
  const char* end = p + len;
  // Allowed at the end: CR, LF, NUL.
  while (end > p && (end[-1] == '\r' || end[-1] == '\n' || end[-1] == 0)) end--;
  const size_t pre = strlen(TELEM_BEACON_PREFIX);
  if ((size_t)(end - p) < pre + 1 || memcmp(p, TELEM_BEACON_PREFIX, pre) || p[pre] != ' ') return false;
  p += pre + 1;
  uint8_t a[4];
  for (int i = 0; i < 4; i++) {
    uint32_t v;
    if (!readNumber(&p, end, 3, 255, &v)) return false;
    a[i] = (uint8_t)v;
    if (i < 3) {
      if (p >= end || *p != '.') return false;
      p++;
    }
  }
  if (p >= end || *p != ' ') return false;
  p++;
  uint32_t pt;
  if (!readNumber(&p, end, 5, 65535, &pt) || pt == 0) return false;
  if (p != end) return false;  // nothing after the port
  const bool zero = !a[0] && !a[1] && !a[2] && !a[3];
  const bool bcast = a[0] == 255 && a[1] == 255 && a[2] == 255 && a[3] == 255;
  if (zero || bcast) return false;
  memcpy(ip, a, 4);
  *port = (uint16_t)pt;
  return true;
}

static bool allHex(const char* s) {
  for (; *s; s++)
    if (!((*s >= '0' && *s <= '9') || (*s >= 'a' && *s <= 'f') || (*s >= 'A' && *s <= 'F'))) return false;
  return true;
}

SetResult parseWifiSet(const char* args, char ssid[WIFI_SSID_MAX + 1], char pass[WIFI_PASS_MAX + 1]) {
  ssid[0] = 0;
  pass[0] = 0;
  if (!args || !*args) return SET_USAGE;
  const char* s;
  const char* e;
  const char* rest;
  if (args[0] == '"') {  // quoted SSID: up to the next double quote
    s = args + 1;
    e = strchr(s, '"');
    if (!e) return SET_USAGE;
    rest = e + 1;
    if (*rest != 0 && *rest != ' ') return SET_USAGE;
  } else {
    s = args;
    e = strchr(s, ' ');
    if (!e) e = s + strlen(s);
    rest = e;
  }
  const size_t n = (size_t)(e - s);
  if (n == 0 || n > WIFI_SSID_MAX) return SET_SSID_LENGTH;
  if (*rest == ' ') rest++;  // one space; the password is the rest of the line
  const size_t m = strlen(rest);
  // WPA2: 8..63 characters, or 64 hex digits (the PSK). No password: an open network.
  if (m > WIFI_PASS_MAX || (m > 0 && m < 8) || (m == WIFI_PASS_MAX && !allHex(rest))) return SET_PASS_LENGTH;
  memcpy(ssid, s, n);
  ssid[n] = 0;
  memcpy(pass, rest, m);
  pass[m] = 0;
  return SET_OK;
}

size_t buildDatagram(uint8_t* out, size_t cap, const uint8_t mac[6], const uint8_t* payload, size_t len) {
  if (cap < TELEM_DGRAM_HDR || len > cap - TELEM_DGRAM_HDR) return 0;
  out[0] = TELEM_DGRAM_MAGIC0;
  out[1] = TELEM_DGRAM_MAGIC1;
  out[2] = TELEM_DGRAM_VERSION;
  out[3] = out[4] = out[5] = 0;
  memcpy(out + 6, mac, 6);
  memcpy(out + TELEM_DGRAM_HDR, payload, len);
  return TELEM_DGRAM_HDR + len;
}

bool openDatagram(const uint8_t* data, size_t len, const uint8_t** payload, size_t* payload_len) {
  if (len >= TELEM_DGRAM_HDR && data[0] == TELEM_DGRAM_MAGIC0 && data[1] == TELEM_DGRAM_MAGIC1) {
    if (data[2] != TELEM_DGRAM_VERSION) return false;
    *payload = data + TELEM_DGRAM_HDR;
    *payload_len = len - TELEM_DGRAM_HDR;
    return true;
  }
  *payload = data;
  *payload_len = len;
  return true;
}

const char* passwordState(bool password_stored) { return password_stored ? "set" : "not set"; }

void logSafeCommand(const char* cmd, char* out, size_t cap) {
  if (!cap) return;
  const bool secret = !strncmp(cmd, "wifi", 4) || !strncmp(cmd, "set wifi_", 9);
  if (!secret) {
    snprintf(out, cap, "%s", cmd);
    return;
  }
  // The first two words only, then a note.
  const char* sp = strchr(cmd, ' ');
  const char* sp2 = sp ? strchr(sp + 1, ' ') : nullptr;
  const int words = sp2 ? (int)(sp2 - cmd) : (int)strlen(cmd);
  snprintf(out, cap, "%.*s (value not logged)", words, cmd);
}

int formatStatus(char* out, size_t cap, const Status& s) {
  char ip[16], srv[24], age[12];
  snprintf(ip, sizeof(ip), "%u.%u.%u.%u", s.ip[0], s.ip[1], s.ip[2], s.ip[3]);
  if (s.server_known)
    snprintf(srv, sizeof(srv), "%u.%u.%u.%u:%u", s.server_ip[0], s.server_ip[1], s.server_ip[2], s.server_ip[3],
             (unsigned)s.server_port);
  else
    snprintf(srv, sizeof(srv), "none");
  if (s.beacon_age_s >= 0) snprintf(age, sizeof(age), "%ld", (long)s.beacon_age_s);
  else snprintf(age, sizeof(age), "none");
  const bool conn = !strcmp(s.state, "connected");
  return snprintf(out, cap,
                  "WIFI mode=%s setting=%s ssid=\"%s\" state=%s rssi=%d ch=%d ip=%s server=%s via=%s beacon_age_s=%s "
                  "sent=%lu failed=%lu unsent=%lu beacons=%lu bad_beacons=%lu mdns=%lu reqs=%lu reconnects=%lu",
                  s.mode, s.setting, s.ssid, s.state, conn ? s.rssi_dbm : 0, conn ? s.channel : 0, ip, srv,
                  s.server_known ? s.server_via : "none", age, (unsigned long)s.sent, (unsigned long)s.failed,
                  (unsigned long)s.unsent, (unsigned long)s.beacons, (unsigned long)s.bad_beacons,
                  (unsigned long)s.mdns_answers, (unsigned long)s.reqs, (unsigned long)s.reconnects);
}

}  // namespace wifitelem
