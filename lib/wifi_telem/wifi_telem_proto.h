// wifi_telem_proto: the parts of the Wi-Fi telemetry link that need no ESP32 (radio_mode "wifi").
// The server beacon, the "wifi set" arguments, the status line and the log text of a command.
// No Arduino or ESP-IDF dependency, so it also builds and runs on a PC (test/telemetry_host).
#pragma once
#include <stddef.h>
#include <stdint.h>

// ---- Values shared with the PC telemetry server (docs/wifi_interface.md) ------------------------
// The PC telemetry server uses the same values. They are here and nowhere else, so a change is one edit.
//
// Beacon: the server sends a UDP broadcast to TELEM_BEACON_PORT once a second. Payload, ASCII:
//   TTSRV1 <server ipv4> 47701        for example "TTSRV1 192.168.4.1 47701"
// A line end (CR, LF) or a NUL at the end is allowed; nothing else. The rover sends to the address
// and port in the beacon (the port is 47701 by agreement; the rover takes it from the beacon).
// mDNS: service TELEM_MDNS_SERVICE, protocol TELEM_MDNS_PROTO, port 47701, TXT "v=1".
// Datagram, rover -> server, unicast UDP to the server's telemetry port:
//   "TT" (2 bytes), version 1 (1 byte), 3 reserved zero bytes, the rover's 6 MAC bytes,
//   then one ESP-NOW payload, unchanged (TELEM_DGRAM_HDR = 12 bytes before it).
#define TELEM_UDP_PORT 47701                 // server telemetry port (rover -> server)
#define TELEM_BEACON_PORT 47700              // server beacon broadcast; the rover listens on it
#define TELEM_BEACON_PREFIX "TTSRV1"
#define TELEM_MDNS_SERVICE "_trailtracer"
#define TELEM_MDNS_PROTO "_udp"
#define TELEM_MDNS_TXT_KEY "v"
#define TELEM_MDNS_TXT_VALUE "1"
#define TELEM_DGRAM_MAGIC0 'T'
#define TELEM_DGRAM_MAGIC1 'T'
#define TELEM_DGRAM_VERSION 1
#define TELEM_DGRAM_HDR 12
// -----------------------------------------------------------------------------------------------

#define TELEM_NO_SERVER_MS 30000u      // association with no server beacon or mDNS answer for this long: "no telemetry server on this network"
#define WIFI_SSID_MAX 32               // bytes (802.11)
#define WIFI_PASS_MAX 64               // a WPA2 passphrase is 8..63 characters; 64 characters = the PSK in hex

namespace wifitelem {

// One datagram: the 12-byte header (magic, version, reserved, MAC), then the ESP-NOW payload.
// Returns its length, or 0 if it does not fit in cap.
size_t buildDatagram(uint8_t* out, size_t cap, const uint8_t mac[6], const uint8_t* payload, size_t len);
// A datagram from the server: if it has the header, the payload after it; else the bytes as they
// are (a bare ESP-NOW payload). false: a header with another version.
bool openDatagram(const uint8_t* data, size_t len, const uint8_t** payload, size_t* payload_len);

// Beacon payload -> server address. false: not a beacon, or a bad one (the caller counts it).
bool parseBeacon(const uint8_t* data, size_t len, uint8_t ip[4], uint16_t* port);

// "wifi set" arguments (the text after "wifi set "):
//   <ssid> <password>       the SSID ends at the first space
//   "<ssid>" <password>     an SSID with spaces, in double quotes
// The password is the rest of the line after one space, spaces included; none = an open network.
// Results:
enum SetResult { SET_OK = 0, SET_USAGE = 1, SET_SSID_LENGTH = 2, SET_PASS_LENGTH = 3 };
SetResult parseWifiSet(const char* args, char ssid[WIFI_SSID_MAX + 1], char pass[WIFI_PASS_MAX + 1]);

// What "list" and "get" show for wifi_password. They never show the password.
const char* passwordState(bool password_stored);

// The text that the vehicle log (event 8) keeps of a config command. A command that can carry the
// Wi-Fi password ("wifi ..." or "set wifi_...") keeps its first two words only; any other command
// is kept as it is.
void logSafeCommand(const char* cmd, char* out, size_t cap);

// One "WIFI" status line (USB command "wifi"). The struct has no password field: the line cannot
// carry it.
struct Status {
  const char* mode;       // radio mode in effect: "wifi" or "espnow"
  const char* setting;    // radio_mode setting (in effect from the next start)
  const char* ssid;       // "" = no network stored
  const char* state;      // association state, for example "connected"
  int rssi_dbm;           // valid when connected
  int channel;            // valid when connected
  uint8_t ip[4];          // own address; 0.0.0.0 = none
  bool server_known;
  uint8_t server_ip[4];
  uint16_t server_port;
  const char* server_via; // "beacon" or "mdns"
  int32_t beacon_age_s;   // -1 = no beacon since boot
  uint32_t sent, failed, unsent, beacons, bad_beacons, mdns_answers, reqs, reconnects;
};
int formatStatus(char* out, size_t cap, const Status& s);

}  // namespace wifitelem
