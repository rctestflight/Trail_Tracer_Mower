// wifi_link: the rover's telemetry over a Wi-Fi network (setting radio_mode "wifi").
// The rover joins the network as a station, finds the PC telemetry server by its UDP beacon or by
// mDNS (wifi_telem_proto.h), and sends each ESP-NOW payload to it as one unicast UDP datagram.
// Telemetry only: the rover takes no commands over Wi-Fi. A REQ (backfill request) from the
// server goes to the caller, which passes it to the log.
//
// Threads and timing: call every function from loop(). None of them waits for the network:
// association, reconnection, the mDNS query and the scan run in the Wi-Fi and mDNS tasks, and
// service() only reads their state. Only begin() stops loop(): the radio start takes about 220 ms,
// as espnowStart() does. Call it from setup(), or while the vehicle stands still.
// ESP-NOW and this link never run together (the caller's choice, from radio_mode at boot), except for a "wifi test"
// in espnow mode: begin(), then end() after the test.
#pragma once
#include <Arduino.h>

namespace wifilink {

struct Hooks {
  Print* out;                                            // "WIFI ..." lines; must not block
  void (*event)(int32_t a, int32_t b, const char* text); // a vehicle log event (event 19)
  void (*frame)(const uint8_t* data, int len);           // a payload from the server, header removed
};

// Event 19 kinds (argument a).
enum : int32_t { EV_ASSOCIATED = 1, EV_LOST = 2, EV_SERVER = 3, EV_NO_SERVER = 4 };

void attach(const Hooks& hooks);                      // setup(), in either radio mode: no radio action
void begin(const char* ssid, const char* pass);       // radio on; association starts if ssid is not empty
bool started();
void end();                                               // "wifi test" in espnow mode: link off (sockets, mDNS, association)
void mdnsStop();                                          // 0.6.17: mDNS off until the next start (download memory)
void setCredentials(const char* ssid, const char* pass);  // "wifi set" / "wifi clear" while the link runs: a new association
void service(uint32_t now_ms);                            // each loop() pass
bool send(const uint8_t* payload, size_t len);            // one datagram to the server; false: not sent (counted)
bool connected();                                         // associated, with an IP address
bool ready();                                             // connected and the server known: a send can go out
bool server(uint8_t ip[4], uint16_t* port, const char** via);  // the server address, if known
uint32_t pollRx(uint32_t now_ms);  // update mode (service() stops): read the datagrams, as a held one holds a Wi-Fi receive
                                   // buffer (about 1.7 KB); beacons and server frames as in service(); returns the count

// "wifi scan": start, then scanService() each loop() pass prints one line per network.
// Works in either radio mode while the radio runs; espnow mode loses its frames during the scan.
bool scanStart();
void scanService();

// "wifi blast": a throughput test. Datagrams of <bytes> (16..1472) at <per_s> a second for <secs> s to the server's
// address, UDP port <port>; each starts "TB", then the sequence number and millis() (both 4 bytes, little-endian).
// A refused send (no lwIP buffer) is counted, never retried. One "WIFI blast" line a second and one at the end.
bool blastStart(uint16_t bytes, uint16_t per_s, uint16_t secs, uint16_t port);
void blastStop();

// "wifi": one status line. mode: the radio mode in effect; setting: radio_mode now (next start).
void printStatus(const char* mode, const char* setting, const char* stored_ssid);

}  // namespace wifilink
