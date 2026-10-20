// Telemetry page on the ESP32 (Wi-Fi mode), on the shared web server (include/web_server.h).
//
// GET /telemetry             the page: static, gzipped in flash (src/web_telemetry_page.html,
//                            tools/gen_web_telemetry_page.py); it polls /api/telemetry at 1 Hz (2 Hz as an option)
//                            while its tab is visible
// GET /api/telemetry?def=1   the field list from the telemetry table: name, record, group, label, unit, decimals,
//                            kind, place in the short set (lib/tlm_page)
// GET /api/telemetry[?all=1] one snapshot: the values of the short set (all=1: of every field), the ages of the status
//                            and of the sample, the free heap, the lowest free heap and the largest free block
//
// The server task reads no live value: a request asks loop() (service()) for a copy of the newest status record, the
// newest log sample and the table values, then formats the copy. Nothing is added to the frame path.
#pragma once
#include <stdint.h>

namespace webtlm {

void begin();                   // setup(): adds the page to the web server
void service(uint32_t now_ms);  // loop(), Wi-Fi mode: takes the copy that a waiting request asks for

}  // namespace webtlm
