// Settings page on the ESP32 (Wi-Fi mode), on the shared web server (include/web_server.h).
//
// GET  /settings       the page: static, gzipped in flash (src/web_settings_page.html, tools/gen_web_settings_page.py)
// GET  /api/settings   every setting of the settings table, from the table at run time (msc_fat_setting_json)
// POST /api/set        {"key": k, "value": v}: the command "set k v"; answer {"code", "reply", "value"}
// POST /api/save       the command "save"
// POST /api/erase      the command "config erase"
// POST /api/restart    a restart, refused while the vehicle moves
// POST /api/stop       the command "run stop" (a save is refused in autonomous mode)
//
// The server task does no drive work and reads no live setting: each request goes to loop() (service()), which runs
// the command through configCommand() as a radio command (the page uses the Wi-Fi link) or copies the settings for
// GET /api/settings. Command answers are the "CFG" answers of USB, code and text.
#pragma once
#include <Arduino.h>

namespace webcfg {

void begin();                                    // setup(): adds the page to the web server
void service(uint32_t now_ms);                   // loop(), Wi-Fi mode: runs a waiting page request
void answer(uint8_t code, const String& reply);  // loop(): the late answer of a request (a new update period)

}  // namespace webcfg
