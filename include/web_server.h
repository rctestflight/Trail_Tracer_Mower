// Web server on the ESP32 (0.5.116): one HTTP server for the pages the Rx hosts in Wi-Fi mode.
// The server starts once the Wi-Fi station is connected and stays up. Each page module adds its handlers with
// web::addPage(); the server calls every registrar when it starts. GET / lists the pages.
// Driving keeps priority: the server task runs at priority 0 (below loop()), and a handler only reads or copies data;
// work that touches the drive (a save, an update) goes through the same rules as the USB and radio commands.
#pragma once
#include <stdint.h>
#include "esp_http_server.h"

namespace web {

typedef void (*RegisterFn)(httpd_handle_t server);

// A page module: a path and a title for the list on GET /, and the function that registers its handlers.
void addPage(const char* path, const char* title, RegisterFn reg);

void service(uint32_t now_ms);  // from loop() in Wi-Fi mode: starts the server once the station is connected
bool running();
void stop();  // the server ends and frees its heap (the raw stream needs it); service() starts it again later
httpd_handle_t handle();

}  // namespace web
