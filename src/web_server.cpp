// Web server on the ESP32: see include/web_server.h.
#include "web_server.h"
#include <Arduino.h>
#include "wifi_link.h"

extern "C" const char* rover_fw_version();  // src/platform/board.inc

namespace web {
namespace {

struct Page { const char* path; const char* title; RegisterFn reg; };
constexpr int PAGES_MAX = 6;
constexpr int SOCKETS_MAX = 3;
Page s_pages[PAGES_MAX];
int s_n = 0;
httpd_handle_t s_server = nullptr;
uint32_t s_try_ms = 0;

esp_err_t index_get(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_sendstr_chunk(req,
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width'>"
    "<title>Trail Tracer Rx</title><style>body{font-family:system-ui,sans-serif;margin:1.5em;max-width:40em}"
    "@media(prefers-color-scheme:dark){body{background:#111;color:#ddd}a{color:#8af}}</style></head><body>"
    "<h1>Trail Tracer Rx</h1><p>Firmware ");
  httpd_resp_sendstr_chunk(req, rover_fw_version());
  httpd_resp_sendstr_chunk(req, "</p><ul>");
  for (int i = 0; i < s_n; i++) {
    char b[160];
    snprintf(b, sizeof(b), "<li><a href='%s'>%s</a></li>", s_pages[i].path, s_pages[i].title);
    httpd_resp_sendstr_chunk(req, b);
  }
  httpd_resp_sendstr_chunk(req, "</ul></body></html>");
  return httpd_resp_sendstr_chunk(req, nullptr);
}

}  // namespace

void addPage(const char* path, const char* title, RegisterFn reg) {
  if (s_n < PAGES_MAX) s_pages[s_n++] = Page{path, title, reg};
}

void service(uint32_t now_ms) {
  if (s_server || !wifilink::connected()) return;
  if (s_try_ms && now_ms - s_try_ms < 10000) return;  // a failed start (no memory) tries again after 10 s
  s_try_ms = now_ms ? now_ms : 1;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.task_priority = 0;        // below loop(): the server runs only while loop() waits for a frame (at 1 a
                                // page request split loop() passes: 21 lost frames in 10 min at 2960 us, 2026-10-17)
  cfg.stack_size = 6144;
  cfg.max_open_sockets = SOCKETS_MAX;
  cfg.max_uri_handlers = 16;
  // An upload holds the server task for its whole length. Connections that wait for it hold Wi-Fi buffers: with
  // a second client polling, the free heap fell to 1.3 KB and the upload failed (bench, 2026-10-17). So no purge of
  // an open connection (it could close the upload), and a short queue: a client beyond it is refused at once.
  cfg.lru_purge_enable = false;
  cfg.backlog_conn = 1;
  if (httpd_start(&s_server, &cfg) != ESP_OK) { s_server = nullptr; return; }
  static const httpd_uri_t idx = {"/", HTTP_GET, index_get, nullptr};
  httpd_register_uri_handler(s_server, &idx);
  for (int i = 0; i < s_n; i++) s_pages[i].reg(s_server);
}

bool running() { return s_server != nullptr; }
void stop() {
  if (!s_server) return;
  httpd_stop(s_server);
  s_server = nullptr;
  s_try_ms = 0;  // service() starts it again at its next call
}
httpd_handle_t handle() { return s_server; }

}  // namespace web
