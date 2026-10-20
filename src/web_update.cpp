// Update page on the ESP32: see include/web_update.h.
//
// The upload handlers run in the web server task. They stream the body to the writers of "ota url" and "stm url"
// (usb_ota::push_*); no file is kept in RAM. A start goes to loop() through a one-slot request (loopService), so that
// the vehicle-stationary check, update mode and the Rx STM32 state change run in loop() context, as for the commands.
// Rx STM32 file: into the tlog partition with the read-back check, then "stm url" goes on from its step 2 (command 41,
// the feed); the handler waits for the result before it reads on. ESP32 image: into the next slot with the image
// checks, its SHA-256 digest checked at the slot select; the restart comes after the answer to the client.
#include "web_update.h"

#include <Arduino.h>

#include <new>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "multipart.h"
#include "usb_ota.h"
#include "web_server.h"

// src/main.cpp
bool webUpdateGrant(uint8_t kind, unsigned long size, const char* sha, char* err, size_t cap);  // loop() context
void webUpdateModeEnd();                                                                       // loop() context
int stmUrlOutcome();        // 0 runs, 1 done, -1 failed, -2 idle
const char* stmUrlError();  // the reason after a failure
void stmStatusLine(String& out);
String rxStm32Version();
bool vehicleStationary();
extern "C" const char* rover_fw_version();

namespace webupd {
namespace {

enum : uint8_t { K_NONE, K_ESP, K_RX, Q_END };  // the file in hand; Q_END: a request to loop() only
constexpr uint32_t STALL_MS = 30000;     // no body byte for 30 s ends an upload (as "ota url")
constexpr uint32_t FEED_MS = 240000;     // the Rx STM32 update: download end to its result

// One request from the web server task to loop(): 0 free, 1 asked, 2 taken by loop(), 3 answered.
// Kinds: K_ESP and K_RX start an upload (webUpdateGrant); Q_END ends it (usb_ota::push_end). The end runs in loop()
// for its stack: the slot select checks the whole image (esp_image_verify), as "ota end" does over USB in loop().
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t s_q = 0;
uint8_t s_q_kind = 0;
unsigned long s_q_size = 0;
char s_q_sha[65];
bool s_q_ok = false;
char s_q_err[96];
volatile bool s_end_mode = false;  // a failed ESP32 upload: loop() ends update mode at once (not after 120 s)

// The last result and the phase, for GET /update/status (written and read in the web server task only).
enum : uint8_t { PH_IDLE, PH_RX, PH_FEED, PH_ESP, PH_RESTART };
const char* const PH_NAMES[] = {"idle", "receiving Rx STM32 file", "updating Rx STM32", "receiving ESP32 image",
                                "restarting"};
uint8_t s_phase = PH_IDLE;
int8_t s_ok = -1;  // last result: -1 none, 0 failed or refused, 1 done
char s_msg[200] = "";
// 0.6.2: the free heap over the last upload, for GET /update/status (heap_min there is since boot, and a good upload
// restarts the board before the next status read): at the request, the least during it (with the largest free block
// then), and at the answer.
uint32_t s_h0 = 0, s_hmin = 0, s_bmin = 0, s_hend = 0;
uint32_t s_m0 = 0, s_m1 = 0, s_m2 = 0;
// 0.6.5: failed heap allocations since boot (any task), with the size, the caps and the task of the last one. The hook
// runs inside the failed malloc: counts and copies only.
volatile uint32_t s_af_n = 0, s_af_size = 0, s_af_caps = 0, s_af_ms = 0;
char s_af_task[16] = "";
void allocFailed(size_t size, uint32_t caps, const char* fn) {
  (void)fn;
  s_af_n++;
  s_af_size = size;
  s_af_caps = caps;
  s_af_ms = (uint32_t)(esp_timer_get_time() / 1000);
  const char* t = pcTaskGetName(nullptr);
  strlcpy(s_af_task, t ? t : "?", sizeof(s_af_task));
}  // 0.6.3: the least free heap since boot at the request, before and after the answer
void heapSample() {
  const uint32_t h = ESP.getFreeHeap(), b = ESP.getMaxAllocHeap();
  if (h < s_hmin) s_hmin = h;
  if (b < s_bmin) s_bmin = b;
}

bool ask(uint8_t kind, unsigned long size, const char* sha, char* err, size_t cap) {
  portENTER_CRITICAL(&s_mux);
  const bool free_slot = s_q == 0;
  if (free_slot) {
    s_q_kind = kind;
    s_q_size = size;
    snprintf(s_q_sha, sizeof(s_q_sha), "%s", sha ? sha : "");
    s_q = 1;
  }
  portEXIT_CRITICAL(&s_mux);
  if (!free_slot) { snprintf(err, cap, "an update runs"); return false; }
  for (uint32_t t0 = millis();;) {
    if (s_q == 3) {
      const bool ok = s_q_ok;
      snprintf(err, cap, "%s", s_q_err);
      s_q = 0;
      return ok;
    }
    if (millis() - t0 > 5000) {  // loop() did not take it (it always does within a pass)
      portENTER_CRITICAL(&s_mux);
      const bool cancelled = s_q == 1;
      if (cancelled) s_q = 0;
      portEXIT_CRITICAL(&s_mux);
      if (cancelled) { snprintf(err, cap, "no answer from the main loop"); return false; }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// One upload request (form post or raw post).
struct Up {
  httpd_req_t* req;
  uint8_t kind;          // the part in hand: K_NONE (ignored), K_ESP, K_RX
  bool started;          // its upload runs
  bool esp_seen, esp_ok, rx_ok, stop;
  bool rx_late;          // an Rx STM32 file after the ESP32 image: not used
  int code;              // HTTP status of the answer
  unsigned long size;    // raw post: the body size; form post: 0 (not known)
  size_t left;           // body bytes not read yet
  char sha[65];          // raw post: ?sha256=
  char err[112];
  uint8_t buf[1436];     // one TCP segment
};

const char* kind_name(uint8_t k) { return k == K_ESP ? "ESP32 firmware" : "Rx STM32 firmware"; }

void fail(Up* u, int code, const char* fmt, ...) {
  // After a good Rx STM32 update in the same form post, the message says so first.
  const size_t n = u->rx_ok ? strlen(s_msg) + 1 : 0;
  if (n > 1) s_msg[n - 1] = ' ';
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s_msg + (n < sizeof(s_msg) ? n : 0), sizeof(s_msg) - (n < sizeof(s_msg) ? n : 0), fmt, ap);
  va_end(ap);
  u->code = code;
  u->stop = true;
  s_ok = 0;
}

bool on_part(void* ctx, const char* name, const char*) {
  Up* u = (Up*)ctx;
  u->kind = !strcmp(name, "esp32") ? K_ESP : !strcmp(name, "rxstm32") ? K_RX : K_NONE;
  u->started = false;
  if (u->kind == K_RX && u->esp_seen) {
    // The ESP32 image restarts the ESP32, and the running ESP32 feeds the Rx STM32: the Rx STM32 file goes first.
    // The ESP32 image is in its slot already: it stays, and this file is not used.
    u->kind = K_NONE;
    u->rx_late = true;
  }
  return true;
}

bool on_data(void* ctx, const uint8_t* b, size_t n) {
  Up* u = (Up*)ctx;
  if (u->kind == K_NONE || n == 0) return true;
  if (!u->started) {
    if (!ask(u->kind, u->size, u->sha, u->err, sizeof(u->err))) {
      fail(u, 409, "%s refused: %s", kind_name(u->kind), u->err);
      return false;
    }
    u->started = true;
    if (u->kind == K_ESP) u->esp_seen = true;
    s_phase = u->kind == K_ESP ? PH_ESP : PH_RX;
  }
  if (!usb_ota::push_write(b, n, u->err, sizeof(u->err))) {
    if (u->kind == K_ESP) s_end_mode = true;
    fail(u, 400, "%s refused: %s", kind_name(u->kind), u->err);
    return false;
  }
  return true;
}

// The Rx STM32 file is in flash: "stm url" goes on in loop() (command 41, the feed). Wait for its result.
bool wait_rx(Up* u) {
  s_phase = PH_FEED;
  const uint32_t t0 = millis();
  int r;
  while ((r = stmUrlOutcome()) == 0) {
    if (millis() - t0 > FEED_MS) { fail(u, 500, "Rx STM32 firmware: no result in %lu s", (unsigned long)(FEED_MS / 1000)); return false; }
    heapSample();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (r != 1) {
    fail(u, 400, "Rx STM32 firmware not updated: %s", r == -1 ? stmUrlError() : "the update stopped");
    return false;
  }
  for (int i = 0; i < 60 && rxStm32Version() == "unknown"; i++) vTaskDelay(pdMS_TO_TICKS(100));  // read 2 s after the start
  u->rx_ok = true;
  snprintf(s_msg, sizeof(s_msg), "Rx STM32 firmware updated: now %s.", rxStm32Version().c_str());
  s_ok = 1;
  return true;
}

bool on_end(void* ctx) {
  Up* u = (Up*)ctx;
  if (u->kind == K_NONE || !u->started) return true;  // an empty file input
  u->started = false;
  if (!ask(Q_END, 0, nullptr, u->err, sizeof(u->err))) {
    if (u->kind == K_ESP) s_end_mode = true;
    // Rx STM32: "stm url" reports the failure through its own state (fetch_poll); the reason is the same text.
    fail(u, 400, "%s refused: %s", kind_name(u->kind), u->err);
    return false;
  }
  if (u->kind == K_RX) return wait_rx(u);
  u->esp_ok = true;
  return true;
}

// Read the body: each piece to the multipart parser (form post) or straight to the writer (raw post).
void receive(Up* u, multipart::Parser* mp) {
  size_t& left = u->left;
  uint32_t last = millis();
  while (left && !u->stop) {
    const int k = httpd_req_recv(u->req, (char*)u->buf, left < sizeof(u->buf) ? left : sizeof(u->buf));
    if (k == HTTPD_SOCK_ERR_TIMEOUT) {
      if (millis() - last < STALL_MS) continue;
      fail(u, 408, "upload stopped: no data for 30 s");
      break;
    }
    heapSample();
    if (k <= 0) { fail(u, 400, "upload stopped: the connection ended"); break; }
    last = millis();
    left -= k;
    if (mp) {
      const multipart::Result r = mp->feed(u->buf, k);
      if (r == multipart::MP_ERROR) fail(u, 400, "not a valid form post: %s", mp->error());
    } else if (!on_data(u, u->buf, k)) {
      break;
    }
  }
  // A short rest after a refusal (an empty field, the closing boundary) is read: a close with unread bytes sends a TCP
  // reset, and the client can lose the answer.
  while (u->stop && left && left <= 8192) {
    const int k = httpd_req_recv(u->req, (char*)u->buf, left < sizeof(u->buf) ? left : sizeof(u->buf));
    if (k <= 0) break;
    left -= k;
  }
  if (!mp && !u->stop) on_end(u);
  if (mp && !u->stop && mp->state() != multipart::MP_DONE) fail(u, 400, "not a valid form post: it ended early");
  // The client went away in the middle of a file, or a refusal came while a writer runs: the writer stops.
  if (u->started || u->stop) usb_ota::push_abort(s_msg);
  if (u->started && u->kind == K_ESP) s_end_mode = true;
}

void json_str(char* dst, size_t cap, const char* s) {  // a JSON string body: quotes and backslashes escaped
  size_t n = 0;
  for (; *s && n + 2 < cap; s++) {
    if (*s == '"' || *s == '\\') dst[n++] = '\\';
    dst[n++] = (*s >= ' ') ? *s : ' ';
  }
  dst[n] = 0;
}

esp_err_t answer(Up* u) {
  if (u->code == 200 && !u->rx_ok && !u->esp_ok) {
    snprintf(s_msg, sizeof(s_msg), "No file came: choose a file first.");
    u->code = 400;
    s_ok = 0;
  }
  if (u->code == 200 && u->esp_ok) {
    snprintf(s_msg + strlen(s_msg), sizeof(s_msg) - strlen(s_msg), "%sESP32 firmware written: the Rx restarts.%s",
             u->rx_ok ? " " : "", u->rx_late ? " The Rx STM32 file was not used: it must come before the ESP32 image." : "");
    s_phase = PH_RESTART;
    s_ok = 1;
  } else {
    s_phase = PH_IDLE;
  }
  static const char* const ST[] = {"200 OK", "400 Bad Request", "408 Request Timeout", "409 Conflict", "500 Internal Server Error"};
  const char* st = u->code == 200 ? ST[0] : u->code == 400 ? ST[1] : u->code == 408 ? ST[2] : u->code == 409 ? ST[3] : ST[4];
  httpd_resp_set_status(u->req, st);
  char m[240];
  json_str(m, sizeof(m), s_msg);
  char a[48] = "";
  const bool html = httpd_req_get_hdr_value_str(u->req, "Accept", a, sizeof(a)) == ESP_OK && strstr(a, "text/html");
  char b[420];
  if (html) {  // a form post with no script: a small result page
    httpd_resp_set_type(u->req, "text/html; charset=utf-8");
    snprintf(b, sizeof(b),
             "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>%s<title>Update</title>"
             "<body style='font-family:\"Roboto Mono\",monospace;max-width:36em;margin:auto;padding:16px'><p>%s</p>"
             "<p><a href=/update>Back to the update page</a></p>",
             u->esp_ok && u->code == 200 ? "<meta http-equiv=refresh content='20;url=/update'>" : "", s_msg);
  } else {
    httpd_resp_set_type(u->req, "application/json");
    snprintf(b, sizeof(b), "{\"ok\":%s,\"restart\":%s,\"msg\":\"%s\",\"up_heap_start\":%lu,\"up_heap_min\":%lu,\"up_maxblk_min\":%lu,\"up_bootmin_start\":%lu,\"up_bootmin_end\":%lu}",
             u->code == 200 ? "true" : "false", u->code == 200 && u->esp_ok ? "true" : "false", m, (unsigned long)s_h0,
             (unsigned long)s_hmin, (unsigned long)s_bmin, (unsigned long)s_m0, (unsigned long)ESP.getMinFreeHeap());
  }
  httpd_resp_set_hdr(u->req, "Connection", "close");
  httpd_resp_sendstr(u->req, b);
  return ESP_OK;
}

// A quick refusal before any byte of the body is read (moving, busy): the client gets its answer at once.
bool refuse_early(Up* u) {
  const char* why = !vehicleStationary() ? "Stop the vehicle first"
                    : (usb_ota::busy() || stmUrlOutcome() == 0) ? "an update runs" : nullptr;
  if (!why) return false;
  fail(u, 409, "Not updated: %s", why);
  return true;
}

esp_err_t finish(Up* u) {
  const bool all_read = u->left == 0;
  heapSample();
  s_m1 = ESP.getMinFreeHeap();
  answer(u);
  s_hend = ESP.getFreeHeap();
  s_m2 = ESP.getMinFreeHeap();
  const bool restart = u->code == 200 && u->esp_ok;
  free(u);
  if (restart) usb_ota::push_restart();  // loop() restarts into the new image (after the USB detach, 800 ms)
  // ESP_FAIL closes the connection: the body that was not read is not read (a refusal does not wait for 1 MB).
  return all_read ? ESP_OK : ESP_FAIL;
}

Up* new_up(httpd_req_t* req) {
  s_h0 = s_hmin = s_hend = ESP.getFreeHeap();
  s_m0 = s_m1 = s_m2 = ESP.getMinFreeHeap();
  s_bmin = ESP.getMaxAllocHeap();
  Up* u = (Up*)calloc(1, sizeof(Up));
  heapSample();
  if (!u) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_sendstr(req, "{\"ok\":false,\"restart\":false,\"msg\":\"no memory for the upload\"}");
    return nullptr;
  }
  u->req = req;
  u->code = 200;
  u->left = req->content_len;
  s_msg[0] = 0;
  s_ok = -1;
  return u;
}

esp_err_t post_form(httpd_req_t* req) {
  Up* u = new_up(req);
  if (!u) return ESP_FAIL;
  char ct[128] = "", bnd[72];
  httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct));
  if (!multipart::boundary_from(ct, bnd, sizeof(bnd))) {
    fail(u, 400, "Not updated: not a form post (multipart/form-data)");
    return finish(u);
  }
  if (refuse_early(u)) return finish(u);
  multipart::Parser* mp = new (std::nothrow) multipart::Parser();
  heapSample();
  if (!mp) { fail(u, 500, "Not updated: no memory for the form"); return finish(u); }
  mp->begin(bnd, multipart::Callbacks{u, on_part, on_data, on_end});
  receive(u, mp);
  delete mp;
  return finish(u);
}

esp_err_t post_raw(httpd_req_t* req, uint8_t kind) {
  Up* u = new_up(req);
  if (!u) return ESP_FAIL;
  char q[96];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) httpd_query_key_value(q, "sha256", u->sha, sizeof(u->sha));
  u->kind = kind;
  u->size = req->content_len;
  if (refuse_early(u)) return finish(u);
  receive(u, nullptr);
  return finish(u);
}
esp_err_t post_esp(httpd_req_t* req) { return post_raw(req, K_ESP); }
esp_err_t post_rx(httpd_req_t* req) { return post_raw(req, K_RX); }

esp_err_t get_status(httpd_req_t* req) {
  esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(esp_ota_get_running_partition(), &st);
  String stm;
  stmStatusLine(stm);
  char m[240], sl[120];
  json_str(m, sizeof(m), s_msg);
  json_str(sl, sizeof(sl), stm.c_str());
  const uint8_t ph = usb_ota::busy() && s_phase == PH_IDLE ? PH_ESP : stmUrlOutcome() == 0 && s_phase == PH_IDLE ? PH_FEED : s_phase;
  static char b[900];  // 0.6.5: static (one web server task), 900 B with the allocation fields
  snprintf(b, sizeof(b),
           "{\"esp32\":\"%s\",\"rx_stm32\":\"%s\",\"valid\":%s,\"stationary\":%s,\"phase\":\"%s\",\"got\":%lu,"
           "\"ok\":%s,\"msg\":\"%s\",\"stm\":\"%s\",\"uptime_ms\":%lu,\"heap\":%lu,\"heap_min\":%lu,\"maxblk\":%lu,"
           "\"web_stack_free\":%u,\"up_heap_start\":%lu,\"up_heap_min\":%lu,\"up_maxblk_min\":%lu,\"up_heap_end\":%lu,"
           "\"up_bootmin_start\":%lu,\"up_bootmin_end\":%lu,\"up_bootmin_answered\":%lu,\"alloc_fail\":%lu,"
           "\"alloc_fail_size\":%lu,\"alloc_fail_caps\":%lu,\"alloc_fail_task\":\"%s\",\"alloc_fail_ms\":%lu}",
           rover_fw_version(), rxStm32Version().c_str(), st == ESP_OTA_IMG_PENDING_VERIFY ? "false" : "true",
           vehicleStationary() ? "true" : "false", PH_NAMES[ph], (unsigned long)usb_ota::push_got(),
           s_ok < 0 ? "null" : s_ok ? "true" : "false", m, sl, (unsigned long)millis(),
           (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(),
           (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)),  // least free stack of the web server task
           (unsigned long)s_h0, (unsigned long)s_hmin, (unsigned long)s_bmin, (unsigned long)s_hend, (unsigned long)s_m0,
           (unsigned long)s_m1, (unsigned long)s_m2, (unsigned long)s_af_n, (unsigned long)s_af_size,
           (unsigned long)s_af_caps, s_af_task, (unsigned long)s_af_ms);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, b);
}

// The page. The script sends the same form post as the form itself (XMLHttpRequest, for the progress), joins an
// Rx STM32 pair (.commit_header and .bin.encrypted, header first) into one file, and reads /update/status after it.
const char PAGE[] = R"HTML(<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Rx update</title>
<link rel=stylesheet href="https://fonts.googleapis.com/css2?family=Roboto+Mono&display=swap" media=print onload="this.media='all'">
<style>:root{color-scheme:light dark;--bg:#fff;--fg:#1d1d1d;--mut:#666;--ok:#17702f;--bad:#b3261e}
@media(prefers-color-scheme:dark){:root{--bg:#141414;--fg:#e6e6e6;--mut:#9a9a9a;--ok:#73d38f;--bad:#ff8a80}}
body{font-family:"Roboto Mono",monospace;background:var(--bg);color:var(--fg);max-width:36em;margin:auto;padding:16px;line-height:1.5}
label{display:block;margin-top:1em}input{width:100%;font:inherit}button{font:inherit;padding:.4em 1.5em;margin-top:1.2em}
progress{width:100%}.ok{color:var(--ok)}.bad{color:var(--bad)}small{color:var(--mut)}a{color:inherit}</style></head><body>
<h1>Update</h1><p id=v>Firmware now: ...</p>
<form id=f method=post action=/update enctype=multipart/form-data>
<label for=r>Rx STM32 firmware: one file, or the .commit_header and .bin.encrypted pair</label>
<input id=r name=rxstm32 type=file multiple>
<label for=e>ESP32 firmware (.bin)</label><input id=e name=esp32 type=file accept=.bin>
<button id=b>Update</button></form>
<progress id=p max=1 value=0 hidden></progress><p id=s></p>
<p><small>Stop the vehicle first, and keep the power on until the page says done. If an update fails, the Rx keeps
its old firmware. If the Rx does not start, hold BOOT, plug in USB and use the web programmer.</small></p>
<p><a href=/>Back</a></p>
<script>
const $=i=>document.getElementById(i),S=(t,c)=>{$('s').textContent=t;$('s').className=c||''},W=ms=>new Promise(r=>setTimeout(r,ms));
async function st(){try{const j=await(await fetch('/update/status',{cache:'no-store'})).json();
$('v').textContent='Firmware now: ESP32 '+j.esp32+', Rx STM32 '+j.rx_stm32;return j}catch(e){return null}}
st();
$('f').onsubmit=async ev=>{ev.preventDefault();
const r=[...$('r').files].sort((a,b)=>a.size-b.size),e=$('e').files[0];
if(!r.length&&!e){S('Choose a file first.','bad');return}
const d=new FormData();
if(r.length)d.append('rxstm32',r.length>1?new Blob(r):r[0],r.length>1?'rx_stm32.bin':r[0].name);
if(e)d.append('esp32',e,e.name);
const o=await st()||{},x=new XMLHttpRequest();$('b').disabled=true;$('p').hidden=false;$('p').value=0;
S('Sending... '+(r.length?'The Rx STM32 goes first (about 30 s). ':'')+'Keep the power on.');
x.upload.onprogress=p=>{if(p.lengthComputable)$('p').value=p.loaded/p.total};
x.onload=x.onerror=async()=>{let j=null;try{j=JSON.parse(x.responseText)}catch(_){}
if(!j){const s=await st();j={ok:false,msg:s&&s.msg?s.msg:'No answer from the Rx.'}}
if(j.ok&&j.restart){S(j.msg+' Waiting for it...');let g=0,k=0;
for(let i=0;i<45&&!k;i++){await W(2000);const s=await st();if(!s)g=1;
else if(g||s.uptime_ms<(o.uptime_ms||1e12)||s.esp32!=o.esp32){S('Done. ESP32 '+s.esp32+', Rx STM32 '+s.rx_stm32+'.','ok');k=1}}
if(!k)S('The Rx did not answer after its restart. Reload this page in a minute.','bad')}
else{S(j.msg,j.ok?'ok':'bad');st()}
$('b').disabled=false;$('p').hidden=true};
x.open('POST','/update');x.send(d)};
</script></body></html>
)HTML";

esp_err_t get_page(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, PAGE, sizeof(PAGE) - 1);
}

void reg(httpd_handle_t server) {
  static const httpd_uri_t uris[] = {
      {"/update", HTTP_GET, get_page, nullptr},
      {"/update/status", HTTP_GET, get_status, nullptr},
      {"/update", HTTP_POST, post_form, nullptr},
      {"/update/esp32", HTTP_POST, post_esp, nullptr},
      {"/update/rxstm32", HTTP_POST, post_rx, nullptr},
  };
  for (const httpd_uri_t& u : uris) httpd_register_uri_handler(server, &u);
}

}  // namespace

void setup() {
  heap_caps_register_failed_alloc_callback(allocFailed);  // 0.6.5: counted for GET /update/status
  web::addPage("/update", "Update", reg);
}

void loopService() {
  if (s_q == 1) {
    portENTER_CRITICAL(&s_mux);
    const bool mine = s_q == 1;
    if (mine) s_q = 2;
    portEXIT_CRITICAL(&s_mux);
    if (mine) {
      s_q_err[0] = 0;
      s_q_ok = s_q_kind == Q_END ? usb_ota::push_end(s_q_err, sizeof(s_q_err))
                                 : webUpdateGrant(s_q_kind, s_q_size, s_q_sha, s_q_err, sizeof(s_q_err));
      s_q = 3;
    }
  }
  if (s_end_mode && !usb_ota::busy()) {
    s_end_mode = false;
    webUpdateModeEnd();
  }
}

}  // namespace webupd
