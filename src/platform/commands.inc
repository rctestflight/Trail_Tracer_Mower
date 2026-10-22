// Settings and run commands from USB, the radio and the settings page ("get", "set", "save", "run").
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

// Config commands and restarts, from loop() (docs/espnow_protocol.md).
uint8_t runCommand(bool start, const char* source, String& reply);
// The last ACK of a command with an effect, for a retry with the same seq (the ACK was lost).
static uint8_t last_mac[6];
static uint16_t last_seq = 0;
static bool have_last = false;
static uint8_t last_result = 0;
static String last_reply;
void espnowCacheAck(const uint8_t* mac, uint16_t seq, uint8_t result, const String& reply){
  memcpy(last_mac, mac, 6);
  last_seq = seq;
  last_result = result;
  last_reply = reply;
  have_last = true;
}
void espnowCommands(){
  if (espnow_stop_request) {
    espnow_stop_request = false;
    uint8_t mac[6];
    memcpy(mac, espnow_stop_mac, 6);
    String reply;
    runCommand(false, "STOP frame", reply);
    espnowSendAck(mac, espnow_stop_seq, 0, "stop");
  }
  if (espnow_restart_request) {
    const int what = espnow_restart_request;
    espnow_restart_request = 0;
    if (!vehicleStationary()) {  // never a restart while the vehicle moves (the main-branch baseline has no remote restart)
      usbOut.printf("ESP-NOW restart request %d refused: Stop the vehicle first\n", what);
      telemetryEvent(rec::EV_RESTART_REFUSED, what, 1, "ESP-NOW restart refused: the vehicle moves");
    } else {
      usbOut.printf("ESP-NOW restart request %d\n", what);
      if (what == 2) restartIntoRom();
      restartApp();
    }
  }
  if (espnow_busy_pending) {
    uint8_t mac[6];
    memcpy(mac, espnow_busy_mac, 6);  // copy before the flag is cleared: the callback may write again
    const uint16_t seq = espnow_busy_seq;
    espnow_busy_pending = false;
    espnowSendAck(mac, seq, 3, "busy");
  }
  if (espnow_list_index >= 0 && millis() - espnow_list_ms >= 5) {  // one key every 5 ms
    espnow_list_ms = millis();
    String line;
    if (espnow_list_saved ? msc_fat_key_pair_at(&g_deviceConfig, &g_saved_config, espnow_list_index, &line)
                          : msc_fat_key_at(&g_deviceConfig, espnow_list_index, &line)) {
      espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, 0, line.c_str());
      espnow_list_index = espnow_list_index + 1;
    } else {
      espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, 0,  // key count: a lost item shows; def: settings_def.json version
                    (String("end ") + espnow_list_index + " def " + SETTINGS_DEF_VERSION).c_str());
      espnow_list_index = -1;
    }
  }
  static uint32_t bad_key_logged = 0;
  if (espnow_bad_key != bad_key_logged) {
    bad_key_logged = espnow_bad_key;
    telemetryEvent(rec::EV_BAD_KEY, (int32_t)bad_key_logged, 0, "CMD/RESTART with a wrong fleet key");
  }
  if (!espnow_cmd_pending) return;
  // A retry (same sender, same seq: the ACK was lost) gets the cached ACK and does not run again.
  if (!strcmp(espnow_cmd_text, "list") || !strcmp(espnow_cmd_text, "list saved")) {
    espnow_list_saved = espnow_cmd_text[4] != 0;  // 0.6.15: "list saved": each key with its value of the next start
    if (espnow_list_saved) msc_fat_saved_config(&g_saved_config);
    espnow_list_index = 0;  // the CMD's mac and seq stay in hand until "end"
    espnow_list_ms = 0;
  } else if (have_last && espnow_cmd_seq == last_seq && !memcmp(espnow_cmd_mac, last_mac, 6)
             && strncmp(espnow_cmd_text, "get ", 4) && strncmp(espnow_cmd_text, "defget ", 7)) {
    // Only a command with an effect (set, save, run) is answered from the cache. A read ("get ...")
    // is answered fresh: a cached "get run" said running after a STOP had stopped the vehicle
    // (PC telemetry simulation, 1 in 18 runs at 30 % loss).
    espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, last_result, last_reply.c_str());
  } else {
    String reply;
    uint8_t r;
    if ((update_mode || stmUrlBusy()) && !updateAllowed(espnow_cmd_text)) { reply = "refused: an update runs"; r = 4; }
    else if (updateCommand(espnow_cmd_text, true, reply)) r = updateResult(reply);  // the ACK text = the USB line
    else r = configCommand(espnow_cmd_text, true, reply);
    if (period_ack_want) periodAckDefer(false, espnow_cmd_mac, espnow_cmd_seq);  // 0.5.104: answer after the Rx STM32
    else espnowSendAck(espnow_cmd_mac, espnow_cmd_seq, r, reply.c_str());
    espnowCacheAck(espnow_cmd_mac, espnow_cmd_seq, r, reply);
  }
  espnow_cmd_pending = false;
}

// One config command, "get <key>", "set <key> <value>" or "save", from ESP-NOW (CMD 0x10) or
// from the USB serial port. Result: 0 ok, 1 unknown key, 2 bad value, 4 refused.
// Keys that change only while the vehicle stands still (vehicleStationary()); every other key is
// live. Reasons: a lost wire or a changed charge limit in the STM32 (rx_channel, enable_sensor_2, CHARGE_*), a jump of
// the steering sign, a servo centre or a limit (pid_direction, PWM_OUT_*), a step of the drive
// command or a change of mission (rover_* calibration, enable, direction, beacons; BTxFollowChannel),
// a Wi-Fi start that stops loop() for about 220 ms (espnow_enabled, telemetry_enabled).
// docs/rover_guide.md, "Settings", has the same list.
static const char* const STATIONARY_KEYS[] = {
  "rx_channel", "enable_sensor_2", "pid_direction",
  "drive_output", "steer_output", "throttle_output", "steer_center_us", "steer_min_us", "steer_max_us",
  "throttle_center_us", "throttle_min_us", "throttle_max_us",
  "PWM_OUT_3_DEFAULT", "PWM_OUT_3_MIN", "PWM_OUT_3_MAX", "PWM_OUT_4_DEFAULT", "PWM_OUT_4_MIN", "PWM_OUT_4_MAX",
  "CHARGE_VBATT_STOP", "CHARGE_MAX_CURRENT", "CHARGE_MAX_SECONDS", "BTxFollowChannel",
  "espnow_enabled", "telemetry_enabled",
  "rover_mode", "rover_goal_cw", "rover_sensor_height_mm", "rover_roam_speed_mps",
  "rover_entry_btx", "rover_exit_btx", "imu_required",
};
bool stationaryKey(const char* key){
  if (!strncmp(key, "dsp_", 4)) return true;  // a DSP change stops the STM32 run for a moment
  for (const char* k : STATIONARY_KEYS) if (!strcmp(k, key)) return true;
  return false;
}
bool vehicleStationary();

// Run commands (docs/control_api.md in Trail-Tracer-PC-Telemetry): "run start" enters mode 2
// (autonomous: the rover state machine by rover_mode, else the wire follower) unless an RC transmitter is
// present with its aux switch in manual (RC has priority); "run stop" (and the STOP frame) leaves
// mode 2 for mode 0: with no RC that is neutral output, and the autopilot does not run. The stop is
// held in RAM: a restart with no RC enters mode 2 again (boot behaviour unchanged).
uint8_t runCommand(bool start, const char* source, String& reply){
  const uint32_t aux = pwm_read_rmt_dur(2);
  const bool rc_manual = aux > 500 && aux < 1450;
  uint8_t r = 0;
  if (start && rc_manual) {
    reply = "run=stopped rc_manual";
    r = 4;
  } else if (start) {
    if (drive_mode_status != 2) {
      drive_mode_status = 2;
      case2_use_cruise = false;  // ramp to creep, as at boot
      case2_throttle_current = PWM_OUT_2_DEFAULT;
      case2_aux_entry_is_manual = rc_manual;
      case2_aux_changed = false;
    }
    reply = "run=running";
  } else {
    drive_mode_status = 0;
    reply = "run=stopped";
  }
  telemetryEvent(rec::EV_RUN, start ? 1 : 0, r, (String(source) + " -> " + reply).c_str());
  usbOut.printf("RUN %s (%s)\n", reply.c_str(), source);
  return r;
}

// 0.6.25 "defget <tlm|set> <offset>": the build's telemetry_def.json or settings_def.json, gzipped in the image
// (tools/gen_def_files_build.py), so that an app reads the definition of a build it does not know from the Rx itself.
// Answer "def <kind> <version> <total> <crc32> <offset> <base64>", at most DEFGET_CHUNK bytes from offset (an ESP-NOW
// ACK holds 246 text bytes); version: the table hash (tlm) or the settings_def.json version (set); total: gzip bytes;
// crc32: of the whole gzip. Offset at or past total: result 2, "def <kind> <version> <total> <crc32> end".
#include "def_files_gz.h"
#include <mbedtls/base64.h>
static const size_t DEFGET_CHUNK = 144;  // 192 base64 characters
uint8_t defgetCommand(const char* a, String& reply){
  const uint8_t* gz; size_t total; const char* version; uint32_t crc; const char* kind;
  if (!strncmp(a, "tlm ", 4)) { kind = "tlm"; gz = DEF_TLM_GZ; total = sizeof(DEF_TLM_GZ); version = DEF_TLM_VERSION; crc = DEF_TLM_CRC32; }
  else if (!strncmp(a, "set ", 4)) { kind = "set"; gz = DEF_SET_GZ; total = sizeof(DEF_SET_GZ); version = DEF_SET_VERSION; crc = DEF_SET_CRC32; }
  else { reply = "defget: <tlm|set> <offset>"; return 2; }
  char* end;
  const unsigned long off = strtoul(a + 4, &end, 10);
  if (end == a + 4 || *end) { reply = "defget: <tlm|set> <offset>"; return 2; }
  char head[64];
  snprintf(head, sizeof(head), "def %s %s %u %08lx ", kind, version, (unsigned)total, (unsigned long)crc);
  if (off >= total) { reply = String(head) + "end"; return 2; }
  const size_t n = min(DEFGET_CHUNK, (size_t)(total - off));
  unsigned char b64[DEFGET_CHUNK / 3 * 4 + 4];
  size_t olen = 0;
  mbedtls_base64_encode(b64, sizeof(b64), &olen, gz + off, n);
  b64[olen] = 0;
  reply = String(head) + off + " " + (const char*)b64;
  return 0;
}

uint8_t configCommand(const char* t, bool over_espnow, String& reply){
  if (!strcmp(t, "run start") || !strcmp(t, "run stop"))
    return runCommand(!strcmp(t, "run start"), over_espnow ? "CMD" : "USB", reply);
  if (!strcmp(t, "get run")) {
    reply = drive_mode_status == 2 ? "run=running" : "run=stopped";
    return 0;
  }
  if (!strcmp(t, "tlm")) {  // 0.5.116: the table hash (telemetry_def.json "version") and the number of table values
    char b[48];
    snprintf(b, sizeof(b), "tlm=%08lx values=%d", (unsigned long)tlm::hash(), tlm::count());
    reply = b;
    return 0;
  }
  if (!strcmp(t, "get telemetry_def")) {  // the telemetry_def.json "version" of this build (the table hash)
    char b[32];
    snprintf(b, sizeof(b), "telemetry_def=%08lx", (unsigned long)tlm::hash());
    reply = b;
    return 0;
  }
  if (!strncmp(t, "defget ", 7)) return defgetCommand(t + 7, reply);  // read only: no event (about 100 chunks a file)
  if (!strncmp(t, "wifi", 4)) {  // Wi-Fi network and password: USB "wifi set" only, never over the radio
    reply = "wifi: USB only";
    telemetryEvent(rec::EV_SETTING, 4, over_espnow ? 1 : 0, "wifi (value not logged) -> refused");
    return 4;
  }
  const uint8_t r = configCommandRun(t, over_espnow, reply);
  if (strncmp(t, "get ", 4)) {  // one event per set or save, applied or refused: the log shows what was tried
    char safe[ESPNOW_CMD_TEXT_MAX + 32];
    wifitelem::logSafeCommand(t, safe, sizeof(safe));  // "set wifi_..." keeps no value: the log never holds a password
    telemetryEvent(rec::EV_SETTING, r, over_espnow ? 1 : 0, (String(safe) + " -> " + reply).c_str());
  }
  return r;
}

// 0.5.104: "set dsp_frame_period_us" answers with the period in effect, after the Rx STM32 answer to CID 18 (up to
// 500 ms; the loop runs on meanwhile). configCommandRun sets period_ack_want; the caller (USB or radio) hands its
// reply address to periodAckDefer and sends nothing; periodAckService sends the answer.
bool period_ack_want = false;
long period_ack_asked = 0;
uint32_t period_ack_answers0 = 0, dsp_period_answers = 0;
struct { bool on, usb, web; uint8_t mac[6]; uint16_t seq; long asked; uint32_t t0, answers0; } period_ack = {};
void espnowSendAck(const uint8_t* mac, uint16_t seq, uint8_t result, const char* text);
void espnowCacheAck(const uint8_t* mac, uint16_t seq, uint8_t result, const String& reply);
void periodAckDefer(bool usb, const uint8_t* mac, uint16_t seq){
  period_ack_want = false;
  period_ack.on = true; period_ack.usb = usb; period_ack.web = false;
  if (mac) memcpy(period_ack.mac, mac, 6);
  period_ack.seq = seq; period_ack.asked = period_ack_asked;
  period_ack.t0 = millis(); period_ack.answers0 = period_ack_answers0;
}
void periodAckService(){
  if (!period_ack.on) return;
  const bool answered = dsp_period_answers != period_ack.answers0;
  if (!answered && millis() - period_ack.t0 < 500) return;
  period_ack.on = false;
  String reply = "dsp_frame_period_us=";
  const double eff = g_dsp_eff[DSP_EFF_PERIOD];
  if (!answered || !isfinite(eff)) reply += String(period_ack.asked) + " (no Rx STM32 answer)";
  else if ((long)eff == period_ack.asked) reply += String((long)eff);
  else reply += String((long)eff) + " (asked " + period_ack.asked + ")";
  if (period_ack.web) webcfg::answer(0, reply);
  else if (period_ack.usb) usbOut.printf("CFG 0 %s\n", reply.c_str());
  else { espnowSendAck(period_ack.mac, period_ack.seq, 0, reply.c_str()); espnowCacheAck(period_ack.mac, period_ack.seq, 0, reply); }
}

// Settings page (src/web_settings.cpp): one page command, in loop(). The page uses the Wi-Fi link, so the rules of a
// radio command apply: a key that would cut the link is refused, "config erase" keeps the link settings and the Wi-Fi
// details. "restart": refused while the vehicle moves; the page module restarts after its answer.
// *deferred: a new update period answers later, after the Rx STM32 answer (periodAckService -> webcfg::answer).
uint8_t webCommand(const char* t, String& reply, bool* deferred){
  *deferred = false;
  if ((update_mode || stmUrlBusy()) && !updateAllowed(t)) { reply = "refused: an update runs"; return 4; }
  if (!strcmp(t, "restart")) {
    if (vehicleStationary()) { reply = "restart=restarting"; return 0; }
    reply = "restart=Stop the vehicle first";
    telemetryEvent(rec::EV_RESTART_REFUSED, 1, 2, "page restart refused: the vehicle moves");
    return 4;
  }
  const uint8_t r = configCommand(t, true, reply);
  if (period_ack_want) {
    periodAckDefer(false, nullptr, 0);
    period_ack.web = true;
    *deferred = true;
  }
  return r;
}

// 0.5.104: after "bin <i> <hz>", read CID 20 index i every 50 ms (up to 1.5 s) until the Rx STM32 answers hz: 2.0.58 moves
// the bin about 10 frames after the write and answers the old frequency to the write itself. The reads keep
// dsp_bin_hz_<i>_eff true for the app.
int bin_poll_idx = -1;
float bin_poll_hz = 0;
uint32_t bin_poll_t0 = 0, bin_poll_last = 0;
void binPollService(){
  if (bin_poll_idx < 0 || stm_link_ver != 2) return;
  const uint32_t now = millis();
  if (fabs(g_dsp_eff[DSP_EFF_BIN0 + bin_poll_idx] - bin_poll_hz) < 0.5 || now - bin_poll_t0 > 1500) { bin_poll_idx = -1; return; }
  if (now - bin_poll_last < 50) return;
  bin_poll_last = now;
  stmLink.push(20, 0, (uint8_t)bin_poll_idx);
}

uint8_t configCommandRun(const char* t, bool over_espnow, String& reply){
  char key[48] = {0}, value[64] = {0};
  String out;
  // "config erase" (settings to defaults; the Wi-Fi details stay) and "config erase wifi" (the Wi-Fi details too).
  // Only while stopped (a flash write stops the CPU). Over a radio link the link settings stay (radio_mode,
  // espnow_enabled, telemetry_enabled, cmd_key), and "wifi" is USB only, as "wifi set" and "wifi clear".
  if (!strcmp(t, "config erase") || !strcmp(t, "config erase wifi")) {
    const bool wifi = !strcmp(t, "config erase wifi");
    const String k = wifi ? "config erase wifi" : "config erase";
    if (drive_mode_status == 2 || !vehicleStationary()) { reply = k + "=Stop the vehicle first"; return 4; }
    if (wifi && over_espnow) { reply = k + "=USB only"; return 4; }
    usbOut.drain(100);  // the USB output empty first: no USB transfer runs into the flash write
    const bool ok = msc_fat_erase(&g_deviceConfig, wifi, over_espnow);
    applyDeviceConfig(g_deviceConfig);
    reply = k + (ok ? "=done: settings at defaults" : "=settings at defaults; config.json write failed");
    reply += wifi ? "; Wi-Fi details erased" : over_espnow ? "; link settings and Wi-Fi details kept" : "; Wi-Fi details kept";
    return ok ? 0 : 4;
  }
  if (!strcmp(t, "save")) {
    // Run model (0.5.95): a save only while stopped. A flash write halts everything else for its time.
    if (drive_mode_status == 2 || !vehicleStationary()) { reply = "save=Stop the vehicle first"; return 4; }  // a flash write stops the CPU
    usbOut.drain(100);  // the USB output empty first: no USB transfer runs into the flash write
    // NVS and config.json (design choice 2026-10-10: a save is a save of the file). config.json overrides
    // NVS at boot, so an NVS-only save of a key that the file holds was lost at the next boot.
    if (msc_fat_save_all(&g_deviceConfig)) { reply = "save=json"; return 0; }
    reply = "save=nvs only: config.json write failed";
    return 4;
  }
  // "sweep <index> <value>": CID 43 to the Rx STM32 as it is (index 0 sweep bins 0-8 or 255 auto; 1 range start Hz;
  // 2 range stop Hz; 0 = the span edge). The PC chooses; the Rx STM32 answers (OK, OK_CLAMPED). A change of the bin
  // count restarts the DSP set-up: not while the vehicle moves. The range is safe while driving. Not saved: the next
  // DSP set sends dsp_sweep_bins again.
  {
    int si = -1;
    long sv = 0;
    if (sscanf(t, "sweep %d %ld", &si, &sv) == 2) {
      if (si < 0 || si > 2) { reply = "sweep"; return 2; }
      if (si == 0 && !vehicleStationary()) { reply = "sweep=Stop the vehicle first"; return 4; }
      if (stm_link_ver != 2) { reply = "sweep=no Rx STM32 v2 link"; return 4; }
      if (si == 0) sweep_sent_bins = (int32_t)sv;
      stmLink.pushInt(43 | 0x80000000, (int32_t)sv, (uint8_t)si);
      stm_ans_print_until = millis() + 2000;  // USB shows the answer (STM ans ...)
      reply = String("sweep ") + si + "=" + sv + " sent";
      return 0;
    }
  }
  // "bin <index> <hz>" (0.5.103): CID 20 to the Rx STM32 as it is: bin index 0-7 to hz (0 = back to the sweep; the Rx
  // STM32 takes 46 800-62 800 Hz and answers the value in effect). The spectrum view puts a spare bin on a cursor
  // frequency. Safe while driving; the wire bin of each sensor is refused. Not saved: no Rx STM32 save, no settings
  // change. 0.6.29: it stays through each DSP set until a dsp_bin_hz change or an ESP32 restart (bin_ram_hz).
  {
    int bi = -1;
    float bhz = 0;
    if (sscanf(t, "bin %d %f", &bi, &bhz) == 2) {
      // The answer texts go to the app as they are. The ACK goes before the Rx STM32 answer; the value
      // in effect is "get dsp_bin_hz_<index>_eff" (empty until the Rx STM32 answers).
      if (bi < 0 || bi >= (int)WFRX_BINS) { reply = "bin=Bin index out of range: 0 to 7"; return 2; }
      const String k = String("bin ") + bi;
      if (!isfinite(bhz) || (bhz != 0 && (bhz < 46800 || bhz > 62800))) { reply = k + "=Frequency out of range: 46800 to 62800 Hz"; return 2; }
      if (stm_link_ver != 2) { reply = k + "=No Rx STM32 link"; return 4; }
      for (int sn = 0; sn < 2; sn++)
        if (stmDec.wireBin(sn) == bi) { reply = k + "=Wire bin of sensor " + (sn + 1) + ": not free"; return 4; }
      stmLink.pushFloat(20 | 0x80000000, bhz, (uint8_t)bi);
      bin_ram_hz[bi] = bhz;
      bin_ram_mask |= (uint8_t)(1u << bi);
      stm_ans_print_until = millis() + 2000;  // USB shows the answer (STM ans ...)
      if (bhz != 0) { bin_poll_idx = bi; bin_poll_hz = bhz; bin_poll_t0 = millis(); bin_poll_last = 0; }  // 0.5.104
      reply = bhz == 0 ? k + "=sweep" : k + "=" + String((long)lroundf(bhz)) + " Hz";
      return 0;
    }
  }
  if (sscanf(t, "get %47s", key) == 1) {
    if (!msc_fat_get_key(&g_deviceConfig, key, &out)) { reply = key; return 1; }
    reply = String(key) + "=" + out;
    return 0;
  }
  if (sscanf(t, "set %47s %63s", key, value) == 2) {
    // A text setting (vehicle_name, drive_output; 0.5.69): the rest of the line, spaces included, with no
    // quotes needed; letters, digits, space, '_', '-' and '.' only. It goes to the parser as one JSON string.
    String cur;
    if (msc_fat_get_key(&g_deviceConfig, key, &cur) && cur.startsWith("\"")) {
      const char* v = t + 4;
      while (*v == ' ') v++;
      v += strlen(key);
      while (*v == ' ') v++;
      String raw = v;
      raw.trim();
      if (raw.length() >= 2 && raw.startsWith("\"") && raw.endsWith("\"")) raw = raw.substring(1, raw.length() - 1);
      if (raw.length() == 0 || raw.length() > 32) { reply = key; return 2; }
      for (size_t i = 0; i < raw.length(); i++) {
        const char ch = raw[i];
        if (!isalnum((unsigned char)ch) && ch != ' ' && ch != '_' && ch != '-' && ch != '.') { reply = key; return 2; }
      }
      snprintf(value, sizeof(value), "\"%s\"", raw.c_str());
    } else if (strpbrk(value, ",{}[]:\"")) { reply = key; return 2; }  // one literal only: no JSON injection of other keys
    if (over_espnow && !strcmp(key, "cmd_key")) { reply = "cmd_key not over ESP-NOW"; return 4; }
    if (stationaryKey(key) && !vehicleStationary()) {
      msc_fat_get_key(&g_deviceConfig, key, &out);
      reply = String(key) + "=" + out + " (Stop the vehicle first)";
      return 4;
    }
    const bool radio_key = !strcmp(key, "espnow_enabled") || !strcmp(key, "telemetry_enabled");
    if (over_espnow && radio_key && !strcmp(value, "false")) {
      reply = String(key) + "=true";  // would cut the link that carries the command
      return 4;
    }
    // Over a radio link (ESP-NOW, or Wi-Fi from 0.5.86), radio_mode keeps that link: another value would cut the
    // link that carries the command (after the next start).
    const char* link_mode = radio_wifi ? "wifi" : "espnow";
    if (over_espnow && !strcmp(key, "radio_mode") && strcmp(value, link_mode) && strcmp(value, (String("\"") + link_mode + "\"").c_str())) {
      reply = String(key) + "=\"" + link_mode + "\"";
      return 4;
    }
    {  // 0.5.103: the update period must suit the link mode (a change of the period, radio_mode or the radio keys)
      static DeviceConfig next;
      next = g_deviceConfig;
      const bool period_key = !strcmp(key, "dsp_frame_period_us");
      char* num_end = nullptr;
      const long asked_num = strtol(value, &num_end, 10);
      const bool period_int = period_key && num_end != value && *num_end == 0;
      const int tr = msc_fat_set_key(&next, key, value);  // a trial on a copy
      if (tr == 0 || (tr == 2 && period_int)) {
        if (tr == 2) next = g_deviceConfig;  // the period out of the table range: the floor of the present link mode
        const int m = linkModeStrict(next);
        // the value as asked: under the floor the answer stays "Update period too short" (the table range would refuse it too)
        const long asked = period_key ? asked_num : next.dsp_frame_period_us;
        if (asked < periodFloorUs(m, next)) {
          reply = String(key) + "=Update period too short: " + String(periodFloorUs(m, next) / 1000.0, 2) + " ms or longer";
          return 2;
        }
      }
    }
    const int old_period = g_deviceConfig.dsp_frame_period_us;
    String why;  // out of range: "Range <min> to <max>" (refused, no change)
    const int r = msc_fat_set_key(&g_deviceConfig, key, value, &why);
    if (r != 0) { reply = why.length() ? String(key) + "=" + why : String(key); return (uint8_t)r; }
    // 0.5.104: a new update period goes to the Rx STM32 (dspApply); the answer waits for its CID 18 answer (periodAckService).
    // 0.6.1: also an unchanged setting when the period in effect differs by more than 1 % (a direct CID 18 write, "stm cmd
    // 18", or an Rx STM32 restart left another period): the asked period goes again (0.6.0 answered "(asked ...)" only).
    bool period_resend = false;
    if (!strcmp(key, "dsp_frame_period_us") && stm_link_ver == 2) {
      const double eff = g_dsp_eff[DSP_EFF_PERIOD];
      const long want = g_deviceConfig.dsp_frame_period_us;
      period_resend = g_deviceConfig.dsp_frame_period_us == old_period && isfinite(eff) && fabs(eff - want) > want / 100.0;
      if (g_deviceConfig.dsp_frame_period_us != old_period || period_resend) {
        period_ack_want = true;
        period_ack_asked = g_deviceConfig.dsp_frame_period_us;
        period_ack_answers0 = dsp_period_answers;
      }
    }
    applyDeviceConfig(g_deviceConfig);
    if (period_resend) { dsp_apply_reason = "period in effect differs"; dspApply(); }
    msc_fat_get_key(&g_deviceConfig, key, &out);
    reply = String(key) + "=" + out;
    if (!strcmp(key, "dsp_frame_period_us") && !period_ack_want) {  // no change: the period in effect now
      const double eff = g_dsp_eff[DSP_EFF_PERIOD];
      if (isfinite(eff) && (long)eff != g_deviceConfig.dsp_frame_period_us)
        reply = String(key) + "=" + String((long)eff) + " (asked " + g_deviceConfig.dsp_frame_period_us + ")";
    }
    if (!strcmp(key, "radio_mode") && !over_espnow)  // the reply stays "key=value" for the PC; the note is a line of its own
      usbOut.printf("RADIO radio_mode=%s in effect from the next start (save, then restart); now %s\n", out.c_str(),
                    radio_wifi ? "wifi" : "espnow");
    return 0;
  }
  reply = "unknown command";
  return 2;
}

