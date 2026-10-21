// Restarts and updates: ESP32 app updates, the Rx STM32 app update (bridge, restart command, download
// and feed), the Rx STM32 calibration copy.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

bool update_mode = false;        // from "ota begin": loop() serves only USB and the update (see loop())
uint32_t update_last_ms = 0;     // millis() of the last "ota" line
// Restarts that the PC sees (bench tests). With USB persist off
// (arduino-esp32 2.0.17), usb_persist_restart() resets the USB core but leaves the D+ pull-up on:
// the PC never sees the board leave the bus, and never addresses the new USB device after the
// restart, so the board looks silent. Detach first, and give the PC time to see it.
// Not yet verified on a board.
void usbDetachForRestart(){
  msc_fat_eject();  // the PC stops its drive reads first (a PC that read the drive at the detach lost USB ports)
  usbOut.drain(200);
  delay(500);
  tud_disconnect();  // D+ pull-up off: the PC sees a removal
  delay(300);
}
void restartIntoRom(){  // ROM download mode, for esptool --before no_reset --after hard_reset
  // 0.6.15: the RTC watchdog (8 s, rtcWdtStart) survives the restart into the ROM loader, which never feeds it: the
  // ROM loader reset every 8 s and a flash write from the update page stopped part way. Off first.
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  rtc_wdt_protect_on();
  usbDetachForRestart();
  usb_persist_restart(RESTART_BOOTLOADER);
  ESP.restart();  // only if the shutdown handler could not be registered
}
void restartApp(){
  usbDetachForRestart();
  ESP.restart();
}

// usb_ota "may start": only a vehicle that stands still takes an update, and it then enters update
// mode: outputs to their centre pulses, then loop() serves only USB and the update.
bool vehicleStationary();
bool otaMayStart(){
  if (!vehicleStationary()) return false;
  servoIdle();
  if (!update_mode) {
    usbOut.println("UPDATE mode on: USB and the update only; loop watchdog paused");
    disableLoopWDT();  // the image check at "ota end" can run past 5 s (bench test)
  }
  update_mode = true;
  update_last_ms = millis();
  return true;
}

// usb_ota "ota url": Wi-Fi mode with the station connected (the driver state: the link service stops in update mode).
bool otaNetReady(){
  return radio_wifi && WiFi.status() == WL_CONNECTED;
}

// Bench: a raw link to the STM32's ROM loader (AN3155: 115200 baud, 8 data bits, even parity) for
// the first load of a blank STM32. A blank STM32G0 starts its ROM loader by itself (empty check);
// the Rx board has no BOOT0 or reset line. "stm rom" opens the link (only while the vehicle stands
// still), "stm tx <hex>" sends bytes, the board prints "STM rx <hex>" for each read, and "stm off"
// returns to the 2 Mbaud packet link. A PC tool runs the protocol.
bool stm_raw = false;
void stmRestartCommand(const char* arg);
void stmRawStreamCommand(const char* arg);
// STM32 app update through the boot checker (IAP), bench and field: "stm iaphold" sets a one-shot
// flag in NVS. At the next power-on the handshake sends HELLO only (no BOOT), so the STM32 stays in
// its boot checker, and the board becomes a transparent bridge between USB and the STM32 UART
// (2 Mbaud): the PC runs the IAP update (protocol in TrailTracer-Rx-STM32
// iap_proto.h). No USB text in bridge mode. The bridge ends STM_BRIDGE_IDLE_MS after the last USB
// byte (STM_BRIDGE_WAIT_MS if none came); then HELLO/BOOT if the STM32 is still silent. The board
// then continues as usual, with no ESP32 restart.
#define STM_BRIDGE_WAIT_MS 300000
#define STM_BRIDGE_IDLE_MS 10000
#define STM_BRIDGE_HELLO_TRIES 3
bool stm_bridge = false;
uint32_t stm_bridge_start_ms = 0, stm_bridge_rx_ms = 0, stm_bridge_up = 0, stm_bridge_down = 0;
bool stmIapHoldTake(){  // read and clear the one-shot flag
  Preferences p;
  if (!p.begin("wfrx_bench", false)) return false;
  const bool hold = p.getBool("iaphold", false);
  if (hold) p.remove("iaphold");
  p.end();
  return hold;
}
void stmIapHoldSet(){
  msc_fat_eject();  // a power-off follows: the PC stops its drive reads now
  Preferences p;
  const bool ok = p.begin("wfrx_bench", false) && p.putBool("iaphold", true);
  p.end();
  usbOut.println(ok ? "STM iaphold set: at the next power-on the STM32 stays in its boot checker and USB bridges to it"
                    : "STM iaphold: NVS write failed");
}

// STM32 calibration copy (design choice, 2026-10-11): an IAP update erases the STM32 NVM (it lies in
// the app region), and the new app starts with its firmware defaults. The ESP32 keeps the per-unit
// calibration in NVS ("wfrx_cal"): bin gain (CID 26, 16), radius (27, 2), angle (28, 2), PGA (37, 12).
// "stm iaphold" reads all 32 first and stores them; if a read fails, the stored copy stays. Then the
// restore flag is set. At the next v2 start the board writes the copy, saves it (CID 3), reads all 32
// back and clears the flag when they are equal. One command at a time, when the link is not busy.
#define STM_CAL_N 32
#define STM_CAL_PHASE_MS 8000
// enum CAL_IDLE ... CAL_VERIFY: platform_internal.h (rx_link.inc uses it too)
uint8_t stm_cal_phase = CAL_IDLE;
bool stm_cal_then_hold = false;     // the read was for "stm iaphold": set the flag at its end
bool stm_cal_then_restart = false;  // the read was for "stm restart go": the restart command follows at its end
void stmRestartBegin();
uint32_t stm_cal_val[STM_CAL_N];    // the stored copy (raw float bits)
bool stm_cal_valid = false;
uint32_t stm_cal_new[STM_CAL_N];    // this read, or the verify read
uint8_t stm_cal_i = 0, stm_cal_ok = 0;
bool stm_cal_sent = false, stm_cal_got = false;
uint32_t stm_cal_phase_ms = 0;
void stmCalItem(int k, uint8_t* cid, uint8_t* idx){
  if (k < 16) { *cid = 26; *idx = (uint8_t)k; }
  else if (k < 18) { *cid = 27; *idx = (uint8_t)(k - 16); }
  else if (k < 20) { *cid = 28; *idx = (uint8_t)(k - 18); }
  else { *cid = 37; *idx = (uint8_t)(k - 20); }
}
void stmCalLoad(){
  Preferences p;
  stm_cal_valid = p.begin("wfrx_cal", true) && p.getBytes("v", stm_cal_val, sizeof(stm_cal_val)) == sizeof(stm_cal_val);
  p.end();
}
bool stmCalPending(){
  Preferences p;
  const bool r = p.begin("wfrx_cal", true) && p.getBool("restore", false);
  p.end();
  return r;
}
void stmCalSetPending(bool on){
  Preferences p;
  if (p.begin("wfrx_cal", false)) p.putBool("restore", on);
  p.end();
}
void stmCalStart(uint8_t phase){
  stm_cal_phase = phase;
  stm_cal_i = 0;
  stm_cal_ok = 0;
  stm_cal_sent = false;
  stm_cal_phase_ms = millis();
}
// One v2 answer (before stmLink.onAnswerV2).
void stmCalOnAnswer(const stmv2::Answer& a){
  if (stm_cal_phase == CAL_IDLE || !stm_cal_sent || !a.present || (a.status != RSP_OK && a.status != RSP_OK_CLAMPED)) return;
  if (stm_cal_phase == CAL_SAVE) { if (a.cid == (3 | ESP_CID_WRITE)) stm_cal_got = true; return; }
  uint8_t cid, idx;
  stmCalItem(stm_cal_i, &cid, &idx);
  const uint8_t want = stm_cal_phase == CAL_WRITE ? (uint8_t)(cid | ESP_CID_WRITE) : cid;
  if (a.cid != want || a.index != idx) return;
  stm_cal_new[stm_cal_i] = a.value;
  stm_cal_got = true;
}
void stmCalEnd(){
  const uint8_t phase = stm_cal_phase;
  stm_cal_phase = CAL_IDLE;
  if (phase == CAL_READ) {
    if (stm_cal_ok == STM_CAL_N) {
      memcpy(stm_cal_val, stm_cal_new, sizeof(stm_cal_val));
      Preferences p;
      stm_cal_valid = p.begin("wfrx_cal", false) && p.putBytes("v", stm_cal_val, sizeof(stm_cal_val)) == sizeof(stm_cal_val);
      p.end();
      usbOut.printf("STM cal read: %u of %u, stored %s\n", (unsigned)stm_cal_ok, STM_CAL_N, stm_cal_valid ? "ok" : "FAILED");
    } else {
      usbOut.printf("STM cal read failed: %u of %u answered; the stored copy stays (%s)\n", (unsigned)stm_cal_ok, STM_CAL_N, stm_cal_valid ? "valid" : "none");
    }
    if (stm_cal_then_hold) {
      stm_cal_then_hold = false;
      if (stm_cal_valid) stmCalSetPending(true);
      stmIapHoldSet();
    }
    if (stm_cal_then_restart) {
      stm_cal_then_restart = false;
      stmRestartBegin();
    }
  } else if (phase == CAL_VERIFY) {
    int eq = 0;
    for (int k = 0; k < STM_CAL_N; k++) eq += stm_cal_new[k] == stm_cal_val[k];
    if (eq == STM_CAL_N) stmCalSetPending(false);
    usbOut.printf("STM cal restore: written %u, saved, %d of %u read back equal%s\n", (unsigned)stm_cal_ok, eq, STM_CAL_N,
                  eq == STM_CAL_N ? "" : "; FAILED, the restore runs again at the next start");
    telemetryEvent(rec::EV_STM32_CMD, stmset::K_CAL_RESTORE, eq, eq == STM_CAL_N ? "STM32 calibration restored after update" : "STM32 calibration restore failed");
  } else if (phase == CAL_SAVE) {
    usbOut.printf("STM cal restore: CID 3 save %s\n", stm_cal_got ? "ok" : "no answer");
    stmCalStart(CAL_VERIFY);
  }
}
// Once a v2 frame, after stmLink.onAnswerV2: the next command when the last one is done.
void stmCalService(uint32_t now){
  if (stm_cal_phase == CAL_IDLE) return;
  if (stm_cal_sent) {
    if (stmLink.busy()) {
      if (now - stm_cal_phase_ms < STM_CAL_PHASE_MS) return;
    }
    stm_cal_sent = false;
    if (stm_cal_got) stm_cal_ok++;
    if (stm_cal_phase == CAL_SAVE) { stmCalEnd(); return; }
    if (++stm_cal_i >= STM_CAL_N || now - stm_cal_phase_ms >= STM_CAL_PHASE_MS) {
      if (stm_cal_phase == CAL_WRITE) { stmCalStart(CAL_SAVE); }
      else { stmCalEnd(); return; }
    }
  }
  if (stmLink.busy()) return;
  stm_cal_got = false;
  if (stm_cal_phase == CAL_SAVE) {
    stm_v2_save_ok = true;
    stmLink.pushInt(3 | stmset::WRITE, 1);
  } else {
    uint8_t cid, idx;
    stmCalItem(stm_cal_i, &cid, &idx);
    if (stm_cal_phase == CAL_WRITE) stmLink.push(cid | stmset::WRITE, stm_cal_val[stm_cal_i], idx);
    else stmLink.push(cid, 0, idx);
  }
  stm_cal_sent = true;
}
void stmBridgeService(){
  uint8_t b[256];
  int n = 0;
  while (n < (int)sizeof(b) && Serial.available()) b[n++] = (uint8_t)Serial.read();
  if (n) { Serial1.write(b, n); stm_bridge_up += n; stm_bridge_rx_ms = millis(); }
  int room = Serial.availableForWrite();  // never block: what the host cannot take waits in Serial1's buffer
  n = 0;
  while (n < room && n < (int)sizeof(b) && Serial1.available()) b[n++] = (uint8_t)Serial1.read();
  if (n) { Serial.write(b, n); stm_bridge_down += n; }
  const uint32_t now = millis();
  const bool idle = stm_bridge_rx_ms ? now - stm_bridge_rx_ms >= STM_BRIDGE_IDLE_MS
                                     : now - stm_bridge_start_ms >= STM_BRIDGE_WAIT_MS;
  // nDATA_READY edges do not show a running application: the boot checker makes them too (bench
  // test). So the bridge ends only by time; send nothing to the board until "STM bridge off".
  if (!idle) return;
  stm_bridge = false;
  stm_link_ver = 0;  // the STM32 app may be new: learn the link version again
  stm_v2_cfg_sent = false;
  stmDec.reset();
  data_ready = 0;
  delay(50);
  usbOut.printf("STM bridge off: up=%lu down=%lu B\n", (unsigned long)stm_bridge_up, (unsigned long)stm_bridge_down);
  if (data_ready == 0 && Serial1.available() == 0) {  // STM32 still in its boot checker
    char t[72];
    uint8_t plen = 0;
    const int r = stm32_iap_boot(STM_BRIDGE_HELLO_TRIES, t, sizeof(t), &plen);
    usbOut.printf("STM bridge: HELLO/BOOT result %d, %s\n", r, t);
    telemetryEvent(rec::EV_STM32, 1, plen, t);
  }
  serial1Drain();
  data_ready = 0;
}
// Rx STM32 restart into its boot checker, with no power cycle (command 41, Rx STM32 app 2.0.39 and later;
// TrailTracer-Rx-STM32 docs/RX_STM32_RESTART_COMMAND.md). The PC puts the first 1092 B of the update file
// set on the board (52 B commit header, then the first 1040 B of the encrypted image: chunk 0 and its tag)
// with "stm restart add <hex>" lines, then sends "stm restart go". The board copies the calibration (as
// "stm iaphold"), sets idle mode, sends the bytes in 273 commands, and asks the Rx STM32 to check them.
// The Rx STM32 runs the boot checker's own checks; a pass restarts it into its boot checker. The board
// then sends nothing for 60 ms (the boot checker drops a partial frame only after a 50 ms gap), reads the
// boot checker's status frame, sends HELLO and bridges USB to it as after "stm iaphold" (stmBridgeService).
// The PC runs the normal update through the bridge. Each step has a progress time-out; a failure prints
// "STM restart failed" and sets run mode again.
#define STM_RS_LEN 1092
#define STM_RS_CID 41
#define STM_RS_STEP_MS 2000  // one command with no answer this long: failed (the link retries 3 times in 35 ms)
// enum RS_IDLE ... RS_RESTART: platform_internal.h (rx_link.inc and setup_loop.inc use it too)
uint8_t stm_rs_phase = RS_IDLE;
uint8_t stm_rs_buf[STM_RS_LEN];
uint16_t stm_rs_n = 0;       // bytes held
uint16_t stm_rs_i = 0;       // data words answered
bool stm_rs_sent = false, stm_rs_got = false;
uint8_t stm_rs_status = 0;
uint32_t stm_rs_value = 0, stm_rs_ms = 0, stm_rs_t0 = 0;

void stmUrlOn41Fail(const char* why, long a, long b);
bool stmUrlOn41Pass();
void stmRestartFail(const char* why, long a, long b){
  usbOut.printf("STM restart failed: %s (%ld, %ld) after %lu ms\n", why, a, b, (unsigned long)(millis() - stm_rs_t0));
  telemetryEvent(rec::EV_STM32, 0, STM_RS_CID, "Rx STM32 restart command failed");
  stm_rs_phase = RS_IDLE;
  stm_rs_sent = false;
  if (stm_v2_autorun) { stmLink.pushInt(STM_RUN_CMD | 0x80000000, 1); stm_v2_run_ms = millis(); }
  stmUrlOn41Fail(why, a, b);
}

extern bool uart_force_old;
extern bool rtc_wdt_test;
void stmRestartEnter();
// 0.6.0: command 41 with no readable Rx STM32 frame (a frame format this ESP32 does not read, or an app whose frames
// fail): the sequence goes one 10 B command per frame edge with no answer awaited, from the 1092 B held in stm_rs_buf.
// A running app takes 10 B command frames in every version. The boot checker's HELLO afterwards tells whether the
// check passed. Blocks loop() for about 280 update periods (2.8 s at 10 ms). false: no frame edges (no app runs).
bool stmRestartBlind(){
  const bool force_old = uart_force_old;
  uart_force_old = true;  // the Rx UART frame interrupt sends nothing meanwhile
  uint8_t seq = (uint8_t)esp_random();
  bool ok = true;
  auto send = [&](uint8_t cid8, uint8_t idx, uint32_t v) {
    if (!ok) return;
    const uint32_t e0 = stm_edge_us, t0 = millis();
    while (stm_edge_us == e0) {  // the next frame edge: the window opens
      if (millis() - t0 > 200) { ok = false; return; }
      delayMicroseconds(50);
    }
    delayMicroseconds(100);
    uint8_t f[ESP_FRAME_LEN];
    stmv2::encodeCmd(seq++, cid8, idx, v, f);
    Serial1.write(f, ESP_FRAME_LEN);
    serial1Drain();  // the frames this ESP32 cannot read: nothing to keep
    feedLoopWDT();
    if (!rtc_wdt_test) rtc_wdt_feed();
  };
  send(1 | ESP_CID_WRITE, 0, 0);  // idle mode: command 41 runs only there
  send(1 | ESP_CID_WRITE, 0, 0);
  send(STM_RS_CID | ESP_CID_WRITE, 0, 0);  // start
  for (unsigned i = 0; i < STM_RS_LEN / 4; i++) {
    const uint8_t* q = stm_rs_buf + 4 * i;
    send(STM_RS_CID | ESP_CID_WRITE, 1, (uint32_t)q[0] | ((uint32_t)q[1] << 8) | ((uint32_t)q[2] << 16) | ((uint32_t)q[3] << 24));
  }
  send(STM_RS_CID | ESP_CID_WRITE, 2, 0);  // check: on a pass the app restarts into its boot checker
  uart_force_old = force_old;
  return ok;
}

// The command 41 sequence with the 1092 B held: calibration copy first. nullptr: started; else the refusal.
const char* stmRestartGo(){
  if (stm_rs_phase != RS_IDLE) return "in progress";
  if (stm_link_ver != 2) return "no v2 link to the Rx STM32";
  if (!vehicleStationary()) return "the vehicle moves";
  if (stm_cal_phase != CAL_IDLE) return "calibration copy busy";
  stm_rs_t0 = millis();
  stm_rs_phase = RS_CAL;
  stm_cal_then_restart = true;
  stmCalStart(CAL_READ);  // the calibration copy first: an update erases the Rx STM32 settings
  return nullptr;
}

void stmRestartCommand(const char* arg){
  while (*arg == ' ') arg++;
  if (!strcmp(arg, "clear")) {
    stm_rs_n = 0;
    usbOut.printf("STM restart held 0 B\n");
  } else if (!strncmp(arg, "add ", 4)) {
    const char* h = arg + 4;
    while (h[0] && h[1] && stm_rs_n < STM_RS_LEN) {
      unsigned v;
      if (sscanf(h, "%2x", &v) != 1) break;
      stm_rs_buf[stm_rs_n++] = (uint8_t)v;
      h += 2;
    }
    usbOut.printf("STM restart held %u B%s\n", (unsigned)stm_rs_n, h[0] ? ", rest refused (full or not hex)" : "");
  } else if (!strcmp(arg, "force")) {  // 0.6.0: command 41 with no frame link (a silent or wrong-frame app)
    if (stm_rs_phase != RS_IDLE) { usbOut.println("STM restart refused: in progress"); return; }
    if (stm_rs_n != STM_RS_LEN) { usbOut.printf("STM restart refused: %u B held, %u needed\n", (unsigned)stm_rs_n, STM_RS_LEN); return; }
    if (!vehicleStationary()) { usbOut.println("STM restart refused: the vehicle moves"); return; }
    stm_rs_t0 = millis();
    usbOut.println("STM restart force: idle mode and command 41, one command per frame edge, no answer awaited");
    if (!stmRestartBlind()) { usbOut.println("STM restart force failed: no frame edges (no Rx STM32 app runs; a boot checker waits: stm url probes it)"); return; }
    stmRestartEnter();  // the boot checker's HELLO, then the USB bridge
  } else if (!strcmp(arg, "go")) {
    if (stm_rs_phase != RS_IDLE) { usbOut.println("STM restart refused: in progress"); return; }
    if (stm_rs_n != STM_RS_LEN) { usbOut.printf("STM restart refused: %u B held, %u needed\n", (unsigned)stm_rs_n, STM_RS_LEN); return; }
    if (const char* e = stmRestartGo()) { usbOut.printf("STM restart refused: %s\n", e); return; }
    usbOut.println("STM restart: calibration copy, then idle mode and command 41");
  } else {
    usbOut.println("STM usage: stm restart clear | stm restart add <hex> | stm restart go | stm restart force");
  }
}

void stmRestartBegin(){  // from stmCalEnd(): the calibration read is done (passed or not; the stored copy stays)
  if (stm_rs_phase != RS_CAL) return;
  stm_rs_phase = RS_STOP;
  stm_rs_sent = false;
  stm_rs_ms = millis();
}

// One v2 answer (before stmLink.onAnswerV2). A pass of the check sets RS_RESTART at once, so no command
// goes out in this frame's window.
void stmRestartOnAnswer(const stmv2::Answer& a){
  if (stm_rs_phase < RS_START || stm_rs_phase > RS_CHECK || !stm_rs_sent || !a.present) return;
  const uint8_t idx = stm_rs_phase == RS_START ? 0 : stm_rs_phase == RS_DATA ? 1 : 2;
  if (a.cid != (STM_RS_CID | ESP_CID_WRITE) || a.index != idx) return;
  stm_rs_got = true;
  stm_rs_status = a.status;
  stm_rs_value = a.value;
  if (stm_rs_phase == RS_CHECK && a.status == RSP_OK && a.value == 1) stm_rs_phase = RS_RESTART;
}

// Once a v2 frame, after stmLink.onAnswerV2: the next command when the last one is done.
void stmRestartService(uint32_t now){
  if (stm_rs_phase < RS_STOP || stm_rs_phase > RS_CHECK) return;
  if (stm_rs_sent) {
    if (stmLink.busy()) {
      if (now - stm_rs_ms >= STM_RS_STEP_MS) stmRestartFail("no progress", stm_rs_phase, stm_rs_i);
      return;
    }
    stm_rs_sent = false;
    if (stm_rs_phase == RS_STOP) {
      stm_rs_phase = RS_START;
    } else if (!stm_rs_got) {
      stmRestartFail("no answer", stm_rs_phase, stm_rs_i);
      return;
    } else if (stm_rs_status != RSP_OK && stm_rs_status != RSP_OK_CLAMPED) {
      // check refused: the value is the reason (1 byte count, 2 header CRC, 3 magic, 4 product ID,
      // 5 chunk size, 6 image size, 7 chunk count, 8 tag); -1: the Rx STM32 is in run mode
      stmRestartFail(stm_rs_phase == RS_CHECK ? "check refused, status and reason" : "command refused, status and value",
                     stm_rs_status, (long)(int32_t)stm_rs_value);
      return;
    } else if (stm_rs_phase == RS_START) {
      stm_rs_phase = RS_DATA;
      stm_rs_i = 0;
    } else if (stm_rs_phase == RS_DATA) {
      if (stm_rs_value != 4u * (stm_rs_i + 1u)) { stmRestartFail("bytes held", (long)stm_rs_value, 4L * (stm_rs_i + 1)); return; }
      if (++stm_rs_i == STM_RS_LEN / 4) stm_rs_phase = RS_CHECK;
    } else {  // RS_CHECK with an OK status, but not the value 1
      stmRestartFail("check value", (long)(int32_t)stm_rs_value, 1);
      return;
    }
  }
  if (stmLink.busy()) return;
  stm_rs_got = false;
  if (stm_rs_phase == RS_STOP) {
    stmLink.pushInt(STM_RUN_CMD | 0x80000000, 0);  // idle mode: command 41 runs only there
  } else if (stm_rs_phase == RS_START) {
    stmLink.push(STM_RS_CID | stmset::WRITE, 0, 0);
  } else if (stm_rs_phase == RS_DATA) {
    const uint8_t* q = stm_rs_buf + 4 * stm_rs_i;
    stmLink.push(STM_RS_CID | stmset::WRITE, (uint32_t)q[0] | ((uint32_t)q[1] << 8) | ((uint32_t)q[2] << 16) | ((uint32_t)q[3] << 24), 1);
  } else {
    stmLink.push(STM_RS_CID | stmset::WRITE, 0, 2);
  }
  stm_rs_sent = true;
  stm_rs_ms = now;
}

// From loop() after the check passed: the Rx STM32 restarts into its boot checker. Then the USB bridge.
void stmRestartEnter(){
  if (stmUrlOn41Pass()) return;  // "stm url": the ESP32 feeds the boot checker itself
  stm_rs_phase = RS_IDLE;
  stm_rs_n = 0;
  if (stm_cal_valid) stmCalSetPending(true);  // the next v2 start writes the copy back (an update erases it)
  delay(60);  // nothing to the Rx STM32 for at least 50 ms
  uint8_t st[16];
  size_t k = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 500 && k < 7) {  // the boot checker's 7 B status frame: about 0.1 s plus the app CRC time
    if (Serial1.available()) st[k++] = (uint8_t)Serial1.read();
    else delay(1);
  }
  char hx[3 * sizeof(st) + 1];
  hx[0] = 0;
  for (size_t j = 0; j < k; j++) snprintf(hx + 3 * j, 4, " %02x", st[j]);
  uint8_t frame[6], payload[IAP_MAX_FRAME], plen = 0;
  uint16_t cid = 0;
  bool ok = false;
  for (int i = 0; i < IAP_HELLO_RETRIES && !ok; i++) {
    serial1Drain();
    iap_encode_frame(IAP_CID_HELLO, frame);
    Serial1.write(frame, 6);
    ok = iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen) && cid == IAP_CID_HELLO;
  }
  usbOut.printf("STM restart passed in %lu ms; status frame %u B:%s\n", (unsigned long)(millis() - stm_rs_t0), (unsigned)k, hx);
  usbOut.printf("STM bridge on: Rx STM32 in its boot checker by command 41 (HELLO %s); USB is now the STM32 UART, no text until it ends\n",
                ok ? "answered" : "not answered");
  telemetryEvent(rec::EV_STM32, 1, STM_RS_CID, "Rx STM32 restart command passed: USB bridge to the boot checker");
  usbOut.drain(500);
  serial1Drain();
  data_ready = 0;  // the boot checker leaves nDATA_READY floating: its edges are not an application
  stm_bridge_rx_ms = 0;
  stm_bridge_up = stm_bridge_down = 0;
  stm_bridge = true;
  stm_bridge_start_ms = millis();
}

// Update commands from the telemetry app (0.5.114) -------------------------------------------------------------------
// "ota url", "ota status", "ota abort", "stm url", "stm status" and "get rx_stm32_version" work on USB and over the
// radio command link (CMD frame, Wi-Fi or ESP-NOW). The radio ACK text is the line that USB prints.
//
// "stm url <http://a.b.c.d[:port]/path> <size> <sha256>": an Rx STM32 update from one file on an HTTP server: the 52 B
// commit header, then the encrypted image (chunk count x 1040 B), as tools/signing/encrypt_image.py writes them.
//   1 Download into the "tlog" partition (usb_ota::fetch_start, in update mode), SHA-256 of the bytes in flash, and
//     the header's chunk count against the size.
//   2 With Rx STM32 frames: the command 41 sequence with the first 1092 B of the file, read from flash ("stm restart").
//     With no frames and a boot checker that answers HELLO (no app): straight to step 3.
//   3 The ESP32 feeds the boot checker from flash: boot-status frame (or HELLO), BEGIN_UPDATE, CHUNK 0 to N-1,
//     FINALIZE, BOOT, one frame at a time, each after the reply to the one before. loop() runs only the feed, USB and the
//     radio commands (no drive, telemetry or log service): a gap of 50 ms inside a frame counts against the update.
//     A chunk with no reply goes once more; a reply 0x07 to it then means "arrived, go on". Any other break: 10 single
//     bytes 60 ms apart (the boot checker leaves an update after 10 cut frames), HELLO, again from chunk 0. 3 tries.
//   4 BOOT accepted: the Rx STM32 link starts again as after the USB bridge; "done" when its frames come (5 s).
// Nothing is kept across an ESP32 restart: after a failure the same command starts again from step 1.
#define SU_HDR_LEN 52
#define SU_CHUNK_LEN 1040
#define SU_MAX_CHUNKS 112  // boot checker: image_size 1 to 114688 B, 1024 B a chunk
#define SU_TRIES 3
#define SF_GAP_MS 60       // more than the boot checker's 50 ms frame gap
enum : uint8_t { SU_IDLE, SU_DOWNLOAD, SU_FEED, SU_DONE, SU_FAILED };  // "stm status" state
// Feed steps. SF_RESTART: the command 41 sequence in the normal loop; SF_START: BOOT accepted, frames awaited.
enum : uint8_t { SF_RESTART, SF_SETTLE, SF_STATUS, SF_PROBE, SF_ABORT, SF_HELLO, SF_BEGIN, SF_CHUNK, SF_FINAL, SF_BOOT, SF_START };
const char* const SF_NAMES[] = {"restart", "settle", "status", "probe", "abort", "hello", "begin", "chunks", "finalize",
                                "boot", "start"};
uint8_t su_state = SU_IDLE, sf_step = SF_RESTART, sf_try = 1, sf_count = 0;
bool sf_on = false;      // loop() runs the feed branch
bool sf_resent = false;  // the chunk in hand went twice
uint16_t su_chunks = 0, sf_chunk = 0;
uint32_t su_size = 0, su_got = 0, su_t0 = 0, su_print_ms = 0, sf_ms = 0, sf_timeout = 0, sf_resend_at = 0, sf_step_ms = 0;
const esp_partition_t* su_part = nullptr;
uint8_t su_hdr[SU_HDR_LEN];
uint8_t* sf_frame = nullptr;            // one CHUNK frame, 6 + 1042 B, held for the feed only
uint8_t sf_rx[32], sf_rx_n = 0;         // the reply frame in hand
uint8_t sf_pl[16], sf_plen = 0;         // the last complete reply: payload
uint16_t sf_cid = 0;
char su_err[112] = "";

bool stmUrlBusy(){ return su_state == SU_DOWNLOAD || su_state == SU_FEED; }

// The Rx STM32 link learns its version again (the app may be new or gone), as at the end of the USB bridge.
void stmLinkRelearn(){
  stm_link_ver = 0;
  stm_v2_cfg_sent = false;
  stmDec.reset();
  serial1Drain();
  data_ready = 0;
  stm_v2_resync = true;
  stm_last_packet_ms = millis();  // the silence recovery counts from here
}

void sfEnd(){
  free(sf_frame);
  sf_frame = nullptr;
  sf_on = false;
}

void stmUrlFail(const char* fmt, ...){
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(su_err, sizeof(su_err), fmt, ap);
  va_end(ap);
  const bool restarted = su_state == SU_FEED && sf_step != SF_RESTART;  // the Rx STM32 left its app
  su_state = SU_FAILED;
  sfEnd();
  if (restarted) stmLinkRelearn();  // a running app comes back; a boot checker gets HELLO/BOOT from the silence recovery
  usbOut.printf("STM url failed: %s (%lu ms)\n", su_err, (unsigned long)(millis() - su_t0));
  telemetryEvent(rec::EV_STM32, 0, 44, (String("stm url failed: ") + su_err).c_str());
}

// One reply frame from the boot checker, read a byte at a time (FINALIZE sends two frames back to back).
// 1: complete and valid (sf_cid, sf_pl, sf_plen); 0: not yet; -1: bad length or CRC (dropped).
int sfRead(){
  while (Serial1.available()) {
    if (sf_rx_n >= sizeof(sf_rx)) { sf_rx_n = 0; return -1; }
    sf_rx[sf_rx_n++] = (uint8_t)Serial1.read();
    if (sf_rx_n < 4) continue;
    const uint16_t len = sf_rx[0] | (uint16_t)sf_rx[1] << 8;
    if (len < 6 || len > sizeof(sf_rx) || len - 6 > (int)sizeof(sf_pl)) { sf_rx_n = 0; return -1; }
    if (sf_rx_n < len) continue;
    sf_rx_n = 0;
    if (crc16_ibm(sf_rx, len - 2) != (uint16_t)(sf_rx[len - 2] | sf_rx[len - 1] << 8)) return -1;
    sf_cid = sf_rx[2] | (uint16_t)sf_rx[3] << 8;
    sf_plen = (uint8_t)(len - 6);
    memcpy(sf_pl, sf_rx + 4, sf_plen);
    return 1;
  }
  return 0;
}

// One frame to the boot checker; its payload is in sf_frame + 4 (n bytes). Old input is dropped first.
void sfSend(uint16_t cid, uint16_t n, uint32_t timeout_ms){
  uint8_t* f = sf_frame;
  f[0] = (uint8_t)(n + 6); f[1] = (uint8_t)((n + 6) >> 8);
  f[2] = (uint8_t)cid; f[3] = (uint8_t)(cid >> 8);
  const uint16_t crc = crc16_ibm(f, 4 + n);
  f[4 + n] = (uint8_t)crc; f[5 + n] = (uint8_t)(crc >> 8);
  serial1Drain();
  sf_rx_n = 0;
  Serial1.write(f, 6 + n);  // the whole frame in one write: no gap inside it
  sf_ms = millis();
  sf_timeout = timeout_ms;
}

void sfStep(uint8_t s){ sf_step = s; sf_ms = sf_step_ms = millis(); sf_count = 0; }

void sfSendChunk(){
  uint8_t* p = sf_frame + 4;
  p[0] = (uint8_t)sf_chunk; p[1] = (uint8_t)(sf_chunk >> 8);
  if (esp_partition_read(su_part, SU_HDR_LEN + (uint32_t)SU_CHUNK_LEN * sf_chunk, p + 2, SU_CHUNK_LEN) != ESP_OK) {
    stmUrlFail("flash read at chunk %u", (unsigned)sf_chunk);
    return;
  }
  sfSend(0x0003, 2 + SU_CHUNK_LEN, sf_chunk == 0 ? 5000 : 1000);  // chunk 0: the app erase, about 1.3 s
  sf_resend_at = 0;
}

void sfChunks(uint16_t from){
  sf_step = SF_CHUNK;
  sf_chunk = from;
  sf_resent = false;
  sfSendChunk();
}

void sfBegin(){
  memcpy(sf_frame + 4, su_hdr, SU_HDR_LEN);
  sf_step = SF_BEGIN;
  sfSend(0x0002, SU_HDR_LEN, 2000);
}

void sfHello(){
  sf_step = SF_HELLO;
  sfSend(0x0001, 0, 300);
}

// A break: again from chunk 0 (10 cut frames first, HELLO after), up to SU_TRIES tries.
void sfBreak(const char* fmt, ...){
  char why[80];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(why, sizeof(why), fmt, ap);
  va_end(ap);
  if (sf_try >= SU_TRIES) { stmUrlFail("%s (try %u of %u)", why, (unsigned)sf_try, SU_TRIES); return; }
  usbOut.printf("STM feed break: %s (try %u of %u); again from chunk 0\n", why, (unsigned)sf_try, SU_TRIES);
  sf_try++;
  sf_chunk = 0;
  sfStep(SF_ABORT);
}

void sfService(){
  const uint32_t now = millis();
  const int r = (sf_step >= SF_STATUS && sf_step != SF_ABORT && sf_step != SF_PROBE) ? sfRead() : 0;
  const bool late = now - sf_ms >= sf_timeout;
  switch (sf_step) {
    case SF_SETTLE:  // after the command 41 pass: the app's last bytes go, the boot checker starts
      if (now - sf_ms < SF_GAP_MS) return;
      serial1Drain();
      sf_rx_n = 0;
      sfStep(SF_STATUS);
      sf_timeout = 1000;
      return;
    case SF_STATUS:  // the boot checker's start frame (CID 1, 7 B); else HELLO
      if (r == 1 && sf_cid == 0x0001 && sf_plen == 7) { sfBegin(); return; }
      if (late) sfHello();
      return;
    case SF_PROBE:  // no Rx STM32 frames: a boot checker is silent, a running app is not
      if (now - sf_ms < SF_GAP_MS) return;
      if (Serial1.available()) { stmUrlFail("the Rx STM32 sends, but not on the v2 link: no update"); return; }
      sf_count = 0;
      sfHello();
      return;
    case SF_ABORT:  // one byte, then more than 50 ms of silence: a cut frame; 10 of them end an update in progress
      if (now - sf_ms < SF_GAP_MS) return;
      if (sf_count < 10) { const uint8_t b = 0; Serial1.write(&b, 1); sf_count++; sf_ms = now; return; }
      serial1Drain();
      sf_count = 0;
      sfHello();
      return;
    case SF_HELLO:
      if (r == 1 && sf_cid == 0x0001 && sf_plen == 7) { sfBegin(); return; }  // idle: a new update from the header
      if (r == 1 && sf_cid == 0x0001 && sf_plen == 4 && sf_pl[0] == 0x09) { sfChunks(0); return; }  // waits for chunk 0
      if (!late) return;
      if (++sf_count < 3) { sfHello(); return; }  // sfHello keeps the count
      sfBreak("no boot checker answer to HELLO");
      return;
    case SF_BEGIN:
      if (r == 1 && sf_cid == 0x0002 && sf_plen == 4) {
        if (sf_pl[0] == 0) sfChunks(0);
        else sfBreak("BEGIN_UPDATE reply 0x%02x", sf_pl[0]);
        return;
      }
      if (late) sfBreak("no BEGIN_UPDATE reply");
      return;
    case SF_CHUNK:
      if (r == 1 && sf_cid == 0x0003 && sf_plen == 4) {
        const uint16_t idx = sf_pl[0] | (uint16_t)sf_pl[1] << 8;
        const uint8_t why = sf_pl[2];
        const bool ok = idx == sf_chunk && (why == 0 || (sf_resent && why == 0x07));
        // the reply to the last chunk was lost and the boot checker is FINALIZED: a resend gets [09 00 00 00]
        const bool last_in = sf_resent && sf_chunk == su_chunks - 1 && sf_pl[0] == 0x09 && sf_pl[1] == 0 && why == 0;
        if (!ok && !last_in) { sfBreak("chunk %u reply index %u reason 0x%02x", (unsigned)sf_chunk, (unsigned)idx, why); return; }
        if (!last_in && ++sf_chunk < su_chunks) { sf_resent = false; sfSendChunk(); return; }
        sf_chunk = su_chunks;
        sf_step = SF_FINAL;
        sfSend(0x0004, 0, 3000);
        return;
      }
      if (sf_resend_at) {
        if ((int32_t)(now - sf_resend_at) >= 0) { sfSendChunk(); sf_resent = true; }
        return;
      }
      if (!late) return;
      if (sf_resent) { sfBreak("no reply to chunk %u", (unsigned)sf_chunk); return; }
      sf_resend_at = now + SF_GAP_MS;  // silence first: the boot checker drops a cut frame
      return;
    case SF_FINAL:  // two frames on success: boot-status (CID 1), then the reply (CID 4)
      if (r == 1 && sf_cid == 0x0004 && sf_plen == 4) {
        if (sf_pl[0] != 0) { sfBreak("FINALIZE reply 0x%02x", sf_pl[0]); return; }
        sf_step = SF_BOOT;
        sfSend(0x0005, 0, 2000);
        return;
      }
      if (late) { sf_step = SF_BOOT; sfSend(0x0005, 0, 2000); }  // reply lost: BOOT tells whether the image is in
      return;
    case SF_BOOT:
      if (r == 1 && sf_cid == 0x0005 && sf_plen == 4) {
        if (sf_pl[0] != 0) { sfBreak("BOOT reply 0x%02x", sf_pl[0]); return; }
        sfEnd();
        stmLinkRelearn();
        sfStep(SF_START);
        usbOut.printf("STM url: BOOT accepted after %u chunks (try %u); waiting for Rx STM32 frames\n", (unsigned)su_chunks,
                      (unsigned)sf_try);
        return;
      }
      if (late) sfBreak("no BOOT reply");
      return;
    default:
      return;
  }
}

// Feed start: after the command 41 pass (SF_SETTLE), or with no Rx STM32 frames (SF_PROBE).
void sfOpen(uint8_t first){
  sf_frame = (uint8_t*)malloc(6 + 2 + SU_CHUNK_LEN);
  if (!sf_frame) { stmUrlFail("no memory for the feed (%u B)", 6 + 2 + SU_CHUNK_LEN); return; }
  if (stm_cal_valid) stmCalSetPending(true);  // an update erases the Rx STM32 settings: the next v2 start writes the copy
  serial1Drain();
  data_ready = 0;
  sf_chunk = 0;
  sf_on = true;
  sfStep(first);
}

// From stmRestartEnter(): true when the command 41 pass belongs to "stm url" (the ESP32 feeds; no USB bridge).
bool stmUrlOn41Pass(){
  if (su_state != SU_FEED || sf_step != SF_RESTART) return false;
  stm_rs_phase = RS_IDLE;
  stm_rs_n = 0;
  usbOut.printf("STM url: command 41 passed in %lu ms; the ESP32 feeds the boot checker\n", (unsigned long)(millis() - sf_step_ms));
  telemetryEvent(rec::EV_STM32, 1, STM_RS_CID, "stm url: Rx STM32 restart command passed");
  sfOpen(SF_SETTLE);
  return true;
}

// From stmRestartFail().
void stmUrlOn41Fail(const char* why, long a, long b){
  if (su_state == SU_FEED && sf_step == SF_RESTART) stmUrlFail("command 41: %s (%ld, %ld)", why, a, b);
}

// The chunk count for a file size: 52 + 1040 x count, 1 to 112 chunks; 0 for any other size.
uint16_t stmUrlChunks(unsigned long size){
  const unsigned long n = size > SU_HDR_LEN ? (size - SU_HDR_LEN) / SU_CHUNK_LEN : 0;
  return n == 0 || n > SU_MAX_CHUNKS || size != SU_HDR_LEN + SU_CHUNK_LEN * n ? 0 : (uint16_t)n;
}
const char SU_SIZE_ERR[] = "size must be 52 + 1040 x chunk count (1 to 112 chunks)";

static uint32_t rd32(const uint8_t* p){ return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

void stmUrlDownloaded(){
  if (su_chunks == 0) {  // an upload from the update page: the size is known only now
    su_size = su_got;
    su_chunks = stmUrlChunks(su_size);
    if (!su_chunks) { stmUrlFail("%s; the file is %lu B", SU_SIZE_ERR, (unsigned long)su_size); return; }
  }
  if (esp_partition_read(su_part, 0, su_hdr, SU_HDR_LEN) != ESP_OK) { stmUrlFail("flash read of the header"); return; }
  // The boot checker's own first checks (BEGIN_UPDATE), here before command 41: a file that is not an Rx STM32 update
  // file leaves the Rx STM32 app untouched.
  if (memcmp(su_hdr, "SIAP", 4) || rd32(su_hdr + 4) != 0x46544352 || (su_hdr[16] | su_hdr[17] << 8) != 1024
      || rec::crc32(su_hdr, 48) != rd32(su_hdr + 48)) {
    stmUrlFail("not an Rx STM32 update file (header magic, product or CRC-32)");
    return;
  }
  const uint16_t n = su_hdr[18] | (uint16_t)su_hdr[19] << 8;
  if (n != su_chunks) { stmUrlFail("header chunk count %u, the size gives %u", (unsigned)n, (unsigned)su_chunks); return; }
  usbOut.printf("STM url: file in flash, SHA-256 equal (%lu B, %u chunks, %lu ms)\n", (unsigned long)su_size,
                (unsigned)su_chunks, (unsigned long)(millis() - su_t0));
  su_state = SU_FEED;
  sf_chunk = 0;
  sf_try = 1;
  if (esp_partition_read(su_part, 0, stm_rs_buf, STM_RS_LEN) != ESP_OK) { stmUrlFail("flash read of the first 1092 B"); return; }
  stm_rs_n = STM_RS_LEN;
  if (stm_link_ver != 2) {
    // 0.6.0: frame edges but no frame this ESP32 reads (an app with another frame format, or failing frames): command
    // 41 goes with no answer awaited, then the feed as after a normal pass. No edges: a boot checker waits (HELLO).
    if ((uint32_t)((uint32_t)esp_timer_get_time() - stm_edge_us) < 200000u) {
      usbOut.println("STM url: Rx STM32 frames not read: command 41 with no answer awaited");
      sfStep(SF_RESTART);
      stm_rs_t0 = millis();
      if (stmRestartBlind()) { stmRestartEnter(); return; }
    }
    usbOut.println("STM url: no Rx STM32 frames: HELLO to the boot checker");
    sfOpen(SF_PROBE);
    return;
  }
  sfStep(SF_RESTART);
  if (const char* e = stmRestartGo()) { stmUrlFail("command 41 refused: %s", e); return; }
  usbOut.println("STM url: calibration copy, then idle mode and command 41");
}

// The refusals before an Rx STM32 download or upload, in this order (size 0: not known yet). nullptr: none.
const char* stmStartRefusal(unsigned long size){
  if (stmUrlBusy()) return "an Rx STM32 update runs";
  if (usb_ota::busy()) return "an ESP32 update runs (ota abort stops it)";
  if (stm_bridge || raw_mode || stm_raw || stm_rs_phase != RS_IDLE || stm_cal_phase != CAL_IDLE) return "the Rx STM32 link is busy";
  if (size && !stmUrlChunks(size)) return SU_SIZE_ERR;
  su_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "tlog");
  if (!su_part) return "no tlog partition";
  return nullptr;
}

// The download or upload runs: "stm url" state from step 1 (chunks 0: not known until the end of an upload).
void stmDownloadArmed(unsigned long size, uint16_t chunks){
  su_state = SU_DOWNLOAD;
  su_size = size;
  su_chunks = chunks;
  su_got = 0;
  su_t0 = su_print_ms = millis();
  su_err[0] = 0;
  sf_try = 1;
  sf_chunk = 0;
}

void stmUrlCommand(const char* arg, String& out){
  char url[160], hex[72];
  unsigned long size = 0;
  if (sscanf(arg, "%159s %lu %71s", url, &size, hex) != 3) { out = "STM error usage: stm url http://a.b.c.d[:port]/path <size> <sha256>"; return; }
  if (const char* e = stmStartRefusal(size)) { out = String("STM error ") + e; return; }
  const uint16_t n = stmUrlChunks(size);
  char err[96];
  if (!usb_ota::fetch_start(url, size, hex, su_part, err, sizeof(err))) { out = String("STM error ") + err; return; }
  stmDownloadArmed(size, n);
  out = String("STM url started size=") + size + " chunks=" + n;
}

// For the update page (src/web_update.cpp): the "stm url" result, and its reason.
int stmUrlOutcome(){ return su_state == SU_DONE ? 1 : su_state == SU_FAILED ? -1 : stmUrlBusy() ? 0 : -2; }
const char* stmUrlError(){ return su_err; }

// The update page asks to start an upload (src/web_update.cpp); this runs in loop() context. kind 1: ESP32 image, the
// checks of "ota url"; kind 2: Rx STM32 file, the checks of "stm url". size 0: not known (a form post). The reasons
// are the command texts. True: the upload runs (update mode on, as for the commands).
bool webUpdateGrant(uint8_t kind, unsigned long size, const char* sha, char* err, size_t cap){
  extern uint32_t web_upload_quiet_ms;
  web_upload_quiet_ms = millis() | 1;  // 0.6.15: telemetry datagrams pause for the upload (wifiTelemetrySend)
  if (kind == 1) {
    if (stmUrlBusy()) { snprintf(err, cap, "an Rx STM32 update runs"); return false; }
    if (!usb_ota::push_begin(nullptr, size, sha, err, cap)) return false;
    usbOut.printf("OTA upload started from the update page (size=%lu)\n", size);
    return true;
  }
  if (const char* e = stmStartRefusal(size)) { snprintf(err, cap, "%s", e); return false; }
  if (!usb_ota::push_begin(su_part, size, sha, err, cap)) return false;
  stmDownloadArmed(size, stmUrlChunks(size));
  usbOut.printf("STM upload started from the update page (size=%lu)\n", size);
  return true;
}

// After a failed ESP32 upload from the update page: update mode ends at once (after "ota url" it holds 120 s).
void webUpdateModeEnd(){
  if (!update_mode || usb_ota::busy() || stmUrlBusy()) return;
  update_mode = false;
  enableLoopWDT();
  usbOut.println("UPDATE mode off: the upload from the update page ended; loop watchdog on");
}

void stmStatusLine(String& out){
  char t[200];
  if (su_state == SU_DOWNLOAD)
    snprintf(t, sizeof(t), "STM status state=download got=%lu size=%lu", (unsigned long)su_got, (unsigned long)su_size);
  else if (su_state == SU_FEED)
    snprintf(t, sizeof(t), "STM status state=feed chunk %u of %u step=%s try=%u", (unsigned)sf_chunk, (unsigned)su_chunks,
             SF_NAMES[sf_step], (unsigned)sf_try);
  else if (su_state == SU_DONE)
    snprintf(t, sizeof(t), "STM status state=done chunks=%u tries=%u", (unsigned)su_chunks, (unsigned)sf_try);
  else if (su_state == SU_FAILED)
    snprintf(t, sizeof(t), "STM status state=failed error=\"%s\"", su_err);
  else
    snprintf(t, sizeof(t), "STM status state=idle");
  out = t;
}

// Each loop() pass: the download result, the time limits outside the feed branch, one progress line a second.
void stmUrlService(){
  if (su_state != SU_DOWNLOAD && su_state != SU_FEED) return;
  const uint32_t now = millis();
  if (su_state == SU_DOWNLOAD) {
    uint32_t got = 0;
    const char* why = "";
    const usb_ota::FetchResult fr = usb_ota::fetch_poll(&got, &why);
    su_got = got;
    if (fr == usb_ota::FETCH_OK || fr == usb_ota::FETCH_FAILED) {
      update_mode = false;  // the download held update mode; the Rx STM32 link comes back for the next step
      enableLoopWDT();
      usbOut.println("UPDATE mode off: the Rx STM32 download ended; loop watchdog on");
      serial1Drain();
      data_ready = 0;
      stm_v2_resync = true;
      stm_last_packet_ms = now;
      if (fr == usb_ota::FETCH_FAILED) stmUrlFail("download: %s", why);
      else stmUrlDownloaded();
      return;
    }
  } else if (sf_step == SF_RESTART && now - sf_step_ms > 30000) {  // e.g. no Rx STM32 frames: the sequence never runs
    stm_cal_then_restart = false;
    stmRestartFail("no progress in 30 s", stm_rs_phase, stm_rs_i);  // run mode again; "stm url" fails through the hook
    return;
  } else if (sf_step == SF_START) {
    if (stm_link_ver == 2) {
      su_state = SU_DONE;
      usbOut.printf("STM url done: Rx STM32 updated and started (%u chunks, try %u, %lu ms)\n", (unsigned)su_chunks,
                    (unsigned)sf_try, (unsigned long)(now - su_t0));
      telemetryEvent(rec::EV_STM32, 1, 44, "stm url done: Rx STM32 updated and started");
      return;
    }
    if (now - sf_step_ms > 5000) { stmUrlFail("BOOT accepted, but no Rx STM32 frames in 5 s"); return; }
  }
  if (now - su_print_ms < 1000) return;
  su_print_ms = now;
  if (su_state == SU_DOWNLOAD)
    usbOut.printf("STM progress download %lu/%lu heap=%lu\n", (unsigned long)su_got, (unsigned long)su_size, (unsigned long)ESP.getFreeHeap());
  else if (sf_step == SF_RESTART)
    usbOut.printf("STM progress feed chunk 0 of %u step=restart word %u of %u\n", (unsigned)su_chunks, (unsigned)stm_rs_i, STM_RS_LEN / 4);
  else
    usbOut.printf("STM progress feed chunk %u of %u step=%s try=%u\n", (unsigned)sf_chunk, (unsigned)su_chunks, SF_NAMES[sf_step], (unsigned)sf_try);
}

// The version of the Rx STM32 app from its own answer to a read of CID 30 (bytes: major, minor, patch, link protocol),
// read once 2 s after each v2 link start.
uint32_t rx_stm32_ver = 0, rx_stm32_ver_link_ms = 0, rx_stm32_ver_ask_ms = 0;
bool rx_stm32_ver_ok = false;
void stmVersionService(uint32_t now){
  if (rx_stm32_ver_ok || stm_rs_phase != RS_IDLE || stm_cal_phase != CAL_IDLE || stmLink.busy()) return;
  if (now - rx_stm32_ver_link_ms < 2000 || now - rx_stm32_ver_ask_ms < 2000) return;
  rx_stm32_ver_ask_ms = now;
  stmLink.push(30, 0, 0);
}
String rxStm32Version(){
  if (!rx_stm32_ver_ok || stm_link_ver != 2) return "unknown";
  return String(rx_stm32_ver & 0xFF) + "." + ((rx_stm32_ver >> 8) & 0xFF) + "." + ((rx_stm32_ver >> 16) & 0xFF);
}

// Keeps the first line printed through it (the radio answer) and passes every byte on to USB.
class LineCapture : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    usbOut.write(b, n);
    for (size_t i = 0; i < n && !done_; i++) {
      if (b[i] == '\n' || b[i] == '\r') done_ = len_ > 0;
      else if (len_ < sizeof(buf_) - 1) buf_[len_++] = (char)b[i];
    }
    buf_[len_] = 0;
    return n;
  }
  void reset() { len_ = 0; done_ = false; buf_[0] = 0; }
  const char* line() const { return buf_; }
 private:
  char buf_[248] = "";
  size_t len_ = 0;
  bool done_ = false;
};
LineCapture line_cap;

// Commands that run while an update holds loop() (update mode, "stm url"); the others are refused.
bool updateAllowed(const char* t){
  return !strncmp(t, "ota ", 4) || !strncmp(t, "stm url", 7) || !strcmp(t, "stm status") || !strncmp(t, "get ", 4)
         || !strncmp(t, "defget ", 7)
         || !strcmp(t, "restart");
}

// One update command from USB or the radio. True: handled; line: the answer, printed on USB here.
bool updateCommand(const char* t, bool radio, String& line){
  if (!strncmp(t, "ota ", 4)) {
    const char* a = t + 4;
    const bool url = !strncmp(a, "url ", 4);
    if (url && stmUrlBusy()) line = "OTA error an Rx STM32 update runs";
    else if (!radio) return false;  // USB: every ota command as before (usb_ota prints its line)
    else if (!url && strcmp(a, "status") && strcmp(a, "abort")) line = "OTA error USB only";
    else {
      update_last_ms = millis();
      line_cap.reset();
      Print* old = usb_ota::set_out(&line_cap);
      usb_ota::command(a);
      usb_ota::set_out(old);
      line = line_cap.line();  // on USB already
      return true;
    }
  } else if (!strncmp(t, "stm url ", 8) || !strcmp(t, "stm url")) {
    stmUrlCommand(t + 7, line);
  } else if (!strcmp(t, "stm status")) {
    stmStatusLine(line);
  } else if (!strcmp(t, "get rx_stm32_version")) {
    line = String("CFG 0 rx_stm32_version=") + rxStm32Version();
  } else {
    return false;
  }
  usbOut.println(line);
  return true;
}
uint8_t updateResult(const String& line){ return line.startsWith("OTA error") || line.startsWith("STM error") ? 4 : 0; }

// Update mode and the feed: radio commands go on (CMD frames from the server over Wi-Fi, or ESP-NOW).
void updateRadioService(){
  if (radio_wifi) wifilink::pollRx(millis());
  espnowCommands();
}

// The feed branch of loop(): the feed, USB commands and the radio commands, nothing else.
void stmFeedLoop(){
  sfService();
  if (!sf_on) return;
  benchSerial();
  updateRadioService();
}
