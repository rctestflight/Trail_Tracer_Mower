#ifdef DIAG_STEPTIME
void steptime_onStep(uint8_t seq);  // src/diag_steptime.cpp
#endif
// The link to the Rx STM32: frame interrupt, frame read and decode, commands, start handshake, DSP settings,
// sweep and polarity records.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.


StmTxPacket stmPacket;  // Data container for UART buffer and individual values
#define SERIAL1_FLUSH_MAX_BYTES 64  // Limit post-read drain to avoid long blocking stalls
#define SERIAL1_FLUSH_MAX_US 150  // Microsecond budget for post-read drain

volatile uint32_t data_ready = 0;  // incremented by ISR each time STM32 asserts nDATA_READY
TaskHandle_t loop_task_handle = nullptr;  // 0.5.98: the frame edge wakes loop() from its end-of-pass wait
volatile uint32_t stm_edge_us = 0;
uint32_t uart_overrun_count = 0;   // diagnostic counter, incremented on each overrun recovery
// Layout 2 (0.5.50): a late loop pass keeps the frames. The UART buffer (2048 B) holds 16 frames of 128 B; each pass
// reads one, in order, while 128 B or more wait. Frames stay aligned: each one has its CRC. A bad CRC drains the buffer
// and sets stm_v2_resync: the next frame counts only when exactly 128 B wait at an nDATA_READY edge. No command goes
// out while frames wait (stm_v2_backlog): one command per STM32 command window.
bool stm_v2_resync = true;
bool stm_v2_backlog = false;
volatile uint32_t uart_last_good_ms = 0;  // millis() of the last good Rx STM32 frame
uint32_t uart_skip_catchup = 0;   // Rx UART frame: good frames read in the catch-up loop (older frames: no block)
uint32_t uart_skip_backlog = 0;   // good frames with a newer frame already waiting (the block goes with the newest)
uint32_t stm_catchup_frames = 0;
#define CONTROL_PERIOD_US 10000  // 0.5.96: control at most at 100 Hz (the drive part costs the same at every reading time)
uint32_t control_last_us = 0;  // layout 2: older frames decoded without the control step (bench "rt" prints it)
uint32_t stm_backlog_frames = 0;   // frames read from the backlog (a late pass), counted
uint32_t crc_error_count = 0;      // diagnostic counter, incremented on each CRC mismatch
uint16_t last_computed_crc = 0;    // last CRC computed by the ESP32, for diagnostics
int stm_v2_len = STM_FRAME_LEN;  // 0.5.128: the layout 2 frame length in use: 128 B (version 2) or 144 B (version 3)
/*
Commands to the STM32 (0.5.25: lib/stm_settings). Use the espPacketFifo_addCmdxxx functions to queue a
command, as before. espPacketFifo_sendCmd runs once per valid STM32 packet: it reads the STM32 answer
in the packet (CMD_ACK, rid, response), then sends at most one command. The link sends the next
command only after the answer to the one before, or after a timeout; it sends a command with no
answer again (not 3, 4, 5), counts each failure, reads back the five settings (1, 2, 6, 7, 8) one each
2 s, and sends the five again after a wrong read-back or a gap in the packets. Events: 18.
*/
stmset::Link stmLink;
// STM32 link version: learned in-band from the first good frame. At the nDATA_READY edge the whole
// frame is buffered, so its length tells the version: 122 B = v1 (all shipped STM32 apps), 128 B with
// proto_ver 2 and a good CRC = Trail Tracer v2.0 link, layout 2 (include/rctf_wfrx_comms_v2.h, app
// 2.0.2 and later). Layout 1 (98 B, app 2.0.0/2.0.1 prototypes) is no longer read. 50 bad frames in a
// row make the version unknown again (for example after an STM32 app update through the bridge).
stmv2::Decoder stmDec;
// Bench "rt bench": CPU cycles of the live v2 frame read and decode (mean, max since the
// last report), then 1000 decodes of the last good frame in a copy of the decoder, then single float operations.
uint8_t rt_last_frame[STM_FRAME_LEN];
bool rt_last_ok = false;
uint64_t rt_read_cyc = 0, rt_dec_cyc = 0, rt_ans_cyc = 0, rt_post_cyc = 0;
uint32_t rt_read_max = 0, rt_dec_max = 0, rt_bench_n = 0;
stmv2::Answer stmAns;
uint8_t stm_link_ver = 0;          // 0 not known, 1 = v1, 2 = v2.0 layout 2
uint32_t stm_bad_frames_run = 0;
bool stm_v2_cfg_sent = false;      // lanes and detail formats sent after v2 was learned
uint8_t stm_v2_lanes = 0x02;       // CID 13: lane 1 = sensor 1, lane 2 = sensor 2 (v1 behaviour)
bool stm_v2_save_ok = false;       // layout 2: a CID 3 goes out only after a centre calibration
// Layout 2 (app 2.0.6+): the STM32 boots idle; CID 1 = 1 starts acquisition, after the configuration.
// In the queue the run command has its own ID (STM_RUN_CMD): CID 1 there is the v1 channel select.
#define STM_RUN_CMD 126
bool stm_v2_autorun = true;        // send run after the configuration and each STM32 reset ('stm run 0' stops it)
uint32_t stm_v2_run_ms = 0;        // millis() of the last run command queued
uint32_t stm_ans_print_until = 0;  // bench: print each v2 answer until this millis()
// Bench: raw v2 frames into RAM ('stm cap <n>', n <= 300, 128 B each, heap), printed
// later at the pace the USB queue takes ('stm dump'). Command round trip in frames: the frame seq when a
// command goes out against the frame seq of its answer (stm v2 'rtt').
uint8_t* stm_cap = nullptr;
uint16_t stm_cap_n = 0, stm_cap_want = 0, stm_cap_out = 0;
// Bench "stm stream <s>" (coded-Tx test): per good v2 frame one line with frame bytes 1 (seq), 14-19 and
// 38-43 (bin 0 coil 1 and coil 2 of result 1, CID 40 raw mode), 80-83 (t0) as hex, and the PGA bits; about 17 KB/s
// at 2354 us frames. A line that does not fit the USB queue is dropped (counted in the end line).
uint32_t stm_stream_end = 0, stm_stream_n = 0, stm_stream_drop = 0;
bool stm_cap_dump = false;
bool stm_rtt_wait = false;
uint8_t stm_rtt_cmd_seq = 0, stm_rtt_frame_seq = 0;
uint32_t stm_rtt_hist[16] = {};  // [d] = answers d frames after the send (15 = 15 or more)
uint8_t stm_dup_frame[ESP_FRAME_LEN];  // bench 'stm dup': one v2 command frame sent twice, 2 frames apart
int stm_dup_step = 0;               // 0 off; 1 send; 2, 3 wait; 4 send again
uint32_t stm_last_packet_ms = 0;   // millis() of the last valid STM32 packet
bool stm_packet_seen = false;      // a valid STM32 packet this boot
bool stm_rom_used = false;         // "stm rom" this boot: no HELLO/BOOT recovery (a ROM loader may listen)
uint8_t stm_recover_tries = 0;     // HELLO/BOOT recoveries in this packet gap
uint32_t stm_recover_ms = 0;
uint32_t stm_iap_boots = 0;        // HELLO/BOOT recoveries that started the STM32 application
uint32_t stm_iap_fails = 0;

// IAP boot handshake ---------------------------------------------------------------------------------------------------------------
#define IAP_CID_HELLO             0x0001
#define IAP_CID_BOOT              0x0005
#define IAP_STATUS_OK             0x00
#define IAP_MAX_FRAME             32    // largest expected frame is 13 bytes; 32 gives headroom
#define IAP_HELLO_RETRIES         10
#define IAP_HELLO_TIMEOUT_MS      100
#define IAP_BOOT_TIMEOUT_MS       100
#define IAP_DATA_READY_TIMEOUT_MS 200

static void iap_encode_frame(uint16_t cid, uint8_t *buf) {
  // Zero-payload frame: length(2LE) + cid(2LE) + crc16_ibm(2LE)
  buf[0] = 0x06; buf[1] = 0x00;
  buf[2] = cid & 0xFF; buf[3] = (cid >> 8) & 0xFF;
  uint16_t crc = crc16_ibm(buf, 4);
  buf[4] = crc & 0xFF; buf[5] = (crc >> 8) & 0xFF;
}

static bool iap_read_frame(uint32_t timeout_ms, uint16_t *cid_out, uint8_t *payload_out, uint8_t *payload_len_out) {
  uint32_t deadline = millis() + timeout_ms;

  while (Serial1.available() < 4) { if (millis() > deadline) return false; }
  uint8_t hdr[4];
  Serial1.readBytes(hdr, 4);
  uint16_t length = hdr[0] | ((uint16_t)hdr[1] << 8);
  uint16_t cid    = hdr[2] | ((uint16_t)hdr[3] << 8);
  if (length < 6 || length > IAP_MAX_FRAME) return false;

  uint8_t rest_len = length - 4;  // payload + 2 CRC bytes
  uint8_t rest[IAP_MAX_FRAME];
  while (Serial1.available() < rest_len) { if (millis() > deadline) return false; }
  Serial1.readBytes(rest, rest_len);

  uint8_t plen = length - 6;
  uint16_t crc_recv = rest[plen] | ((uint16_t)rest[plen + 1] << 8);
  uint8_t crc_buf[4 + IAP_MAX_FRAME];
  memcpy(crc_buf, hdr, 4);
  memcpy(crc_buf + 4, rest, plen);
  if (crc16_ibm(crc_buf, 4 + plen) != crc_recv) return false;

  *cid_out = cid;
  *payload_len_out = plen;
  memcpy(payload_out, rest, plen);
  return true;
}

void stm32_boot_handshake() {
  stmCalLoad();
  if (stmIapHoldTake()) {  // one-shot: HELLO only, then the USB bridge (stmBridgeService)
    uint8_t frame[6], payload[IAP_MAX_FRAME], plen = 0;
    uint16_t cid = 0;
    bool ok = false;
    for (int i = 0; i < IAP_HELLO_RETRIES && !ok; i++) {
      serial1Drain();
      iap_encode_frame(IAP_CID_HELLO, frame);
      Serial1.write(frame, 6);
      ok = iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen) && cid == IAP_CID_HELLO;
    }
    usbOut.printf("STM bridge on: STM32 held in its boot checker (HELLO %s); USB is now the STM32 UART, no text until it ends\n",
                  ok ? "answered" : "not answered");
    usbOut.drain(500);
    serial1Drain();
    data_ready = 0;  // edges from the STM32 power-up are not an application
    stm_bridge = true;
    stm_bridge_start_ms = millis();
    return;
  }
  // After a firmware upload only the ESP32 resets; the STM32 is already running
  // its application and will not respond to IAP HELLO.  Skip the handshake for
  // any reset reason other than a cold power-on so we don't restart-loop.
  // If the STM32 is silent after an ESP32-only reset, it waits in its bootloader
  // (for example after an STM32 load), so send HELLO/BOOT anyway. The running app
  // signals a packet on nDATA_READY every 10 ms.
  esp_reset_reason_t reset_reason = esp_reset_reason();
  // 0.5.63: only UART bytes show a running application. The boot checker also makes nDATA_READY
  // edges (bench, 2026-10-12: an RTS reset with the STM32 in its boot checker gave "handshake
  // skipped" and no STM32 start). An application sends a frame at least every frame period, so
  // 250 ms with no byte means the boot checker.
  if (reset_reason != ESP_RST_POWERON) {
    data_ready = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 250 && Serial1.available() == 0) delay(1);
    const int bytes = Serial1.available();
    if (bytes > 0) {
      usbOut.printf("STM32 handshake skipped (reset reason: %d, STM32 app running: %d B, %lu edges)\n",
                    (int)reset_reason, bytes, (unsigned long)data_ready);
      return;
    }
    usbOut.printf("STM32 silent after reset (%lu nDATA_READY edges, no byte in 250 ms): sending HELLO/BOOT\n",
                  (unsigned long)data_ready);
  }

  char t[72];
  uint8_t plen = 0;
  const int r = stm32_iap_boot(IAP_HELLO_RETRIES, t, sizeof(t), &plen);
  if (r == 1) {
    usbOut.println("IAP HELLO failed");
    telemetryEvent(rec::EV_STM32, 0, 0, "IAP HELLO failed: STM32 gives no answer");
    return;  // STM32 may already be running; continue without handshake
  }
  telemetryEvent(rec::EV_STM32, 1, plen, t);  // the HELLO answer as it came (IAP and app versions, image state)
  if (r == 2) usbOut.println("IAP BOOT failed");
  else if (r == 3) usbOut.println("IAP nDATA_READY timeout");
  else usbOut.println("STM32 boot OK");
}

// Drain the STM32 UART input: only the bytes there now, read in blocks. A byte-by-byte drain
// ("while available: read") never ends when frames arrive faster than one read() per byte (bench:
// 2.3 ms frames, a byte each 18 us): loop() hung and the task watchdog reset the board.

size_t serial1Drain(){
  uint8_t tmp[128];
  size_t total = 0;
  int n = Serial1.available();
  while (n > 0) {
    const size_t k = Serial1.read(tmp, n < (int)sizeof(tmp) ? (size_t)n : sizeof(tmp));
    if (k == 0) break;
    total += k;
    n -= (int)k;
  }
  return total;
}

// Boot guard. The STM32 keeps its frame period over an ESP32 reset. At frames below about 2.7 ms the loop
// did not keep up (task watchdog reset, set-up S2N), so the next boot would reset again. Before loop()
// starts: more than 100 frames a second (more than about 1500 B in 50 ms) can only come from a v2 STM32;
// it gets a write of the frame period 10000 us. A v1 STM32 sends 100 packets a second and gets nothing.
void stmFastFrameGuard(){
  for (int attempt = 0; attempt < 3; attempt++) {
    const uint32_t t0 = millis();
    uint32_t n = 0;
    while (millis() - t0 < 50) {
      n += serial1Drain();
      delay(1);
    }
    if (n < 1500) return;
    uint8_t f[ESP_FRAME_LEN];
    stmv2::encodeCmd((uint8_t)esp_random(), 18 | ESP_CID_WRITE, 0, 10000, f);
    Serial1.write(f, ESP_FRAME_LEN);
    usbOut.printf("STM32 fast frames at boot (%lu B in 50 ms): frame period 10000 us sent\n", (unsigned long)n);
    delay(30);
  }
}

// HELLO (up to hello_tries sends), then BOOT, then the first nDATA_READY of the STM32 application.
// Result: 0 ok, 1 no HELLO answer, 2 BOOT failed, 3 nDATA_READY timeout. hello_text: the HELLO
// answer as it came, "hello <n> B: xx ..." (IAP and application versions, image state).
int stm32_iap_boot(int hello_tries, char* hello_text, size_t text_len, uint8_t* hello_len){
  uint8_t frame[6];
  uint8_t payload[IAP_MAX_FRAME];
  uint8_t plen;
  uint16_t cid;
  hello_text[0] = 0;
  *hello_len = 0;

  // Flush, send HELLO, wait for HELLO response; retry on flush-race or garbled frame
  bool hello_ok = false;
  for (int i = 0; i < hello_tries && !hello_ok; i++) {
    serial1Drain();
    iap_encode_frame(IAP_CID_HELLO, frame);
    Serial1.write(frame, 6);
    if (!iap_read_frame(IAP_HELLO_TIMEOUT_MS, &cid, payload, &plen)) continue;
    if (cid != IAP_CID_HELLO) continue;
    hello_ok = true;
  }
  if (!hello_ok) return 1;
  {
    int k = snprintf(hello_text, text_len, "hello %u B:", (unsigned)plen);
    for (int j = 0; j < plen && k < (int)text_len - 3; j++) k += snprintf(hello_text + k, text_len - k, " %02x", payload[j]);
    *hello_len = plen;
  }

  // BOOT — STM32 state confirmed by HELLO, single attempt
  iap_encode_frame(IAP_CID_BOOT, frame);
  Serial1.write(frame, 6);
  if (!iap_read_frame(IAP_BOOT_TIMEOUT_MS, &cid, payload, &plen) ||
      cid != IAP_CID_BOOT || plen < 1 || payload[0] != IAP_STATUS_OK) {
    return 2;
  }

  // Wait for STM32 app to assert nDATA_READY for the first time
  uint32_t deadline = millis() + IAP_DATA_READY_TIMEOUT_MS;
  while (digitalRead(STM_nDATA_READY) != LOW) {
    if (millis() > deadline) return 3;
  }
  return 0;
}


// An STM32 that restarts alone stays in its boot checker: IAP 0.1.0 (TrailTracer-Rx-STM32
// iap_fsm.c) sends one HELLO frame at its start and then waits for BOOT, so no packet comes and
// the STM32 settings are lost. After STM_RECOVER_GAP_MS with no valid packet (packets seen earlier
// in this boot), send HELLO/BOOT again: 3 HELLO tries, at most 3 times per gap, 5 s apart. Only
// when the line is silent for 50 ms: a running STM32 application reads 8-byte commands, and a 6-byte
// HELLO would shift every later command. When packets come back, the link sends the five settings
// again (gap event). Not after "stm rom" (a ROM loader may listen), not in update mode, not with
// the drive off for a failed IMU (no packet interrupt). Blocks loop() for 50 ms to 650 ms; the
// drive outputs stopped already, 250 ms after the last packet (ODRIVE_OUTPUT_TIMEOUT_MS).
#define STM_RECOVER_GAP_MS 1000
void stmRecoveryService(){
  if (stm_raw || stm_rom_used || imu_halt || update_mode || !stm_packet_seen || raw_mode || raw_stm_state) return;  // never during a raw stream
  const uint32_t now = millis();
  if (now - stm_last_packet_ms < STM_RECOVER_GAP_MS) { stm_recover_tries = 0; return; }
  // 0.5.110: 3 tries 5 s apart, then one every 30 s for as long as the Rx STM32 stays silent (it gave up after 3). An
  // Rx STM32 window watchdog reset (2.0.63+) leaves it in its boot checker, which waits for this BOOT.
  if (stm_recover_tries > 0 && now - stm_recover_ms < (stm_recover_tries < 3 ? 5000u : 30000u)) return;
  if (stm_recover_tries < 250) stm_recover_tries++;
  stm_recover_ms = now;
  const int pending = Serial1.available();  // the boot checker's own HELLO frame at its start: 13 bytes
  serial1Drain();
  data_ready = 0;
  delay(50);
  char t[72];
  if (data_ready > 0 || Serial1.available() > 0) {
    snprintf(t, sizeof(t), "silent %lu ms, line not silent (%lu edges, %d B): no HELLO",
             (unsigned long)(now - stm_last_packet_ms), (unsigned long)data_ready, Serial1.available());
    usbOut.printf("STM32 %s\n", t);
    telemetryEvent(rec::EV_STM32_CMD, stmset::K_IAP_BOOT, -1, t);
    return;
  }
  char hello[72];
  uint8_t plen = 0;
  const int r = stm32_iap_boot(3, hello, sizeof(hello), &plen);
  serial1Drain();
  data_ready = 0;
  if (r == 0) stm_iap_boots++; else stm_iap_fails++;
  static const char* const RESULT[] = {"boot ok", "no HELLO answer", "BOOT failed", "no nDATA_READY"};
  snprintf(t, sizeof(t), "silent %lu ms, %d B: %s; %s", (unsigned long)(now - stm_last_packet_ms), pending,
           RESULT[r], hello[0] ? hello : "-");
  usbOut.printf("STM32 HELLO/BOOT try %u: %s\n", (unsigned)stm_recover_tries, t);
  telemetryEvent(rec::EV_STM32_CMD, stmset::K_IAP_BOOT, r, t);
}

// STM32 UART ISR, triggered by falling edge of nDATA_READY after each full packet
DRAM_ATTR volatile bool uart_cmd_raw = false;      // 0.6.29: the command frame holds command 42 ("stm raw")
void IRAM_ATTR receiveStmUart(){
  data_ready++;
  stm_edge_us = (uint32_t)esp_timer_get_time();
  rtprof::edgeIsr();
  BaseType_t woken = pdFALSE;
  uart_edge_seq++;
  // 0.5.128: the Rx UART frame goes now: the command frame once, else the newest block. The live flags are read here too:
  // a raw stream, a restart or "uart off|pause" starts in loop() between two builds.
  const uint32_t isr_ms = (uint32_t)(esp_timer_get_time() / 1000);  // millis() base
  // ... and Rx STM32 frames still run: a boot checker makes edges too, with no frame (no byte may go to it)
  if (uart_isr_go && (uint32_t)(isr_ms - uart_last_good_ms) < 100 && !raw_mode && !rawStreamRuns() && !stm_dup_step &&
      stm_rs_phase != RS_RESTART && !uart_force_old && !(uart_pause_until && (int32_t)(isr_ms - uart_pause_until) < 0)) {
    if (uart_ll_get_txfifo_len(UART_LL_GET_HW(1)) < ESP_UART_FRAME_LEN) {
      uart_isr_fifo++;
    } else if (uart_cmd_ready) {
      uart_ll_write_txfifo(UART_LL_GET_HW(1), uart_cmdf, ESP_UART_FRAME_LEN);
      uart_cmd_ready = false;
      if (uart_cmd_raw) raw_stm_state = 0xFD;  // 0.6.29: command 42 went: no byte more (one would stop the stream)
      uart_frames++;
    } else if (uart_blk_ok) {
      if (uart_blk_sent) uart_isr_repeat++;
      uart_ll_write_txfifo(UART_LL_GET_HW(1), uart_blk[uart_blk_i], ESP_UART_FRAME_LEN);
      uart_blk_sent = true;
      uart_frames++;
    } else {
      uart_isr_none++;
    }
  }
  if (loop_task_handle) vTaskNotifyGiveFromISR(loop_task_handle, &woken);  // 0.5.98: wake loop() at once
  if (woken) portYIELD_FROM_ISR();
}

void flushSerial1RxBounded(){
  uint32_t start_us = micros();
  uint16_t flushed = 0;
  while(Serial1.available() && (flushed < SERIAL1_FLUSH_MAX_BYTES)){
    Serial1.read();
    flushed++;
    if((micros() - start_us) >= SERIAL1_FLUSH_MAX_US){
      break;
    }
  }
}

// CRC-16/IBM: poly 0x8005, init 0xFFFF, no input/output reflection, no final XOR.
static uint16_t crc16_ibm(const uint8_t *data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i = 0; i < len; i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b = 0; b < 8; b++){
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
  }
  return crc;
}

void espPacketFifo_addCmdInt32(uint32_t cid, int32_t cmd){
  stmLink.pushInt(cid, cmd);  // a full queue: dropped, counted, event 18
}

void espPacketFifo_addCmdFloat(uint32_t cid, float cmd){
  stmLink.pushFloat(cid, cmd);
}

// The answer in a v2 frame (0.5.97: called for every decoded frame from stmReadFrame, catch-up and decode-only frames
// included; up to 0.5.96 only the frame that ran control had its answer read, and the others were lost:
// "STMCMD no answer" at 2.35 ms frames).
void stmV2Answer(){
  const uint32_t now = millis();
  if (stmAns.present && (int32_t)(stm_ans_print_until - now) > 0)
    usbOut.printf("STM ans seq=%u cid=%u%s index=%u status=%u value=%ld (0x%08lx)\n", (unsigned)stmAns.seq, (unsigned)(stmAns.cid & 0x7F),
                  (stmAns.cid & 0x80) ? "w" : "r", (unsigned)stmAns.index, (unsigned)stmAns.status, (long)(int32_t)stmAns.value,
                  (unsigned long)stmAns.value);
  if (stm_rtt_wait && stmAns.present && stmAns.seq == stm_rtt_cmd_seq) {
    const uint8_t d = (uint8_t)(stmDec.lastSeq() - stm_rtt_frame_seq);
    stm_rtt_hist[d < 15 ? d : 15]++;
    stm_rtt_wait = false;
  }
  stmCalOnAnswer(stmAns);
  stmRestartOnAnswer(stmAns);
  stmRawStreamOnAnswer(stmAns);
  dspOnAnswer(stmAns);
  if (stmAns.present && stmAns.cid == 30 && (stmAns.status == RSP_OK || stmAns.status == RSP_OK_CLAMPED)) {
    rx_stm32_ver = stmAns.value;  // app major, minor, patch, link protocol (bytes 0-3)
    rx_stm32_ver_ok = true;
  }
  if (stmAns.present && (stmAns.cid & 0x7F) == 20 && stmAns.index < WFRX_BINS && (stmAns.status == RSP_OK || stmAns.status == RSP_OK_CLAMPED))
    memcpy(&stm_bin_hz[stmAns.index], &stmAns.value, 4);  // the bin frequency in effect, for STMCOIL8
  stmLink.onAnswerV2(now, stmDec.status_bits, stmAns.present, stmAns.seq, stmAns.cid, stmAns.index, stmAns.status, stmAns.value);
}

// Rx UART frame (Rx STM32 app with status bit 27): every frame carries the control block (version, angle = vehicle
// roll, flags: bit 0 charge enable) and one optional command, with one CRC. Until the app sets bit 27 the ESP32 sends
// the 10 B command frame only when it has a command (older apps). Bench "uart": status, roll source, bad blocks, pause.
uint8_t uart_roll_src = 0;       // 0 IMU (accelerometer roll; 0 with no IMU), 1 fixed, 2 sweep (one turn in 10 s)
int16_t uart_roll_fixed = 0;     // fixed roll, 65 536 = one turn
bool uart_charge = false;        // flag bit 0: the charge state machine's CID 9 value
bool uart_force_old = false;     // bench "uart off": the 10 B command frame even with bit 27
uint8_t uart_bad_kind = 0;       // bench: 1 bad CRC, 2 wrong version (block-only frames; command frames stay valid)
int16_t uart_bad_angle = 0;      // bench: the angle the bad frames carry (a kept bad block would show it in CID 53)
int8_t uart_charge_force = -1;   // bench "uart charge on|off|auto": -1 the charge state machine, 0 or 1 forced
uint32_t uart_pause_t0 = 0;      // millis() at the last "uart pause"
int32_t uart_cp_off_ms = -1;     // ms from the last pause start to the charge enable status bit (13) falling; -1 none
uint32_t uart_bad_left = 0;      // frames still to send bad
uint32_t uart_pause_until = 0;   // bench "uart pause <s>": millis() until frames go again
uint32_t uart_frames = 0, uart_frames_cmd = 0, uart_frames_bad = 0, uart_frames_old = 0;
int16_t uart_last_angle = 0;
uint8_t uart_last_blk[ESP_UART_FRAME_LEN], uart_last_cmd[ESP_UART_FRAME_LEN];  // the last block-only and command frames sent
bool uart_have_blk = false, uart_have_cmd = false;
bool uartFramePaused() { return uart_pause_until && (int32_t)(millis() - uart_pause_until) < 0; }
// During "uart pause" no control block goes (no valid 16 B frame); commands go as 10 B frames, so reads still work.
bool uartFrameOn() { return !uart_force_old && !uartFramePaused() && (stmDec.status_bits & WFRX_STATUS_UART_FRAME) != 0; }
int16_t uartRoll() {
  if (uart_roll_src == 1) return uart_roll_fixed;
  if (uart_roll_src == 2) return (int16_t)(uint16_t)((millis() % 10000u) * 65536u / 10000u);
  return IMU_ROLL16();
}
// 0.5.128: the Rx UART frame goes from the frame edge interrupt (receiveStmUart) straight into the UART transmit FIFO.
// loop() builds the frames after each good Rx STM32 frame (uartRefresh): the block alone in a double buffer, and a
// command frame when a command waits. The interrupt sends the command frame once, else the newest block (again, if
// loop() built none since). 0.5.125-0.5.127 used a task for the build; its 3 KB stack cost the page update its heap.
portMUX_TYPE uart_mux = portMUX_INITIALIZER_UNLOCKED;
bool uart_pend_on = false;        // a command waits for the next frame build
uint8_t uart_pend_seq = 0, uart_pend_cid = 0, uart_pend_idx = 0;
uint32_t uart_pend_raw = 0;
uint32_t uart_pend_busy = 0;      // commands offered while one still waited (dropped: the command link retries it)
uint32_t uart_task_wait = 0;      // builds with the frame mode not allowed (no frame run, raw stream, restart, pause)
DRAM_ATTR uint8_t uart_blk[2][ESP_UART_FRAME_LEN];  // block-only frames: the interrupt sends uart_blk[uart_blk_i]
DRAM_ATTR uint8_t uart_cmdf[ESP_UART_FRAME_LEN];    // the command frame
volatile uint8_t uart_blk_i = 0;
volatile bool uart_blk_ok = false;      // a block frame is built and the mode runs
volatile bool uart_cmd_ready = false;   // the command frame waits for the next edge
volatile bool uart_isr_go = false;      // the Rx UART frame mode runs and frames may go
volatile uint32_t uart_isr_none = 0;    // edges with the mode on and no frame built yet
volatile uint32_t uart_isr_fifo = 0;    // edges with the transmit FIFO too full (another writer)
volatile uint32_t uart_isr_repeat = 0;  // edges that sent the same block again (loop() built none since)
volatile uint32_t uart_edge_seq = 0;    // +1 at each frame edge
volatile bool uart_blk_sent = false;    // the newest block has gone at least once
uint32_t uart_jitter_us = 0, uart_jit_busy = 0;  // 0.5.127 bench test, not in this build (kept for the "uart" line)
// The frames go only while Rx STM32 frames run (a boot checker also makes edges, with no frame), and never in a raw
// stream (any byte ends it), the duplicate bench test or a restart into the boot checker.
bool uartTaskMaySend() {
  return uartFrameOn() && stm_link_ver == 2 && !raw_mode && !rawStreamRuns() && !stm_dup_step && stm_rs_phase != RS_RESTART &&
         (uint32_t)(millis() - uart_last_good_ms) < 100;
}
// Build one Rx UART frame into out.
void uartBuild(uint8_t* out, uint8_t seq, uint8_t cid8, uint8_t idx, uint32_t raw) {
  uint8_t ver = ESP_UART_FRAME_VERSION;
  uint16_t crc_xor = 0;
  int16_t angle = uartRoll();
  if (uart_bad_left && cid8 == 0) {  // a bad block-only frame: the commands keep passing in the valid frames
    uart_bad_left--;
    uart_frames_bad++;
    if (uart_bad_kind == 1) crc_xor = 0x5A5A; else ver = ESP_UART_FRAME_VERSION + 1;
    angle = uart_bad_angle;
  } else {
    uart_last_angle = angle;
  }
  const bool charge = uart_charge_force >= 0 ? uart_charge_force == 1 : uart_charge;
  stmv2::encodeUartFrame(ver, angle, charge ? ESP_UART_FLAG_CHARGE : 0u, seq, cid8, idx, raw, out, crc_xor);
  if (cid8) { uart_frames_cmd++; memcpy(uart_last_cmd, out, ESP_UART_FRAME_LEN); uart_have_cmd = true; }
  else { memcpy(uart_last_blk, out, ESP_UART_FRAME_LEN); uart_have_blk = true; }
}
// After each good Rx STM32 frame (stmReadFrame): the next block, and the command frame when one waits.
void uartRefresh() {
  const bool go = uartTaskMaySend();
  uart_isr_go = go;
  if (!go) { uart_blk_ok = false; uart_cmd_ready = false; uart_task_wait++; return; }
  const uint8_t n = uart_blk_i ^ 1u;
  uartBuild(uart_blk[n], 0, 0, 0, 0);
  uart_blk_i = n;  // the interrupt now sends the new block
  uart_blk_sent = false;
  uart_blk_ok = true;
  if (!uart_cmd_ready) {
    portENTER_CRITICAL(&uart_mux);
    const bool have = uart_pend_on;
    const uint8_t s = uart_pend_seq, c = uart_pend_cid, i = uart_pend_idx;
    const uint32_t r = uart_pend_raw;
    uart_pend_on = false;
    portEXIT_CRITICAL(&uart_mux);
    if (have) { uartBuild(uart_cmdf, s, c, i, r); uart_cmd_raw = c == (42 | ESP_CID_WRITE); uart_cmd_ready = true; }
  }
}
// One command to the Rx STM32: with the Rx UART frame it waits in the pending slot for the next frame build; else the
// 10 B command frame goes now (older apps, "uart off", "uart pause").
void stmTx(uint8_t seq, uint8_t cid8, uint8_t idx, uint32_t raw) {
#ifdef DIAG_LAT2
  rtprof::stmTxAt();
#endif
  if (!uartFrameOn()) {
    uint8_t f[ESP_FRAME_LEN];
    stmv2::encodeCmd(seq, cid8, idx, raw, f);
    Serial1.write(f, ESP_FRAME_LEN);
    uart_frames_old++;
    return;
  }
  portENTER_CRITICAL(&uart_mux);
  if (uart_pend_on) {
    uart_pend_busy++;
  } else {
    uart_pend_seq = seq; uart_pend_cid = cid8; uart_pend_idx = idx; uart_pend_raw = raw;
    uart_pend_on = true;
  }
  portEXIT_CRITICAL(&uart_mux);
}
// The block alone: the edge interrupt sends a frame at each edge, so loop() sends nothing here (kept for the call sites).
void stmTxIdle() {}
void stmTxBlockOnly() {}
// Bench "uart [roll imu|sweep|<deg> | bad crc|ver <frames> [deg] | pause <s> | charge on|off|auto | off | on]": the Rx
// UART frame state and tests. "pause": no control block for <s> seconds; commands go as 10 B frames meanwhile.
void uartCommand(const char* a) {
  char w[8] = {0};
  float v = 0;
  unsigned long n = 0;
  if (!strncmp(a, "roll ", 5)) {
    const char* r = a + 5;
    if (!strcmp(r, "imu")) uart_roll_src = 0;
    else if (!strcmp(r, "sweep")) uart_roll_src = 2;
    else if (sscanf(r, "%f", &v) == 1 && v >= -180 && v < 180) { uart_roll_src = 1; uart_roll_fixed = (int16_t)lroundf(v * 65536.0f / 360.0f); }
    else { usbOut.println("UART usage: uart roll imu|sweep|<deg -180..179.99>"); return; }
  } else if (sscanf(a, "bad %7s %lu %f", w, &n, &v) >= 2 && (!strcmp(w, "crc") || !strcmp(w, "ver")) && n >= 1 && n <= 100000 &&
             v >= -180 && v < 180) {
    uart_bad_kind = !strcmp(w, "crc") ? 1 : 2;
    uart_bad_angle = (int16_t)lroundf(v * 65536.0f / 360.0f);
    uart_bad_left = n;
  } else if (sscanf(a, "pause %lu", &n) == 1 && n >= 1 && n <= 120) {
    uart_pause_until = millis() + n * 1000;
    uart_pause_t0 = millis();
    uart_cp_off_ms = -1;
  } else if (!strncmp(a, "charge ", 7) && (!strcmp(a + 7, "on") || !strcmp(a + 7, "off") || !strcmp(a + 7, "auto"))) {
    uart_charge_force = !strcmp(a + 7, "on") ? 1 : !strcmp(a + 7, "off") ? 0 : -1;
  } else if (sscanf(a, "jitter %lu", &n) == 1) {  // the 0.5.127 proof test is not in this build
    usbOut.println("UART jitter: not in this build (0.5.127 only)");
    return;
  } else if (!strcmp(a, "last")) {  // the last frames sent, as bytes
    char h[2 * ESP_UART_FRAME_LEN + 1];
    for (int k = 0; k < 2; k++) {
      const uint8_t* f = k ? uart_last_cmd : uart_last_blk;
      for (unsigned i = 0; i < ESP_UART_FRAME_LEN; i++) snprintf(h + 2 * i, 3, "%02x", f[i]);
      usbOut.printf("UART last %s %s\n", k ? "command" : "block", (k ? uart_have_cmd : uart_have_blk) ? h : "none");
    }
    return;
  } else if (!strcmp(a, "off") || !strcmp(a, "on")) {
    uart_force_old = !strcmp(a, "off");
  } else if (*a) {
    usbOut.println("UART usage: uart | uart roll imu|sweep|<deg> | uart bad crc|ver <frames> [deg] | uart pause <s> | uart charge on|off|auto | uart jitter <max_us 0..3000> | uart last | uart off|on");
    return;
  }
  static const char* const src[] = {"imu", "fixed", "sweep"};
  usbOut.printf("UART frame=%s bit27=%d forced_old=%d frames=%lu cmd=%lu bad=%lu old=%lu bad_left=%lu roll=%s angle=%d (%.2f deg) charge=%d force=%d cp_enable=%d paused=%d cp_off_ms=%ld good_frames=%lu\n",
                uartFrameOn() ? "on" : "off", (int)((stmDec.status_bits & WFRX_STATUS_UART_FRAME) != 0), (int)uart_force_old,
                (unsigned long)uart_frames, (unsigned long)uart_frames_cmd, (unsigned long)uart_frames_bad, (unsigned long)uart_frames_old,
                (unsigned long)uart_bad_left, src[uart_roll_src], (int)uart_last_angle, uart_last_angle * 360.0f / 65536.0f,
                (int)uart_charge, (int)uart_charge_force, (int)((stmDec.status_bits >> 13) & 1u), (int)uartFramePaused(), (long)uart_cp_off_ms,
                (unsigned long)stmDec.st.frames);
  usbOut.printf("UART edge none=%lu fifo=%lu repeat=%lu wait=%lu pend_busy=%lu edges=%lu | from Rx STM32: crc_err=%lu ver_err=%lu seq_lost=%lu\n",
                (unsigned long)uart_isr_none, (unsigned long)uart_isr_fifo, (unsigned long)uart_isr_repeat, (unsigned long)uart_task_wait,
                (unsigned long)uart_pend_busy, (unsigned long)uart_edge_seq, (unsigned long)stmDec.st.crc_err, (unsigned long)stmDec.st.ver_err,
                (unsigned long)stmDec.st.seq_lost);
}


void espPacketFifo_sendCmd(){
  const uint32_t now = millis();
  stm_last_packet_ms = now;
  stm_packet_seen = true;
  const bool v2 = stm_link_ver == 2;
  stmLink.setV2(v2);
  if (v2) {
    if (!stm_v2_cfg_sent) {  // v2 only: lanes and the values the decoder needs (a v1 STM32 knows none of them)
      stm_v2_cfg_sent = true;
      rx_stm32_ver_ok = false;  // a link start: the app may be new; its version is read again 2 s later
      rx_stm32_ver_link_ms = now;
      stmV2Config();
    }
    // run after the configuration: at start (stmV2Config) and after each STM32 reset (bit 21 rises: the link
    // queued the settings again just now, so run goes after them). Not on status bit 24: app 2.0.6 does not
    // set it (bench test 2026-10-11), and each run command restarts the STM32 tracking state and PGA steps.
    static bool reset_prev = false;
    const bool reset_now = (stmDec.status_bits & WFRX_STATUS_STM_RESET) != 0;
    if (reset_now && !reset_prev && stm_v2_autorun && now - stm_v2_run_ms >= 1000) { dsp_apply_reason = "Rx STM32 reset bit"; dspApply(); }  // DSP settings, then run
    reset_prev = reset_now;
    stmCalService(now);
    stmRestartService(now);
    if (stm_rs_phase == RS_RESTART) return;  // the Rx STM32 restarts after this frame: nothing more to it
    if (raw_mode) return;  // a raw stream started on this answer: any byte to the Rx STM32 would stop it
    stmVersionService(now);  // "get rx_stm32_version": CID 30 once per link start
  } else {
    stmLink.onPacket(now, stmPacket.items.status_bits, stmPacket.items.rid, (uint32_t)stmPacket.items.response.int32);
  }
  if (v2 && stm_dup_step) {  // bench: the repeat rule (the STM32 answers a repeat within 10 frames, runs it once)
    if (stm_dup_step == 1 || stm_dup_step == 4) Serial1.write(stm_dup_frame, ESP_FRAME_LEN);
    stm_dup_step = stm_dup_step == 4 ? 0 : stm_dup_step + 1;
    return;  // no link command in these frames: one frame per window
  }
  if (v2 && stm_v2_backlog) { uart_skip_backlog++; return; }  // frames wait: this one is old; the command goes with the newest frame
  uint32_t cid, raw;
  stmLink.setPeriodRatio(periodRatio());  // the answer timeouts in packets at the update period in effect
  if (!stmLink.next(now, &cid, &raw)) { if (v2) stmTxIdle(); return; }
  if (!v2) {
    EspTxPacket p;
    p.items.cid = cid;
    p.items.cmd.int32 = (int32_t)raw;
    Serial1.write(p.uart_data, ESP_TX_PACKET_SIZE);
    return;
  }
  uint32_t id = cid & 0x7FFFFF;
  const bool w = (cid & stmset::WRITE) != 0;
  // The run command goes as CID 1 (layout 2 meaning). 0.6.34: the rewritten command keeps a run mark, so a retry (no
  // answer in time) sends the run again. Up to 0.6.33 a retry came back as a plain CID 1 write, the wire channel
  // command below: it set the sensor 1 wire bin to 1 and the rover read the wrong Tx frequency (no signal) until restart.
  const uint32_t RUN_ON_CID1 = 0x40000000u;  // outside the command id bits (stmset idOf: 0x7FFFFF)
  if (id == STM_RUN_CMD || (cid & RUN_ON_CID1)) {
    stmLink.rewrite(1 | stmset::WRITE | RUN_ON_CID1, raw, 0);
    stmTx(stmLink.seq(), 1 | ESP_CID_WRITE, 0, raw);
    return;
  }
  // Layout 2: the wire channel is a bin choice on the ESP32 (bins = Tx CH 1..5); the v1 detail formats
  // are gone; a save goes only after a centre calibration; centre calibration (4, 5) becomes a write of
  // the angle calibration (28) of that sensor: phi + atan2(a, b) of its present wire bin.
  if (id == 1 || id == 2) {
    if (w) {
      stmDec.setWireBin((int)id - 1, (raw & 0x80u) ? -1 : (int)(raw & 0x7Fu));
    }
    stmLink.doneLocal();
    stmTxIdle();
    return;
  }
  // Rx UART frame: the charge enable is flag bit 0 of the control block, in every frame; CID 9 does not go.
  if (id == 9 && w && uartFrameOn()) {
    uart_charge = raw != 0;
    stmLink.doneLocal();
    stmTxIdle();
    return;
  }
  // CID 3: a write (save) goes only after a centre calibration or a calibration restore; a read (2.0.52: flash state) goes as it is
  if (id == 11 || id == 12 || (id == 3 && w && !stm_v2_save_ok)) { stmLink.doneLocal(); stmTxIdle(); return; }
  if (id == 3 && w) stm_v2_save_ok = false;
  if ((id == 4 || id == 5) && w) {
    float phi;
    if (!stmDec.centreAngle((int)id - 4, &phi)) { stmLink.doneLocal(); stmTxIdle(); return; }
    uint32_t pr;
    memcpy(&pr, &phi, 4);
    stmLink.rewrite(28 | stmset::WRITE, pr, (uint8_t)(id - 4));
    stm_v2_save_ok = true;
    cid = 28 | stmset::WRITE;
    raw = pr;
  }
  const uint8_t seq = stmLink.seq();
  stmTx(seq, (uint8_t)((cid & 0x7F) | (w ? ESP_CID_WRITE : 0)), stmLink.idx(), raw);
  if (!stm_rtt_wait || stm_rtt_cmd_seq != seq) { stm_rtt_wait = true; stm_rtt_cmd_seq = seq; stm_rtt_frame_seq = stmDec.lastSeq(); }
}

// v2 layout 2: the STM32 streams magnitudes only; the ESP32 derives positions, weak and lost itself.
// Set and get are for setup and checks only: here the lanes (tracked) and 4 setup reads, the radius and
// angle calibration of each sensor (27, 28). Bin gains stay 1 (the magnitudes are PGA-calibrated). The
// wire bin of each sensor follows rx_channel and enable_sensor_2, as v1 CID 1 and 2.
// DSP settings (config.json dsp_*) to the STM32 v2 app, at each STM32 start and after a change. The STM32
// takes 38 and 39 only while idle, so: run 0, sensors (38), slow ADC (39), period (18; 39 changes its step),
// bins (20), lanes (13), PGA (29), xy mask (25), run 1 (TrailTracer-Rx-STM32 rctf_wfrx_comms_v2.h). The
// answers fill g_dsp_eff (dspOnAnswer): the values in effect, shown as the read-only <key>_eff settings.

// Bin sweep record (body 0x33, 0.5.103; Rx STM32 2.0.51+ page, docs/espnow_protocol.md). Each frame's sweep bins go
// to the log as they come: the frame seq, the page tag and previous-frame tag on a page frame, and the bin values. The
// PC places each reading on the grid (system split: the Rx measures, the PC analyses; the grid rule changed in Rx
// STM32 2.0.57 and needs no ESP32 change). Up to 0.5.102 the ESP32 placed the readings itself (body 0x32).
uint8_t sweep_body[TELEMLOG_FAST_BODY_MAX];
int sweep_body_n = 0, sweep_body_updates = 0;
uint32_t sweep_body_ms = 0;
uint32_t sweep_updates = 0, sweep_sent = 0, sweep_dropped = 0;
int32_t sweep_sent_bins = -1;  // the last CID 43 index 0 value sent (dspApply or "sweep 0 <n>")
static uint32_t sweepF0(const PageSweep& p){ return p.f0_dhz[0] | (p.f0_dhz[1] << 8) | ((uint32_t)p.f0_dhz[2] << 16); }
void sweepFlush(){
  if (sweep_body_updates) telemlog_add(TELEMLOG_FAST, sweep_body_ms, sweep_body, (size_t)sweep_body_n);
  sweep_body_n = sweep_body_updates = 0;
}
// The 4 int24 values of each sweep bin of this frame (sensor 1 coil 1, coil 2, sensor 2 coil 1, coil 2; as body 0x2E).
void sweepValues(uint8_t out[WFRX_BINS][12], uint8_t mask){
  int i = 0;
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
    if (!((mask >> bn) & 1u)) continue;
    int32_t v[4] = {rec::BINS_NONE, rec::BINS_NONE, rec::BINS_NONE, rec::BINS_NONE};
    bool seen[2] = {false, false};
    for (unsigned k = 0; k < WFRX_RESULTS; k++) {
      const unsigned sn = (stmDec.lanes >> k) & 1u;
      if (seen[sn]) continue;
      seen[sn] = true;
      v[sn * 2] = wfrx_int24_get(stmDec.coil[k][bn].v1);
      v[sn * 2 + 1] = wfrx_int24_get(stmDec.coil[k][bn].v2);
    }
    for (int c = 0; c < 4; c++) wfrx_int24_put(out[i] + 3 * c, v[c]);
    i++;
  }
}
// One frame into the record: seq, flags, tag, previous-frame tag, then n sweep bins of 12 B values.
void sweepAdd(uint8_t seq, uint8_t flags, uint16_t tag, uint16_t tag_prev, const uint8_t vals[WFRX_BINS][12]){
  const PageSweep& p = stmDec.sweep;
  sweep_updates++;
  int n = 0;
  for (unsigned b = 0; b < WFRX_BINS; b++) n += (p.mask >> b) & 1u;
  if (!n || !p.positions) return;
  const int need = rec::SWEEP_ENTRY_HDR + 12 * n;
  // a new table or bin set starts a new record (the header holds them)
  if (sweep_body_updates && (sweep_body[1] != p.table_id || sweep_body[10] != p.mask)) sweepFlush();
  // 0.5.96: a fixed record cap: Wi-Fi 40 records a second, ESP-NOW 4. The cost per datagram, not per byte, limits the
  // board (Wi-Fi test 2026-10-13); frames that find no open record and the cap reached are dropped (sweep_dropped);
  // the PC fills the trace over more sweeps.
  // 0.6.16: on Wi-Fi a record that closed full lets the next one open after 15 ms, not 25: at 10 ms frames with 7 sweep
  // bins a record holds 2 frames (20 ms), so every frame now gets in (it was 80 of 100, 560 of 700 values a second).
  static bool last_full = false;
  const uint32_t min_ms = radio_wifi ? (last_full ? 15 : 25) : 250;
  static uint32_t last_flush_ms = 0;
  if (sweep_body_updates == 0 && millis() - last_flush_ms < min_ms) { sweep_dropped++; return; }
  if (sweep_body_updates == 0) last_full = false;
  if (sweep_body_updates == 0) last_flush_ms = millis();
  if (sweep_body_updates && sweep_body_n + need > (int)sizeof(sweep_body)) sweepFlush();
  if (!sweep_body_updates) {
    const uint32_t f0 = sweepF0(p);
    uint8_t* o = sweep_body;
    o[0] = rec::BODY_SWEEP_RAW;
    o[1] = p.table_id;
    o[2] = (uint8_t)f0; o[3] = (uint8_t)(f0 >> 8); o[4] = (uint8_t)(f0 >> 16);
    o[5] = (uint8_t)p.step_dhz; o[6] = (uint8_t)(p.step_dhz >> 8);
    o[7] = (uint8_t)p.positions; o[8] = (uint8_t)(p.positions >> 8);
    o[9] = (uint8_t)n;
    o[10] = p.mask;
    o[11] = 0;  // frames in this body
    sweep_body_n = rec::SWEEP_RAW_HDR;
    sweep_body_ms = millis();
  }
  uint8_t* e = sweep_body + sweep_body_n;
  e[0] = seq; e[1] = flags;
  e[2] = (uint8_t)tag; e[3] = (uint8_t)(tag >> 8);
  e[4] = (uint8_t)tag_prev; e[5] = (uint8_t)(tag_prev >> 8);
  sweep_body_n += rec::SWEEP_ENTRY_HDR;
  for (int i = 0; i < n; i++) { memcpy(sweep_body + sweep_body_n, vals[i], 12); sweep_body_n += 12; }
  sweep_body_updates++;
  sweep_body[11] = (uint8_t)sweep_body_updates;
  sweep_sent++;
  if (sweep_body_n + need > (int)sizeof(sweep_body)) { last_full = true; sweepFlush(); }
}
// Each good frame (stmReadFrame), the catch-up frames included.
void sweepFrame(){
  if (!stmDec.sweep_on) return;
  const PageSweep& p = stmDec.sweep;  // the newest sweep page: bin set, grid and state
  if (p.state != 2 || !p.positions || !p.mask) return;  // no sweep runs
  uint8_t vals[WFRX_BINS][12];
  sweepValues(vals, p.mask);
  const bool page = stmDec.page_sweep;
  const uint8_t flags = (page ? 1 : 0) | ((stmDec.status_bits & WFRX_STATUS_FRAME_OVERRUN) ? 2 : 0);
  sweepAdd(stmDec.lastSeq(), flags, page ? p.tag : 0xFFFF, page ? p.tag_prev : 0xFFFF, vals);
  if (sweep_body_updates && millis() - sweep_body_ms >= (radio_wifi ? 25u : 250u)) sweepFlush();  // a record waits one cap period at most
}
// Polarity detector record (body 0x37, Rx STM32 2.0.67+ with status bit 26). Each frame's detector block goes to the
// log as it comes: the frame seq, then the block's 8 B header and its n samples (4 B each). The PC decoder (and the
// ESP32 decoder) reads every sample; the time base comes from the block header. Wi-Fi: a record at most every 20 ms
// (about 31 records a second carry the 978 samples a second); ESP-NOW: every 250 ms, the rest dropped (polar_dropped).
uint8_t polar_body[TELEMLOG_FAST_BODY_MAX];
int polar_body_n = 0, polar_body_frames = 0;
uint32_t polar_body_ms = 0, polar_last_start_ms = 0;
uint32_t polar_frames = 0, polar_samples = 0, polar_sent = 0, polar_dropped = 0, polar_lost_flags = 0;
void polarFlush(){
  if (polar_body_frames) telemlog_add(TELEMLOG_FAST, polar_body_ms, polar_body, (size_t)polar_body_n);
  polar_body_n = polar_body_frames = 0;
}
void polarFrame(){
  if (!stmDec.polar_on) { if (polar_body_frames) polarFlush(); return; }
  const uint8_t* blk = stmDec.polar;
  const int n = blk[0] > 10 ? 10 : blk[0];
  polar_frames++;
  polar_samples += n;
  if (blk[1] & 0x80) polar_lost_flags++;
  const int need = 1 + 8 + 4 * n;
  const uint32_t now = millis(), min_ms = radio_wifi ? 20u : 250u;
  if (polar_body_frames && polar_body_n + need > (int)sizeof(polar_body)) polarFlush();
  if (!polar_body_frames) {
    if (now - polar_last_start_ms < min_ms) { polar_dropped++; return; }
    polar_last_start_ms = now;
    polar_body[0] = rec::BODY_POLAR;
    polar_body[1] = 0;  // frames in this body
    polar_body_n = rec::POLAR_HDR;
    polar_body_ms = now;
  }
  uint8_t* e = polar_body + polar_body_n;
  e[0] = stmDec.lastSeq();
  memcpy(e + 1, blk, (size_t)(8 + 4 * n));
  polar_body_n += need;
  polar_body[1] = (uint8_t)++polar_body_frames;
  polar_sent++;
  if (polar_body_n + 1 + 8 + 40 > (int)sizeof(polar_body) || now - polar_body_ms >= min_ms) polarFlush();
}
const char* dsp_apply_reason = "boot";
// 0.6.29: values set on the bench in Rx STM32 RAM ("stm cmd 29 <v>": the PGA hold; "bin <i> <hz>": a bin frequency)
// stay through each DSP set (after a raw stream, an Rx STM32 reset), until a change of that setting or an ESP32 restart.
// Before 0.6.29 the DSP set after each "stm raw" returned them to the settings: a capture series lost its PGA hold.
bool pga_ram_on = false;
int32_t pga_ram = -1;
uint8_t bin_ram_mask = 0;
float bin_ram_hz[8] = {};
void dspApply(){
  if (stm_link_ver != 2) return;
  const DeviceConfig& c = g_deviceConfig;
  for (double& v : g_dsp_eff) v = NAN;
  stmLink.pushInt(STM_RUN_CMD | 0x80000000, 0);
  // 0.6.36: the setting "Sensor 2" (enable_sensor_2) sets the sensor count (CID 38) and the lanes (CID 13): on, 2 sensors,
  // lane 1 sensor 1 and lane 2 sensor 2 (0x02); off, 1 sensor, lane 1 sensor 1 (0x00), so that the Rx STM32 gives sensor 1
  // all its processing time. dsp_sensors and dsp_lanes are no user settings from 0.6.36.
  const bool s2 = c.enable_sensor_2;
  stmDec.setWireBin(1, s2 ? 0 : -1);
  stmLink.pushInt(38 | 0x80000000, s2 ? 2 : 1);
  stmLink.pushInt(39 | 0x80000000, c.dsp_slow_adc);
  int period_us = safe_mode ? 10000 : c.dsp_frame_period_us;  // safe mode: the default period
  const int link_m = linkModeOf(c, radio_wifi);
  if (period_us < periodFloorUs(link_m, c)) {  // 0.5.103: a config.json edit or a radio change: the minimum of the link mode
    usbOut.printf("STM DSP update period %d us too short for %s: %d us sent\n", period_us, linkModeName(link_m), periodFloorUs(link_m, c));
    period_us = periodFloorUs(link_m, c);
  }
  stmLink.pushInt(18 | 0x80000000, period_us);
  for (int b = 0; b < 8; b++) stmLink.pushFloat(20 | 0x80000000, (bin_ram_mask >> b & 1) ? bin_ram_hz[b] : (float)c.dsp_bin_hz[b], (uint8_t)b);
  stm_v2_lanes = s2 ? 0x02 : 0x00;
  stmLink.pushInt(13 | 0x80000000, stm_v2_lanes);
  stmLink.pushInt(29 | 0x80000000, pga_ram_on ? pga_ram : c.dsp_pga);
  stmLink.pushInt(25 | 0x80000000, c.dsp_xy_mask);
  // Bin sweep (CID 43 index 0, Rx STM32 2.0.51+; an older app answers unknown CID and sends no sweep page). Sent at
  // each DSP set: the Rx STM32 starts with the sweep on, so "off" must be sent too. Its OK answer sets the page layout
  // (stmDec.sweep_on). The range (index 1, 2) stays as the PC set it ("sweep 1 <Hz>", "sweep 2 <Hz>").
  sweep_sent_bins = c.dsp_sweep_bins;
  stmLink.pushInt(43 | 0x80000000, c.dsp_sweep_bins);
  // 0.6.11: the decoded Tx channels for polarity detection (CID 50 index 0; Rx STM32 2.1.134+). The Rx STM32 does not
  // store it: it goes at each DSP set. An older app refuses the unknown command once; the link moves on.
  stmLink.pushInt(50 | 0x80000000, c.dt_channels);
  // 0.6.21: the position thresholds per bin (CID 21 weak, 22 lost, 23 regained: magnitude = 1 / distance in mm) and
  // the regain wait (CID 24), from the settings; the answers set the values the decoder uses (stmDec.cal). The
  // sign spread limit (CID 54 index 14, deg x 10; Rx STM32 2.2.0+; an older app refuses it once).
  const double* weak[8] = {&c.rx_weak_mm_0, &c.rx_weak_mm_1, &c.rx_weak_mm_2, &c.rx_weak_mm_3,
                           &c.rx_weak_mm_4, &c.rx_weak_mm_5, &c.rx_weak_mm_6, &c.rx_weak_mm_7};
  const double* lost[8] = {&c.rx_lost_mm_0, &c.rx_lost_mm_1, &c.rx_lost_mm_2, &c.rx_lost_mm_3,
                           &c.rx_lost_mm_4, &c.rx_lost_mm_5, &c.rx_lost_mm_6, &c.rx_lost_mm_7};
  const double* rgn[8] = {&c.rx_regain_mm_0, &c.rx_regain_mm_1, &c.rx_regain_mm_2, &c.rx_regain_mm_3,
                          &c.rx_regain_mm_4, &c.rx_regain_mm_5, &c.rx_regain_mm_6, &c.rx_regain_mm_7};
  for (int b = 0; b < 8; b++) {
    stmLink.pushFloat(21 | 0x80000000, (float)(1.0 / *weak[b]), (uint8_t)b);
    stmLink.pushFloat(22 | 0x80000000, (float)(1.0 / *lost[b]), (uint8_t)b);
    stmLink.pushFloat(23 | 0x80000000, (float)(1.0 / *rgn[b]), (uint8_t)b);
  }
  stmLink.pushInt(24 | 0x80000000, c.rx_regain_wait_ms);
  stmLink.pushInt(54 | 0x80000000, (int32_t)lround(c.dt_spread_limit_deg * 10.0), 14);
  static uint32_t dsp_sets = 0;
  usbOut.printf("STM DSP set %lu (%s)\n", (unsigned long)++dsp_sets, dsp_apply_reason);  // bench: the rate of full DSP sets
  if (stm_v2_autorun) { stmLink.pushInt(STM_RUN_CMD | 0x80000000, 1); stm_v2_run_ms = millis(); }  // app 2.0.6: boots idle
}

// The STM32 frame period in effect, s: the v2 app's answer to CID 18 (dsp_frame_period_us_eff); 10 ms for a
// v1 app (100 packets a second) and before the first answer.
double stmFramePeriodS(){
  const double us = g_dsp_eff[DSP_EFF_PERIOD];
  return (stm_link_ver == 2 && isfinite(us) && us > 0) ? us * 1e-6 : 0.010;
}

// One v2 answer to a DSP write: the value in effect (the period in effect for 18, the bin frequency for 20).
void dspOnAnswer(const stmv2::Answer& a){
  // writes, and from 0.5.104 CID 20 reads: Rx STM32 2.0.58 answers the bin frequency in effect, which follows a write
  // about 10 frames later (binPollService)
  if (!a.present || (!(a.cid & 0x80) && (a.cid & 0x7F) != 20 && (a.cid & 0x7F) != 50) || (a.status != RSP_OK && a.status != RSP_OK_CLAMPED)) return;
  const int32_t v = (int32_t)a.value;
  switch (a.cid & 0x7F) {
    case 18: g_dsp_eff[DSP_EFF_PERIOD] = v; dsp_period_answers++; break;
    case 38: g_dsp_eff[DSP_EFF_SENSORS] = v; break;
    case 39: g_dsp_eff[DSP_EFF_SLOW] = v; break;
    case 13: g_dsp_eff[DSP_EFF_LANES] = v; break;
    case 29: g_dsp_eff[DSP_EFF_PGA] = v; break;
    case 25: g_dsp_eff[DSP_EFF_XY] = v; break;
    case 20: if (a.index < 8) { float f; memcpy(&f, &a.value, 4); g_dsp_eff[DSP_EFF_BIN0 + a.index] = f; } break;
    case 50: if (!(a.cid & 0x80) && a.index == 0) { dt_decoded = v; dt_decoded_ms = millis(); } break;  // 0.6.12: read: the channels decoded now
    case 43:  // index 0: the bin sweep is on or off, so the page layout (stm_v2.h)
      if (a.index == 0) stmDec.sweep_on = sweep_sent_bins != 0;
      break;
  }
}

void stmV2Config(){
  stmDec.setWireBin(0, (int)rx_channel - 1);
  for (int sn = 0; sn < 2; sn++) { stmLink.push(27, 0, (uint8_t)sn); stmLink.push(28, 0, (uint8_t)sn); }
  dsp_apply_reason = "Rx STM32 start";
  dspApply();  // lanes, the DSP settings, then run
  if (stm_cal_valid && stm_cal_phase == CAL_IDLE && stmCalPending()) {  // an IAP update ran: the calibration copy goes back
    usbOut.println("STM cal restore: started");
    stmCalStart(CAL_WRITE);
  }
}

int stmFrameLen(){ return stm_link_ver == 2 ? stm_v2_len : (int)sizeof(stmPacket.uart_data); }

bool stmReadFrame(){
  rtprof::frameRead();
  int n = Serial1.available();
  if (stm_link_ver == 0) { delayMicroseconds(150); n = Serial1.available(); }  // the last bytes of the frame
  bool ok = false;
  if (stm_link_ver == 1 || (stm_link_ver == 0 && n == (int)sizeof(stmPacket.uart_data))) {
    Serial1.readBytes(stmPacket.uart_data, sizeof(stmPacket.uart_data));
    // CRC covers every byte except the trailing crc field itself.
    const uint16_t computed_crc = crc16_ibm(stmPacket.uart_data, sizeof(stmPacket.uart_data) - sizeof(stmPacket.items.crc));
    last_computed_crc = computed_crc;
    ok = computed_crc == stmPacket.items.crc;
    rtprof::frameDecoded();
    if (!pass_kind) pass_kind = 1;  // diag_cpu: this pass read a frame
    if (ok && stm_link_ver == 0) { stm_link_ver = 1; usbOut.println("STM link v1: 122 B packets"); }
  } else if (stm_link_ver == 2 || (stm_link_ver == 0 && (n == (int)STM_FRAME_LEN || n == (int)STM_FRAME_LEN_V3))) {
    // 0.6.0: an unknown link learns layout 2 from a whole frame of 128 B (version 2) or 144 B (version 3); 0.5.128
    // took 128 B only, so a restart into a version 3 app (2.0.81) left the link unknown and every frame unread
    uint8_t f[STM_FRAME_MAX];
    const bool known = stm_link_ver == 2;
    const uint32_t c0 = ESP.getCycleCount();
    Serial1.readBytes(f, STM_FRAME_LEN);
    // 0.5.128: version 3 (Rx STM32 2.0.81+) is 144 B: the 16 B after the first 128 follow within 80 us at 2 Mbaud
    int len = (int)STM_FRAME_LEN;
    if (f[0] == WFRX_PROTO_VER3) {
      const uint32_t t0 = micros();
      while (Serial1.available() < (int)(STM_FRAME_LEN_V3 - STM_FRAME_LEN) && micros() - t0 < 1000) {}
      if (Serial1.available() >= (int)(STM_FRAME_LEN_V3 - STM_FRAME_LEN)) {
        Serial1.readBytes(f + STM_FRAME_LEN, STM_FRAME_LEN_V3 - STM_FRAME_LEN);
        len = (int)STM_FRAME_LEN_V3;
      }
    }
    const uint32_t c1 = ESP.getCycleCount();
    ok = stmDec.decode(f, stmPacket.items, stmAns, len, millis());
    if (ok) stm_v2_len = len;  // the next frames: the same length
    const uint32_t c1a = ESP.getCycleCount();
    if (ok && known) stmV2Answer();  // every frame's answer (0.5.97)
    const uint32_t c2 = ESP.getCycleCount();
    rt_ans_cyc += c2 - c1a;
    rtprof::frameDecoded();
    if (!pass_kind) pass_kind = 1;  // diag_cpu: this pass read a frame
    rt_read_cyc += c1 - c0; rt_dec_cyc += c2 - c1; rt_bench_n++;
    if (c1 - c0 > rt_read_max) rt_read_max = c1 - c0;
    if (c2 - c1 > rt_dec_max) rt_dec_max = c2 - c1;
    if (ok) { memcpy(rt_last_frame, f, STM_FRAME_LEN); rt_last_ok = true; }
    if (stm_stream_end) {
      if ((int32_t)(millis() - stm_stream_end) >= 0) {
        stm_stream_end = 0;
        usbOut.printf("STM stream end lines=%lu dropped=%lu\n", (unsigned long)stm_stream_n, (unsigned long)stm_stream_drop);
        stm_stream_drop = 0;
      } else if (ok) stmStreamFrame(f);
    }
    if (known) {  // layout 2: frames that wait stay in the buffer for the next passes
      stm_v2_backlog = Serial1.available() >= stm_v2_len;
      if (stm_v2_backlog) stm_backlog_frames++;
      if (data_ready) data_ready--;
      stm_v2_resync = !ok;
      if (!ok) { serial1Drain(); data_ready = 0; rtprof::resync(); }
    }
    if (ok && stm_cap && stm_cap_n < stm_cap_want) memcpy(stm_cap + (size_t)STM_FRAME_LEN * stm_cap_n++, f, STM_FRAME_LEN);
    if (ok && stm_acc_want) stmAccFrame();
    if (ok && stm_coil_ms) stmCoilFrame();
    // 0.5.102: the bin record takes 100 Hz input (rec::DECIM_IN_HZ): one frame each 10 ms. Up to 0.5.101 it took every
    // frame, so at 2.35 ms frames its records came 4.25 times too often, with the wrong time step, at 4.25 times the
    // cost (about 280 us a frame).
    // 2026-10-30: at a period factor other than 1 (update period 20 ms: 2), each frame goes in as one input per 10 ms of
    // update period (the frame's values held over its period), so the filter has its 100 inputs a second, the record its
    // set rate and its period label the true period. Period factor 1: the 10 ms clock grid, as before.
    static uint32_t bins_last_us = 0;
    static double bins_hold = 0;
    if (ok && periodRatio() != 1.0) {
      bins_hold += periodRatio();
      const int k = (int)bins_hold;
      bins_hold -= k;
      if (k > 0) telemetryBins(k);
    } else if (ok && (uint32_t)(micros() - bins_last_us) >= 10000) {
      bins_last_us += 10000;
      if ((uint32_t)(micros() - bins_last_us) >= 10000) bins_last_us = micros();  // a gap: start the grid again
      telemetryBins();
    }
    if (ok) sweepFrame();
    if (ok) polarFrame();
    if (ok) uart_last_good_ms = millis();
    if (ok) uartRefresh();  // 0.5.128: the next Rx UART frame (sent by the next frame edge)
    if (ok && uart_pause_t0 && uart_cp_off_ms < 0 && !((stmDec.status_bits >> 13) & 1u)) uart_cp_off_ms = (int32_t)(millis() - uart_pause_t0);
    if (ok && scan_on) scanFrame();
    rt_post_cyc += ESP.getCycleCount() - c2;  // 0.5.101: read, decode, answer and the rest apart (rt bench)
    if (ok && stm_link_ver == 0) { stm_link_ver = 2; stm_v2_resync = false; usbOut.printf("STM link v2.0 layout 2: %d B frames\n", stm_v2_len); }
    if (known) {
      if (ok) { stm_bad_frames_run = 0; return true; }
      crc_error_count++;
      if (++stm_bad_frames_run >= 50) {
        stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
        usbOut.println("STM link version unknown again: 50 bad frames in a row");
      }
      return false;
    }
  } else {
    serial1Drain();  // neither length: wait for the next frame
  }
  data_ready = 0;
  rtprof::resync();
  flushSerial1RxBounded();  // stray bytes: the next frame starts clean
  if (ok) { stm_bad_frames_run = 0; return true; }
  crc_error_count++;
  if (++stm_bad_frames_run >= 50 && stm_link_ver) {
    stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
    usbOut.println("STM link version unknown again: 50 bad frames in a row");
  }
  return false;
}

// A value as the STM32 gives it: float for 6 and 7, else an integer.
static void stmValueText(char* out, size_t n, uint32_t cid, uint32_t raw){
  const uint32_t id = cid & 0x7FFFFF;
  if (id == 6 || id == 7) { float f; memcpy(&f, &raw, 4); snprintf(out, n, "%.3f", f); }
  else snprintf(out, n, "%ld", (long)(int32_t)raw);
}

void stmLinkEvents(){
  stmset::Event e;
  while (stmLink.popEvent(&e)) {
    const stmset::Counters& c = stmLink.counters();
    char t[72], got[16], want[16];
    stmValueText(got, sizeof(got), e.cid, e.got);
    stmValueText(want, sizeof(want), e.cid, e.want);
    switch (e.kind) {
      case stmset::K_NO_REPLY:
        snprintf(t, sizeof(t), "no answer cmd %lu%s tries %u fails %lu", (unsigned long)(e.cid & 0x7FFFFF),
                 (e.cid & stmset::WRITE) ? "w" : "r", (unsigned)e.tries, (unsigned long)c.failures());
        break;
      case stmset::K_BAD_REPLY:
        snprintf(t, sizeof(t), "wrong answer cmd %luw got %s want %s try %u", (unsigned long)(e.cid & 0x7FFFFF), got, want, (unsigned)e.tries);
        break;
      case stmset::K_MISMATCH:
        snprintf(t, sizeof(t), "read-back cmd %lu got %s want %s: send all again", (unsigned long)(e.cid & 0x7FFFFF), got, want);
        break;
      case stmset::K_GAP:
        snprintf(t, sizeof(t), "packets back after %lu ms: send all again", (unsigned long)e.cid);
        break;
      case stmset::K_QUEUE_FULL:
        snprintf(t, sizeof(t), "queue full: cmd %lu dropped", (unsigned long)(e.cid & 0x7FFFFF));
        break;
      case stmset::K_STM_RESET:
        snprintf(t, sizeof(t), "STM32 reset (status bit 21): send all again");
        break;
      case stmset::K_REFUSED:
        snprintf(t, sizeof(t), "refused cmd %lu%s status %lu", (unsigned long)(e.cid & 0x7FFFFF),
                 (e.cid & stmset::WRITE) ? "w" : "r", (unsigned long)e.got);
        break;
      default:
        snprintf(t, sizeof(t), "kind %ld", (long)e.kind);
        break;
    }
    usbOut.printf("STMCMD %s\n", t);
    telemetryEvent(rec::EV_STM32_CMD, e.kind, (int32_t)e.cid, t);
  }
}


// setup(): the frame interrupt, the RC input readings cleared, the Rx STM32 start handshake.
void rxLinkBegin(){
  // Attach falling edge interrupt to STM_nDATA_READY pin
  TRACE(8);
  attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
  pwm_reset_readings();

  TRACE(9);
  stm32_boot_handshake();
  stmFastFrameGuard();  // a fast STM32 frame period left from before this reset: back to 10 ms first

  data_ready = 0;  // Clear any pending data ready flags that may have been set during boot
  stmLink.setSeq((uint8_t)esp_random());  // v2: an ESP32 restart cannot repeat the last seq
}

// 0.6.21: the position thresholds of two configurations differ.
static bool rxThresholdsDiffer(const DeviceConfig& a, const DeviceConfig& b){
  const double DeviceConfig::* f[24] = {
    &DeviceConfig::rx_weak_mm_0, &DeviceConfig::rx_weak_mm_1, &DeviceConfig::rx_weak_mm_2, &DeviceConfig::rx_weak_mm_3,
    &DeviceConfig::rx_weak_mm_4, &DeviceConfig::rx_weak_mm_5, &DeviceConfig::rx_weak_mm_6, &DeviceConfig::rx_weak_mm_7,
    &DeviceConfig::rx_lost_mm_0, &DeviceConfig::rx_lost_mm_1, &DeviceConfig::rx_lost_mm_2, &DeviceConfig::rx_lost_mm_3,
    &DeviceConfig::rx_lost_mm_4, &DeviceConfig::rx_lost_mm_5, &DeviceConfig::rx_lost_mm_6, &DeviceConfig::rx_lost_mm_7,
    &DeviceConfig::rx_regain_mm_0, &DeviceConfig::rx_regain_mm_1, &DeviceConfig::rx_regain_mm_2, &DeviceConfig::rx_regain_mm_3,
    &DeviceConfig::rx_regain_mm_4, &DeviceConfig::rx_regain_mm_5, &DeviceConfig::rx_regain_mm_6, &DeviceConfig::rx_regain_mm_7};
  for (auto m : f) if (a.*m != b.*m) return true;
  return false;
}

// applyDeviceConfig(): DSP settings: a change goes to the STM32 at once (a set, or a config.json edit on the USB drive)
void dspSettingsCheck(const DeviceConfig& cfg){
  static bool dsp_known = false;
  static DeviceConfig dsp_last;
  const bool dsp_changed = !dsp_known || dsp_last.dsp_frame_period_us != cfg.dsp_frame_period_us
      || dsp_last.enable_sensor_2 != cfg.enable_sensor_2 || memcmp(dsp_last.dsp_bin_hz, cfg.dsp_bin_hz, sizeof(cfg.dsp_bin_hz))
      || dsp_last.dsp_xy_mask != cfg.dsp_xy_mask || dsp_last.dsp_slow_adc != cfg.dsp_slow_adc
      || dsp_last.dsp_pga != cfg.dsp_pga
      || dsp_last.dsp_sweep_bins != cfg.dsp_sweep_bins || dsp_last.dt_channels != cfg.dt_channels
      || dsp_last.rx_regain_wait_ms != cfg.rx_regain_wait_ms || dsp_last.dt_spread_limit_deg != cfg.dt_spread_limit_deg
      || rxThresholdsDiffer(dsp_last, cfg);
  if (dsp_known && dsp_last.dsp_pga != cfg.dsp_pga) pga_ram_on = false;  // a setting change ends the bench value
  if (dsp_known && memcmp(dsp_last.dsp_bin_hz, cfg.dsp_bin_hz, sizeof(cfg.dsp_bin_hz))) bin_ram_mask = 0;
  if (dsp_known && dsp_changed) { dsp_apply_reason = "setting change"; dspApply(); }  // the first call (boot) leaves it to stmV2Config
  dsp_last = cfg;
  dsp_known = true;
}

// The frame part of loop(): rxFrameReady() reads the frames, the vehicle control step runs on a new one, then
// rxFrameDone() takes the time of the part.
static uint32_t loop_min_us = 0xFFFFFFFF;
static uint32_t loop_max_us = 0;
#ifdef DIAG_SOLVE_FEED
extern volatile bool g_diagSolveFeed;
extern volatile uint32_t g_diagSolves;
uint32_t diag_steps = 0, diag_fmax_us = 0;   // diagnostic build: control steps and the longest frame part in each HB second
#endif
static uint32_t loop_exec_count = 0;
static bool frame_part_ran = false;      // this pass read frames: rxFrameDone() counts its time
static uint32_t frame_part_start_us = 0;

bool rxFrameReady(){
  stage(ST_PACKET);
  mark(SEC_PACKET);
  last_state.drive_mode = (uint8_t)drive_mode_status;
  last_state.ap_state = telemetryApByte();
  frame_part_ran = false;
  if (stm_raw) { stmRawService(); return false; }  // bench: STM32 ROM loader link, no packets
  if (stm_link_ver == 2 && stm_v2_resync && data_ready && Serial1.available() != stm_v2_len) {
    serial1Drain();  // resync: wait for an edge with exactly one frame waiting
    data_ready = 0;
    rtprof::resync();
    uart_overrun_count++;
    return false;
  }
  if (!((stm_link_ver == 2 && Serial1.available() >= stm_v2_len && (!stm_v2_resync || data_ready)) ||
        (stm_link_ver != 2 && data_ready && (data_ready > 1 || Serial1.available() >= stmFrameLen()))))
    return false;
  frame_part_ran = true;
  frame_part_start_us = micros();
  // Overrun detection (v1 and unknown link): ISR fired more than once, or extra bytes are buffered from a
  // previous cycle.  Drain and skip this cycle; the next interrupt will bring a
  // fresh, complete packet that is guaranteed to be aligned. Layout 2 keeps late frames (see stm_v2_backlog).
  if(stm_link_ver != 2 && (data_ready > 1 || (stm_link_ver && Serial1.available() > stmFrameLen()))){
    serial1Drain();  // drain stale bytes first
    data_ready = 0;                             // then reset (ISR during drain → clean next cycle)
    rtprof::resync();
    uart_overrun_count++;
    if (++stm_bad_frames_run >= 50 && stm_link_ver) {  // the other version sends: learn it again
      stm_link_ver = 0; stm_v2_cfg_sent = false; stmDec.reset();
      usbOut.println("STM link version unknown again: 50 bad frames in a row");
    }
    // Fall through to the rest of loop() without touching stmPacket.
    return false;
  }
  // Normal path: exactly one interrupt fired and one full packet is ready.
  // Layout 2: when older frames wait (the loop fell behind, as at 2.35 ms frames, where one pass with control takes
  // about 3 ms), each older frame is read and decoded here without the control step, so the receive buffer never
  // overflows and no frame is lost; the control step then runs once, on the newest frame (0.5.85).
  if (stm_link_ver == 2 && !stm_v2_resync) {
    for (int k = 0; k < 16 && Serial1.available() >= 2 * stm_v2_len && !raw_mode && stm_rs_phase != RS_RESTART; k++) {
      if (!stmReadFrame()) break;
      stm_catchup_frames++;
      uart_skip_catchup++;
    }
  }
  // 0.5.96: control at most at CONTROL_PERIOD_US (100 Hz). At shorter reading times the frames between are decoded
  // only (no frame is lost; the log and the sweep record get every frame), so the drive part costs the same at every
  // reading time (2.35 ms frames: control on about every 4th frame).
  const bool decode_only = stm_link_ver == 2 && stmFramePeriodS() < CONTROL_PERIOD_US * 0.9e-6 &&
                           (uint32_t)(micros() - control_last_us) < CONTROL_PERIOD_US - 500;
  if (decode_only) {
    if (Serial1.available() >= stm_v2_len && stmReadFrame()) { stm_catchup_frames++; stmTxBlockOnly(); }
    return false;
  }
  if ((stm_link_ver == 2 && Serial1.available() < stm_v2_len) || !stmReadFrame()) {
    return false;  // a bad frame: counted in stmReadFrame (crc_error_count)
  }
  control_last_us = micros();
#ifdef DIAG_STEPTIME
  steptime_onStep(stmDec.lastSeq());  // diag: control step timing (src/diag_steptime.cpp)
#endif
  // 0.5.122: the frame to the Rx STM32 goes right after the frame read, before the control step. The Rx STM32 reads
  // its receive ring at each frame tick; sent at the end of the control step (up to about 2.5 ms later), a 16 B frame
  // could cross the next tick and arrive in two windows (bad CRC and length counts).
  espPacketFifo_sendCmd();  // one Rx UART frame: control block and up to one command

  static bool first_packet = true;
  if (first_packet) {  // startup time to control: the first valid STM32 packet
    first_packet = false;
    boot_first_packet_ms = millis();
    usbOut.printf("FIRST_PACKET ms=%lu\n", (unsigned long)boot_first_packet_ms);
  }
  pass_kind = 2;  // diag_cpu: this pass runs the control step
  return true;
}

void rxFrameDone(){
  if (!frame_part_ran) return;
  uint32_t loop_elapsed_us = micros() - frame_part_start_us;
  if(loop_elapsed_us < loop_min_us) loop_min_us = loop_elapsed_us;
  if(loop_elapsed_us > loop_max_us) loop_max_us = loop_elapsed_us;
  if(loop_elapsed_us > telem_loop_max_us) telem_loop_max_us = loop_elapsed_us;
#ifdef DIAG_SOLVE_FEED
  if(loop_elapsed_us > diag_fmax_us) diag_fmax_us = loop_elapsed_us;
#endif
  loop_exec_count++;
}

// After the frame part: Rx STM32 answers, commands and the link watch.
void rxLinkService(){
  mark(SEC_STMCMD);
  periodAckService();  // 0.5.104: the "set dsp_frame_period_us" answer after the Rx STM32 answer
  binPollService();    // 0.5.104: the bin frequency in effect after "bin"
  dtPollService();     // 0.6.12: the Tx channels the Rx STM32 decodes now (CID 50 read)
  if (loop_svc_due) {
    stmRecoveryService();  // STM32 silent after packets: HELLO/BOOT again
    stmLinkEvents();       // STM32 command failures, read-back differences, packet gaps: event 18 and USB
  }
}
