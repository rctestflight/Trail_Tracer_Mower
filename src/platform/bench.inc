// Bench commands on USB serial: tests, raw streams, captures, the flash test.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.


// Bench "usbload <bytes_per_s> <seconds>": paced 100-byte lines on USB, to measure USB output stops
// (lib/usb_guard) against the output rate. The start and end lines carry the guard counters.
uint32_t usbload_bps = 0, usbload_t0 = 0, usbload_end = 0, usbload_seq = 0, usbload_skip = 0;
uint64_t usbload_sent = 0;
void usbLoadService(){
  if (!usbload_bps) return;
  const uint32_t now = millis();
  if ((int32_t)(now - usbload_end) >= 0) {
    usbload_bps = 0;
    usbOut.printf("BENCH usbload end lines=%lu skipped=%lu dropped=%lu stalls=%lu reconnects=%lu tx_events=%lu\n",
                  (unsigned long)usbload_seq, (unsigned long)usbload_skip, (unsigned long)usbOut.dropped,
                  (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts(), (unsigned long)usb_guard::tx_events());
    return;
  }
  const uint64_t due = (uint64_t)usbload_bps * (now - usbload_t0) / 1000;
  for (int i = 0; i < 20 && usbload_sent + 100 <= due; i++) {
    usbload_sent += 100;
    if (usbOut.room() < 200) { usbload_skip++; continue; }  // the queue is full: the line is counted, not sent
    char b[101];
    int n = snprintf(b, sizeof(b), "U %08lu %010lu ", (unsigned long)++usbload_seq, (unsigned long)now);
    while (n < 99) b[n++] = '.';
    b[99] = '\n'; b[100] = 0;
    usbOut.write((const uint8_t*)b, 100);
  }
}
void stmStreamFrame(const uint8_t* f){
  static const char hx[] = "0123456789abcdef";
  static const uint8_t pos[] = {1, 14, 15, 16, 17, 18, 19, 38, 39, 40, 41, 42, 43, 80, 81, 82, 83};
  char b[2 + 2 * sizeof(pos) + 4];
  int m = 0;
  b[m++] = 'F';
  for (uint8_t p : pos) { b[m++] = hx[f[p] >> 4]; b[m++] = hx[f[p] & 15]; }
  b[m++] = ' ';
  b[m++] = (char)('0' + ((stmDec.status_bits >> 2) & 3));
  b[m++] = '\n';
  if (usbOut.room() < (size_t)m + 64) { stm_stream_drop++; return; }
  usbOut.write((const uint8_t*)b, m);
  stm_stream_n++;
}
// Bench (noise floor, the Rx as a spectrum analyser): 'stm acc <n>' sums the coil values of every bin, both
// lanes and both coils over n frames on the ESP32 (no frame lost to the USB pace), then prints per channel
// the rms, the mean of |v| and the largest |v| in int24 LSB (magnitude 2^-24). mean/rms: 0.886 for noise
// (Rayleigh), near 1 for a steady tone; max/rms shows impulses.
uint32_t stm_acc_want = 0, stm_acc_n = 0, stm_acc_ovl = 0, stm_acc_lost = 0;
uint8_t stm_acc_seq = 0;
double stm_acc_sq[WFRX_RESULTS][WFRX_BINS][2], stm_acc_abs[WFRX_RESULTS][WFRX_BINS][2];
uint32_t stm_acc_max[WFRX_RESULTS][WFRX_BINS][2];
// Bench (coil check, live): 'stm coils <ms>' prints the mean |v| of each coil of both lanes, bin 0, over each
// <ms> period (0 = off), in int24 LSB (magnitude 2^-24): STMCOIL ms=<millis> n=<frames> s1c1 s1c2 s2c1 s2c2.
// 0.5.48 (telemetry program request): also all 8 bins, STMCOIL8 ms= n= hz=<f1>,..,<f8> <32 values> (bin 0
// s1c1 s1c2 s2c1 s2c2, then bin 1 ... bin 7); a sensor that no lane reads gives 0. hz: the default bin frequencies,
// updated from each CID 20 answer (0 = bin off). 0.5.49: each STMCOIL8 value carries a sign: sign(mean v) x mean |v|,
// with v the signed int24 coil value (the STM32 sign: its quadrant state per coil). |value| stays mean |v| (for dBFS).
uint32_t stm_coil_ms = 0, stm_coil_t0 = 0, stm_coil_n = 0;
double stm_coil_sum[2][WFRX_BINS][2];   // [sensor][bin][coil] sum of |v|
double stm_coil_ssum[2][WFRX_BINS][2];  // sum of signed v
float stm_bin_hz[WFRX_BINS] = {53333.3f, 55315.5f, 51323.2f, 57347.7f, 49344.6f, 62500.0f, 46875.0f, 0.0f};
void stmCoilFrame(){
  bool done[2] = {false, false};
  for (unsigned k = 0; k < WFRX_RESULTS; k++) {
    const unsigned sn = (stmDec.lanes >> k) & 1u;  // the sensor this lane reads
    if (done[sn]) continue;  // both lanes on one sensor: lane 1 counts
    done[sn] = true;
    for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
      const int32_t v1 = wfrx_int24_get(stmDec.coil[k][bn].v1), v2 = wfrx_int24_get(stmDec.coil[k][bn].v2);
      stm_coil_sum[sn][bn][0] += v1 < 0 ? -v1 : v1;
      stm_coil_sum[sn][bn][1] += v2 < 0 ? -v2 : v2;
      stm_coil_ssum[sn][bn][0] += v1;
      stm_coil_ssum[sn][bn][1] += v2;
    }
  }
  stm_coil_n++;
  const uint32_t now = millis();
  if (now - stm_coil_t0 < stm_coil_ms) return;
  const double n = (double)stm_coil_n;
  usbOut.printf("STMCOIL ms=%lu n=%lu %.0f %.0f %.0f %.0f\n", (unsigned long)now, (unsigned long)stm_coil_n,
                stm_coil_sum[0][0][0] / n, stm_coil_sum[0][0][1] / n, stm_coil_sum[1][0][0] / n, stm_coil_sum[1][0][1] / n);
  char line[480];
  int m = snprintf(line, sizeof(line), "STMCOIL8 ms=%lu n=%lu hz=", (unsigned long)now, (unsigned long)stm_coil_n);
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) m += snprintf(line + m, sizeof(line) - m, bn ? ",%.1f" : "%.1f", stm_bin_hz[bn]);
  for (unsigned bn = 0; bn < WFRX_BINS; bn++)
    for (unsigned sn = 0; sn < 2; sn++)
      for (unsigned c = 0; c < 2; c++)
        m += snprintf(line + m, sizeof(line) - m, " %.0f", (stm_coil_ssum[sn][bn][c] < 0 ? -1.0 : 1.0) * stm_coil_sum[sn][bn][c] / n);
  usbOut.printf("%s\n", line);
  memset(stm_coil_sum, 0, sizeof(stm_coil_sum));
  memset(stm_coil_ssum, 0, sizeof(stm_coil_ssum));
  stm_coil_n = 0;
  stm_coil_t0 = now;
}
// Channel scan: with the Tx off, the user drives in
// manual mode over the full throttle range and the ESP32 ranks the bins by interference. Per frame and bin: level
// I = sqrt(S1^2 + S2^2) of each lane (int24 S units), the larger lane counts; sign and bin status are ignored. One
// histogram per bin in 1.5 dB steps (heap, 3 kB, no samples). 'scan start' / 'scan stop' / 'scan' (report) /
// 'scan wire <S>' (wire level at the farthest working distance; default the lost threshold 1/1200 x 2^24 = 13981).
// Rank by p99, then p99.9, then the median; margin = 20 log10(S_wire / p99): >= 20 dB clean, 10-20 usable, < 10 reject.
// Candidates are the Tx channels (bins 0-4); bins 5-7 are information only. Valid for the frame period of the scan.
#define SCAN_STEPS 96   // 1.5 dB steps from I = 1 (0 dB) to 144 dB
uint32_t (*scan_hist)[SCAN_STEPS] = nullptr;  // [WFRX_BINS][SCAN_STEPS]
bool scan_on = false;
uint32_t scan_n = 0, scan_t0 = 0;
float scan_wire = 13981.0f;
float scan_max[WFRX_BINS], scan_max_drive[WFRX_BINS];
void scanFrame(){
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
    float lv = 0;
    for (unsigned k = 0; k < WFRX_RESULTS; k++) {
      const float a = (float)wfrx_int24_get(stmDec.coil[k][bn].v1), b = (float)wfrx_int24_get(stmDec.coil[k][bn].v2);
      const float l = sqrtf(a * a + b * b);
      if (l > lv) lv = l;
    }
    int s = lv < 1.0f ? 0 : (int)(20.0f * log10f(lv) / 1.5f);
    scan_hist[bn][s < 0 ? 0 : s >= SCAN_STEPS ? SCAN_STEPS - 1 : s]++;
    if (lv > scan_max[bn]) { scan_max[bn] = lv; scan_max_drive[bn] = odrive_drive_command; }
  }
  scan_n++;
}
float scanPct(unsigned bn, float q){  // level (upper edge of the step) that a fraction q of the frames does not exceed
  const uint32_t want = (uint32_t)ceilf(q * scan_n);
  uint32_t c = 0;
  for (int s = 0; s < SCAN_STEPS; s++) { c += scan_hist[bn][s]; if (c >= want && c) return powf(10.0f, (s + 1) * 1.5f / 20.0f); }
  return powf(10.0f, SCAN_STEPS * 1.5f / 20.0f);
}
void scanReport(){
  if (!scan_hist || !scan_n) { usbOut.println("SCAN no data: 'scan start' with the Tx off, then drive"); return; }
  static const float f_def[WFRX_BINS] = {53333.3f, 55315.5f, 51323.2f, 57347.7f, 49344.6f, 62500.0f, 46875.0f, 0.0f};
  usbOut.printf("SCAN n=%lu s=%lu wire=%.0f period: as set; levels in S units (int24); margin = 20 log10(wire / level)\n",
                (unsigned long)scan_n, (unsigned long)((millis() - scan_t0) / 1000), scan_wire);
  int best = -1;
  float best_p99 = 0, best_p999 = 0, best_med = 0;
  for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
    const float med = scanPct(bn, 0.5f), p99 = scanPct(bn, 0.99f), p999 = scanPct(bn, 0.999f);
    const float m99 = 20.0f * log10f(scan_wire / p99), mmed = 20.0f * log10f(scan_wire / med);
    usbOut.printf("SCAN bin %u %s%.1f Hz: median %.0f p99 %.0f p99.9 %.0f max %.0f (drive %.2f) margin p99 %.1f dB median %.1f dB %s\n",
                  bn, bn < 5 ? "CH" : "info ", f_def[bn], med, p99, p999, scan_max[bn], scan_max_drive[bn], m99, mmed,
                  m99 >= 20 ? "clean" : m99 >= 10 ? "usable" : "reject");
    if (bn < 5 && (best < 0 || p99 < best_p99 || (p99 == best_p99 && (p999 < best_p999 || (p999 == best_p999 && med < best_med))))) {
      best = (int)bn; best_p99 = p99; best_p999 = p999; best_med = med;
    }
  }
  usbOut.printf("SCAN recommend Tx CH %d (bin %d): p99 margin %.1f dB%s\n", best + 1, best, 20.0f * log10f(scan_wire / best_p99),
                scan_n < 3000 ? "; fewer than 3000 frames: drive longer over idle, part and full throttle and braking" : "");
  // BTx (beacon) transmitters: the candidates are bins 5 and 6 (62.5 and 46.875 kHz)
  const unsigned bb = scanPct(5, 0.99f) <= scanPct(6, 0.99f) ? 5 : 6;
  usbOut.printf("SCAN recommend BTx bin %u (%.1f Hz): p99 margin %.1f dB\n", bb, f_def[bb], 20.0f * log10f(scan_wire / scanPct(bb, 0.99f)));
}
void stmAccFrame(){
  if (stm_acc_n && (uint8_t)(stmDec.lastSeq() - stm_acc_seq) != 1) stm_acc_lost += (uint8_t)(stmDec.lastSeq() - stm_acc_seq - 1);
  stm_acc_seq = stmDec.lastSeq();
  if (stmDec.status_bits & ((1u << 6) | (1u << 11))) stm_acc_ovl++;
  for (unsigned k = 0; k < WFRX_RESULTS; k++)
    for (unsigned bn = 0; bn < WFRX_BINS; bn++) {
      const int32_t v[2] = {wfrx_int24_get(stmDec.coil[k][bn].v1), wfrx_int24_get(stmDec.coil[k][bn].v2)};
      for (int c = 0; c < 2; c++) {
        const uint32_t a = (uint32_t)(v[c] < 0 ? -v[c] : v[c]);
        stm_acc_sq[k][bn][c] += (double)a * a;
        stm_acc_abs[k][bn][c] += a;
        if (a > stm_acc_max[k][bn][c]) stm_acc_max[k][bn][c] = a;
      }
    }
  if (++stm_acc_n < stm_acc_want) return;
  stm_acc_want = 0;
  usbOut.printf("STMACC n=%lu lost=%lu ovl=%lu\n", (unsigned long)stm_acc_n, (unsigned long)stm_acc_lost, (unsigned long)stm_acc_ovl);
  for (unsigned k = 0; k < WFRX_RESULTS; k++)
    for (int c = 0; c < 2; c++) {
      char line[400];
      int m = snprintf(line, sizeof(line), "STMACC lane%u coil%d", k + 1, c + 1);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++)
        m += snprintf(line + m, sizeof(line) - m, " %.2f,%.2f,%lu", sqrt(stm_acc_sq[k][bn][c] / stm_acc_n), stm_acc_abs[k][bn][c] / stm_acc_n,
                      (unsigned long)stm_acc_max[k][bn][c]);
      usbOut.printf("%s\n", line);
    }
}
bool bench_timing_print = false;
// USB serial commands for the bench tests (one line each):
//   thru <rate_hz> <size_bytes> <count>   broadcast THRU frames      thru stop
//   timing on | timing off                1 s loop and ESP-NOW counters
//   stat                                  print the counters once
void benchPrintCounters(const char* tag, uint32_t loops, uint32_t min_us, uint32_t max_us){
  usbOut.printf("%s loops=%lu min_us=%lu max_us=%lu crc_err=%lu overrun=%lu espnow=%d rx=%lu pings=%lu missed=%lu pongs=%lu tx_ok=%lu tx_fail=%lu send_err=%lu\n",
    tag, (unsigned long)loops, (unsigned long)(loops ? min_us : 0), (unsigned long)max_us,
    (unsigned long)crc_error_count, (unsigned long)uart_overrun_count, (int)espnow_started,
    (unsigned long)espnow_rx_count, (unsigned long)espnow_ping_count, (unsigned long)espnow_ping_missed,
    (unsigned long)espnow_pong_count, (unsigned long)espnow_tx_ok, (unsigned long)espnow_tx_fail, (unsigned long)espnow_send_err);
}

// "send <mac> <hex>": one ESP-NOW frame, e.g. "send 01:23:45:67:89:ab 7f52535421" (RESTART "RST!").
void benchSend(const char* arg){
  unsigned m[6];
  char hex[2 * ESP_NOW_MAX_DATA_LEN + 1];
  if (sscanf(arg, "%x:%x:%x:%x:%x:%x %500s", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], hex) != 7) {
    usbOut.println("BENCH send: usage send <mac> <hex>");
    return;
  }
  if (!espnow_started) { usbOut.println("BENCH send: espnow off"); return; }
  uint8_t mac[6], frame[ESP_NOW_MAX_DATA_LEN];
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
  const size_t n = strlen(hex) / 2;
  if (n == 0 || n > sizeof(frame) || strlen(hex) % 2) { usbOut.println("BENCH send: bad hex"); return; }
  for (size_t i = 0; i < n; i++) { unsigned b; sscanf(hex + 2 * i, "%2x", &b); frame[i] = (uint8_t)b; }
  espnowEnsurePeer(mac);
  const esp_err_t r = esp_now_send(mac, frame, n);
  usbOut.printf("BENCH send len=%u err=%d\n", (unsigned)n, (int)r);
}

// Raw stream (0.5.73): the Rx STM32 UART passed through to USB, and the rate tests for it.
//   raw usb <s>          USB output as fast as it goes: 512 B blocks "RAWU", u32 sequence, 504 pattern bytes
//   raw pass <baud> <s>  the Rx STM32 UART at <baud> (up to 5000000, the ESP32-S2 UART maximum); every byte to USB unchanged
//   raw loop <baud> <s>  the UART at <baud> in internal loopback: the ESP32 sends a counter pattern and checks it
// Nothing else runs meanwhile (no Rx STM32 packets, no drive output, no radio, no log); stationary only. "RAW start"
// before and "RAW end ..." after (bytes in and out, bytes dropped when USB could not take them, pattern errors, kB/s).
// Then the UART goes back to 2 Mbaud and the Rx STM32 link starts again as after the USB bridge.
uint8_t raw_mode = 0;  // 0 off, 1 usb, 2 pass, 3 loop
uint32_t raw_baud = 0, raw_t0 = 0, raw_end_ms = 0, raw_seq = 0;
uint64_t raw_in = 0, raw_out = 0, raw_drop = 0, raw_err = 0;
uint8_t raw_expect = 0, raw_tx = 0;
bool raw_first = true;
// "stm raw" (Rx STM32 command 42, RAW_STREAM_INTERFACE.md section 4): the packets pass from the first A5 5A to the
// end packet A5 5B (14 B); bytes before the first sync are dropped (the baud switch), a lost boundary resyncs.
bool raw_stm = false;           // pass mode for an Rx STM32 stream (packet framing, stop at the end packet)
uint8_t raw_stm_state = 0;      // 0 wait for the CID 42 answer, 1 before the first sync, 2 in the stream
uint16_t raw_pkt_len = 40;      // 8 + 32 x coils
uint32_t raw_pkt_pos = 0, raw_pkts = 0, raw_resync = 0, raw_presync = 0, raw_cmd_ms = 0, raw_last_rx_ms = 0;
uint8_t raw_prev = 0;
bool raw_end_pkt = false;
uint32_t raw_first_ms = 0;  // millis() of the first stream byte
bool raw_usb_stop = false, raw_stop_sent = false;  // USB input stopped the stream; the stop byte went to the Rx STM32
uint32_t raw_stm_value = 0, raw_stm_sec = 0, raw_stm_baud = 0;
size_t raw_rxbuf = 0;  // the UART receive buffer of the stream in effect (RAW start line)
// Returns the receive buffer it got: rxbuf, or half of it down to 4 KB when the largest free heap block is short
// (Wi-Fi mode leaves about 27 kB free: a 16 KB buffer failed with no message and the stream gave 0 bytes). 0: none.
size_t rawUartBegin(uint32_t baud, size_t rxbuf){
  // The Rx STM32 ends a stream on any received byte. Serial1.end() releases the TX pin (GPIO17); a low level on
  // the floating line can read as a start bit. The pin is driven high between end() and begin(), as at power-on.
  Serial1.flush();
  Serial1.end();
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  const size_t blk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  while (rxbuf > 4096 && rxbuf + 4096 > blk) rxbuf /= 2;
  Serial1.setRxBufferSize(rxbuf);
  Serial1.begin(baud);
  if (!uart_is_driver_installed(UART_NUM_1)) return 0;
  Serial1.setRxFIFOFull(64);
  return rxbuf;
}
void rawStop(){
  const uint32_t ms = millis() - raw_t0;
  const uint8_t mode = raw_mode;
  raw_mode = 0;
  if (mode == 3) uart_set_loop_back(UART_NUM_1, false);
  if (mode >= 2) {
    if (!rawUartBegin(2000000, 2048)) usbOut.println("RAW error: the Rx STM32 UART did not start again (restart the board)");  // the link as in setup()
    stm_link_ver = 0;             // learn the link again, as after the USB bridge
    stm_v2_cfg_sent = false;
    stmDec.reset();
    serial1Drain();
    data_ready = 0;
  }
  const double s = ms / 1000.0;
  if (raw_stm) usbOut.printf("\nRAW stm packets=%lu resyncs=%lu presync=%lu end_packet=%d first_ms=%lu last_ms=%lu\n",
                             (unsigned long)raw_pkts, (unsigned long)raw_resync, (unsigned long)raw_presync, (int)raw_end_pkt,
                             (unsigned long)(raw_first_ms - raw_t0), (unsigned long)(raw_last_rx_ms - raw_t0));
  const bool was_stm = raw_stm;
  raw_stm = false;
  raw_stm_state = 0;
  stm_last_packet_ms = millis();  // the silence recovery counts from the stream end, not from the last frame before it
  const char* stop = raw_usb_stop ? "usb" : "time";
  raw_usb_stop = raw_stop_sent = false;
  usbOut.printf("\nRAW end mode=%s stop=%s baud=%lu ms=%lu in=%llu out=%llu dropped=%llu errors=%llu kBps_in=%.1f kBps_out=%.1f heap=%lu\n",
                mode == 1 ? "usb" : was_stm ? "stm" : mode == 2 ? "pass" : "loop", stop, (unsigned long)raw_baud, (unsigned long)ms,
                (unsigned long long)raw_in, (unsigned long long)raw_out, (unsigned long long)raw_drop, (unsigned long long)raw_err,
                s > 0 ? raw_in / 1000.0 / s : 0.0, s > 0 ? raw_out / 1000.0 / s : 0.0, (unsigned long)ESP.getFreeHeap());
}
// "stm raw <mask> <shift> <seconds> [baud]": Rx STM32 command 42 (idle mode first), then on its answer 1 the UART
// goes to the stream baud at once (the first packet comes 50 ms after the answer frame) and the stream passes to USB.
void stmRawStreamCommand(const char* arg){
  unsigned long mask = 0, shift = 0, sec = 0, baud = 4923077;
  const int n = sscanf(arg, "%lu %lu %lu %lu", &mask, &shift, &sec, &baud);
  int coils = 0;
  for (int i = 0; i < 4; i++) coils += (mask >> i) & 1;
  if (n < 3 || coils < 1 || coils > 4 || mask > 15 || shift > 9 || sec < 1 || sec > 600 || baud < 115200 || baud > 5000000) {
    usbOut.println("STM raw usage: stm raw <coil mask 1..15> <shift 0..9> <seconds 1..600> [baud, default 4923077]");
    return;
  }
  if (stm_link_ver != 2) { usbOut.println("STM raw refused: no v2 link to the Rx STM32"); return; }
  if (!vehicleStationary()) { usbOut.println("STM raw refused: the vehicle moves"); return; }
  if (raw_mode || raw_stm_state) { usbOut.println("STM raw refused: busy"); return; }
  raw_stm_value = (uint32_t)mask | ((uint32_t)shift << 8) | ((uint32_t)sec << 16);
  raw_stm_sec = sec;
  raw_stm_baud = baud;
  raw_pkt_len = (uint16_t)(8 + 32 * coils);
  raw_stm_state = 0xFE;  // waiting for the idle mode answer, then command 42
  stmLink.pushInt(STM_RUN_CMD | 0x80000000, 0);
  stmLink.push(42 | stmset::WRITE, raw_stm_value, 0);
  raw_cmd_ms = millis();
  usbOut.printf("STM raw: command 42 value=0x%08lx (packet %u B), stream at %lu baud\n", (unsigned long)raw_stm_value,
                (unsigned)raw_pkt_len, baud);
}
// One v2 answer (before stmLink.onAnswerV2). Answer 1 to command 42: the UART switches now; nothing more goes out.
void stmRawStreamOnAnswer(const stmv2::Answer& a){
  if (raw_stm_state != 0xFE || !a.present || a.cid != (42 | ESP_CID_WRITE)) return;
  if (a.status != RSP_OK || a.value != 1) {
    raw_stm_state = 0;
    usbOut.printf("STM raw refused by the Rx STM32: status %u value %ld\n", (unsigned)a.status, (long)(int32_t)a.value);
    if (stm_v2_autorun) { stmLink.pushInt(STM_RUN_CMD | 0x80000000, 1); stm_v2_run_ms = millis(); }
    return;
  }
  raw_in = raw_out = raw_drop = raw_err = 0;
  raw_pkts = raw_resync = raw_presync = 0;
  raw_pkt_pos = 0; raw_prev = 0; raw_end_pkt = false;
  raw_baud = raw_stm_baud;
  web::stop();  // the page server frees its heap for the stream buffer (it starts again after the stream)
  raw_rxbuf = rawUartBegin(raw_stm_baud, 16384);  // within the 50 ms before the first packet
  if (!raw_rxbuf) {
    raw_stm_state = 0;
    raw_mode = 2;  // rawStop restores the 2 Mbaud link
    raw_t0 = millis();
    usbOut.printf("RAW error: no memory for the UART buffer (largest free block %lu B)\n",
                  (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    rawStop();
    return;
  }
  raw_stm = true;
  raw_stm_state = 1;
  raw_mode = 2;
  raw_t0 = millis();
  raw_last_rx_ms = raw_t0;
  raw_end_ms = raw_t0 + raw_stm_sec * 1000 + 3000;  // the stream time, the end packet and a margin
  usbOut.printf("RAW start mode=stm baud=%lu s=%lu rxbuf=%u\n", (unsigned long)raw_stm_baud, (unsigned long)raw_stm_sec,
                (unsigned)raw_rxbuf);
}

void rawCommand(const char* arg){
  char m[8] = {0};
  unsigned long baud = 0, sec = 0;
  const int n = sscanf(arg, "%7s %lu %lu", m, &baud, &sec);
  const bool usb = !strcmp(m, "usb") && n >= 2, uart = (!strcmp(m, "pass") || !strcmp(m, "loop")) && n == 3;
  if (usb) { sec = baud; baud = 0; }
  if ((!usb && !uart) || sec < 1 || sec > 120 || (uart && (baud < 115200 || baud > 5000000))) {
    usbOut.println("RAW usage: raw usb <s> | raw pass <baud> <s> | raw loop <baud> <s> (baud up to 5000000, s 1..120)");
    return;
  }
  if (!vehicleStationary()) { usbOut.println("RAW refused: the vehicle moves"); return; }
  raw_in = raw_out = raw_drop = raw_err = 0;
  raw_seq = 0; raw_expect = 0; raw_tx = 0; raw_first = true;
  raw_baud = baud;
  usbOut.printf("RAW start mode=%s baud=%lu s=%lu\n", m, baud, sec);
  usbOut.drain(300);  // the start line out before the binary data
  if (uart) {
    web::stop();
    raw_rxbuf = rawUartBegin(baud, 16384);  // 16 KB: 33 ms at 5 Mbaud (less when the heap is short)
    if (!raw_rxbuf) {
      usbOut.println("RAW error: no memory for the UART buffer");
      raw_mode = 2;
      raw_t0 = millis();
      rawStop();
      return;
    }
    if (m[0] == 'l') uart_set_loop_back(UART_NUM_1, true);
  }
  raw_mode = usb ? 1 : m[0] == 'p' ? 2 : 3;
  raw_t0 = millis();
  raw_end_ms = raw_t0 + sec * 1000;
}
void rawService(){
  if ((int32_t)(millis() - raw_end_ms) >= 0) { rawStop(); return; }
  static uint8_t b[512];
  // Any USB input stops the stream early (the RAW end line then says stop=usb). An Rx STM32 stream gets one byte (it
  // ends its stream on any byte) and ends as at its time: at the end packet, or after 500 ms with no byte. The other
  // raw modes stop at once. No text goes out before the end lines: it would mix with the stream bytes.
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    raw_usb_stop = true;
    if (!raw_stm) { rawStop(); return; }
    if (raw_stm_state >= 1 && !raw_stop_sent) {
      const uint8_t z = 0;
      Serial1.write(&z, 1);
      raw_stop_sent = true;
      raw_end_ms = millis() + 2000;  // the end packet or 500 ms of silence comes first
    }
  }
  if (raw_mode == 1) {  // the USB transmit buffer holds less than a block: each block goes out in pieces
    static size_t off = 0;
    for (int i = 0; i < 256; i++) {
      if (off == 0) {
        memcpy(b, "RAWU", 4);
        memcpy(b + 4, &raw_seq, 4);
        for (int k = 8; k < (int)sizeof(b); k++) b[k] = (uint8_t)(raw_seq + k);
      }
      const int room = Serial.availableForWrite();
      if (room <= 0) break;
      const size_t n = (size_t)room < sizeof(b) - off ? (size_t)room : sizeof(b) - off;
      const size_t w = Serial.write(b + off, n);
      raw_out += w;
      off += w;
      if (off >= sizeof(b)) { off = 0; raw_seq++; }
      if (w < n) break;
    }
    return;
  }
  if (raw_mode == 3) {  // loopback: keep the transmitter busy with the counter pattern
    int room = Serial1.availableForWrite();
    if (room > (int)sizeof(b)) room = sizeof(b);
    for (int k = 0; k < room; k++) b[k] = raw_tx++;
    if (room > 0) Serial1.write(b, room);
  }
  int avail = Serial1.available();
  while (avail > 0) {
    const size_t k = Serial1.read(b, avail < (int)sizeof(b) ? (size_t)avail : sizeof(b));
    if (k == 0) break;
    raw_in += k;
    avail -= (int)k;
    if (raw_mode == 3) {
      for (size_t i = 0; i < k; i++) {
        if (raw_first) { raw_expect = b[i]; raw_first = false; }
        if (b[i] != raw_expect) { raw_err++; raw_expect = b[i]; }
        raw_expect++;
      }
      continue;
    }
    raw_last_rx_ms = millis();
    size_t from = 0, to = k;
    if (raw_stm) {  // framing: drop bytes before the first sync; stop after the end packet
      size_t i = 0;
      for (; i < k; i++) {
        const uint8_t c = b[i];
        if (raw_stm_state == 1) {  // before the first sync
          if (raw_prev == 0xA5 && c == 0x5A) {  // the first sync: the packet starts at its A5
            raw_stm_state = 2; raw_pkt_pos = 2; raw_pkts = 1; raw_first_ms = millis();
            if (i == 0) { const uint8_t a5 = 0xA5; raw_out += Serial.write(&a5, 1); from = 0; }  // its A5 came in the last read
            else from = i - 1;
          }
          else raw_presync++;
          raw_prev = c;
          continue;
        }
        raw_pkt_pos++;
        if (raw_pkt_pos == 2 && raw_prev == 0xA5) {  // the second byte of a packet
          if (c == 0x5A) raw_pkts++;
          else if (c == 0x5B) raw_end_pkt = true;
        }
        if (raw_pkt_pos == 1 && c != 0xA5) { raw_resync++; raw_pkt_pos = 0; }  // a lost boundary: wait for A5
        raw_prev = c;
        const uint32_t len = raw_end_pkt ? 14 : raw_pkt_len;
        if (raw_pkt_pos >= len) {
          raw_pkt_pos = 0;
          if (raw_end_pkt) { to = i + 1; break; }
        }
      }
      if (raw_stm_state == 1) { continue; }  // nothing to pass yet
    }
    const size_t n = to - from;
    const int room = Serial.availableForWrite();
    const size_t w = room <= 0 ? 0 : Serial.write(b + from, (size_t)room < n ? (size_t)room : n);
    raw_out += w;
    raw_drop += n - w;
    if (raw_stm && raw_end_pkt) { rawStop(); return; }
  }
  // An Rx STM32 stream with no byte for 500 ms after its start: over (interface step 5)
  if (raw_stm && raw_stm_state >= 1 && millis() - raw_last_rx_ms > 500) rawStop();
}

void stmRawService(){
  uint8_t b[64];
  int n = 0;
  while (n < (int)sizeof(b) && Serial1.available()) b[n++] = (uint8_t)Serial1.read();
  if (!n) return;
  char hex[2 * sizeof(b) + 1];
  for (int i = 0; i < n; i++) sprintf(hex + 2 * i, "%02x", b[i]);
  usbOut.printf("STM rx %s\n", hex);
}
void stmRawCommand(const char* arg){
  if (!strcmp(arg, "rom")) {
    if (!vehicleStationary()) { usbOut.println("STM refused: the vehicle moves"); return; }
    detachInterrupt(digitalPinToInterrupt(STM_nDATA_READY));
    Serial1.end();
    Serial1.begin(115200, SERIAL_8E1);
    serial1Drain();
    stm_raw = true;
    stm_rom_used = true;
    usbOut.println("STM rom link on: 115200 8E1");
  } else if (!strcmp(arg, "peek")) {
    stm_raw = true;  // the 2 Mbaud link as it is: print the bytes already in the buffer, send nothing
    usbOut.printf("STM peek: %d bytes buffered at 2 Mbaud\n", Serial1.available());
  } else if (!strcmp(arg, "off")) {
    Serial1.end();
    Serial1.begin(2000000);
    Serial1.setRxFIFOFull(64);  // a 128 B frame: the default 120 B leaves 8 B of margin
    stm_raw = false;
    data_ready = 0;
    attachInterrupt(digitalPinToInterrupt(STM_nDATA_READY), receiveStmUart, FALLING);
    usbOut.println("STM rom link off: 2 Mbaud packet link");
  } else if (!strncmp(arg, "tx ", 3)) {
    if (!stm_raw) { usbOut.println("STM tx refused: link off"); return; }
    const char* h = arg + 3;
    uint8_t b[300];
    size_t n = 0;
    while (h[0] && h[1] && n < sizeof(b)) { unsigned v; if (sscanf(h, "%2x", &v) != 1) break; b[n++] = (uint8_t)v; h += 2; }
    Serial1.write(b, n);
    Serial1.flush();
    usbOut.printf("STM tx %u\n", (unsigned)n);
  } else if (!strncmp(arg, "cmd ", 4) || !strncmp(arg, "read ", 5) || !strncmp(arg, "dup ", 4)) {
    // bench: "stm cmd <cid> <value> [index]" (a write; a value with '.' is a float), "stm read <cid> [index]",
    // "stm dup <cid> <value> [index]" (one write frame with its own seq, sent twice 3 frames apart, for the
    // STM32 repeat rule). The v2 answers print for 2 s (STM ans ...).
    const bool dup = arg[0] == 'd', w = arg[0] == 'c' || dup;
    const char* q = arg + (arg[0] == 'r' ? 5 : 4);
    char* e = nullptr;
    const long cid = strtol(q, &e, 10);
    if (cid < 1 || cid > 127) { usbOut.println("STM usage: stm cmd <cid> <value> [index] | stm read <cid> [index] | stm dup <cid> <value> [index]"); return; }
    uint32_t raw = 0;
    while (*e == ' ') e++;
    if (w) {
      char* e2 = nullptr;
      if (strpbrk(e, ".") && strpbrk(e, ".") < (strchr(e, ' ') ? strchr(e, ' ') : e + strlen(e))) { const float f = strtof(e, &e2); memcpy(&raw, &f, 4); }
      else raw = (uint32_t)strtol(e, &e2, 0);
      e = e2;
      while (*e == ' ') e++;
    }
    const uint8_t idx = (uint8_t)strtol(e, nullptr, 0);
    if (cid == 13 && w && raw <= 3) stm_v2_lanes = (uint8_t)raw;
    if (cid == 3 && w) stm_v2_save_ok = true;  // a bench save goes out (a user action), as after a centre calibration
    stm_ans_print_until = millis() + 2000;
    if (dup) {
      if (stm_link_ver != 2) { usbOut.println("STM dup: v2 link only"); return; }
      stmv2::encodeCmd((uint8_t)(stmLink.seq() + 128), (uint8_t)(cid | ESP_CID_WRITE), idx, raw, stm_dup_frame);
      stm_dup_step = 1;
      usbOut.printf("STM dup cid %ld index %u seq %u\n", cid, (unsigned)idx, (unsigned)stm_dup_frame[0]);
      return;
    }
    const bool ok = stmLink.push((uint32_t)cid | (w ? stmset::WRITE : 0), raw, idx);
    usbOut.printf("STM %s %ld index %u queued=%d\n", w ? "cmd" : "read", cid, (unsigned)idx, (int)ok);
  } else if (!strcmp(arg, "pos")) {
    // bench: the decoded STM32 values as the firmware uses them (v1 and v2)
    const __StmTxPacketStruct& k = stmPacket.items;
    usbOut.printf("STMPOS ver=%u ms=%lu sb=0x%08lx s1 %.1f %.1f %.1f s2 %.1f %.1f %.1f btx1 %.0f %.0f btx2 %.0f %.0f vb=%.2f t=%.1f chan=0x%02x\n",
      (unsigned)stm_link_ver, (unsigned long)millis(), (unsigned long)k.status_bits, k.s1_x, k.s1_y, k.s1_radius,
      k.s2_x, k.s2_y, k.s2_radius, k.s1_btx_radius[0], k.s1_btx_radius[1], k.s2_btx_radius[0], k.s2_btx_radius[1],
      k.vbatt, k.temperature, (unsigned)stmDec.lanes);
  } else if (!strncmp(arg, "coils ", 6)) {
    const long ms = atol(arg + 6);
    memset(stm_coil_sum, 0, sizeof(stm_coil_sum));
    memset(stm_coil_ssum, 0, sizeof(stm_coil_ssum));
    stm_coil_n = 0;
    stm_coil_t0 = millis();
    stm_coil_ms = ms <= 0 ? 0 : (uint32_t)(ms < 50 ? 50 : ms);
    usbOut.printf("STMCOIL period %lu ms (0 = off)\n", (unsigned long)stm_coil_ms);
  } else if (!strncmp(arg, "acc ", 4)) {
    memset(stm_acc_sq, 0, sizeof(stm_acc_sq));
    memset(stm_acc_abs, 0, sizeof(stm_acc_abs));
    memset(stm_acc_max, 0, sizeof(stm_acc_max));
    stm_acc_n = stm_acc_ovl = stm_acc_lost = 0;
    const long n = atol(arg + 4);
    stm_acc_want = (uint32_t)(n < 1 ? 1 : n);
    usbOut.printf("STMACC started n=%lu\n", (unsigned long)stm_acc_want);
  } else if (!strncmp(arg, "stream ", 7)) {
    const long s = atol(arg + 7);
    stm_stream_end = s > 0 ? millis() + (uint32_t)(s > 600 ? 600 : s) * 1000u : 0;
    stm_stream_n = 0;
    usbOut.printf("STM stream %s %ld s: one line a frame, F<hex of bytes 1, 14-19, 38-43, 80-83> <PGA bits>\n",
                  stm_stream_end ? "on" : "off", s);
  } else if (!strncmp(arg, "cap ", 4)) {
    const int n = atoi(arg + 4);
    if (stm_cap) { free(stm_cap); stm_cap = nullptr; }
    stm_cap_dump = false;
    stm_cap_n = stm_cap_out = 0;
    stm_cap_want = (uint16_t)(n < 1 ? 1 : n > 300 ? 300 : n);
    stm_cap = (uint8_t*)malloc((size_t)STM_FRAME_LEN * stm_cap_want);
    usbOut.printf("STM cap %u frames %s\n", (unsigned)stm_cap_want, stm_cap ? "started" : "failed: no memory");
  } else if (!strcmp(arg, "dump")) {
    usbOut.printf("STM dump %u frames%s\n", (unsigned)stm_cap_n, stm_cap ? "" : " (none)");
    stm_cap_dump = stm_cap != nullptr;
    stm_cap_out = 0;
  } else if (!strcmp(arg, "rtt")) {
    usbOut.printf("STMRTT");
    for (int i = 0; i < 16; i++) usbOut.printf(" %lu", (unsigned long)stm_rtt_hist[i]);
    usbOut.printf("\n");
  } else if (!strcmp(arg, "rtt0")) {
    memset(stm_rtt_hist, 0, sizeof(stm_rtt_hist));
    usbOut.println("STMRTT cleared");
  } else if (!strncmp(arg, "run ", 4)) {
    // bench: "stm run 1" = acquisition on (and kept on), "stm run 0" = idle mode (layout 2, app 2.0.6+)
    const int v = atoi(arg + 4) ? 1 : 0;
    stm_v2_autorun = v == 1;
    stmLink.pushInt(STM_RUN_CMD | 0x80000000, v);
    stm_ans_print_until = millis() + 2000;
    usbOut.printf("STM run %d%c", v, 10);
  } else if (!strcmp(arg, "ext")) {
    // bench: frame version 3 extension items (Rx STM32 2.0.81+): the last value and age of each item id and index
    usbOut.printf("STM ext proto=%u frame=%d B items=%lu bad=%lu%c", (unsigned)stmDec.proto, stm_v2_len,
                  (unsigned long)stmDec.ext_items, (unsigned long)stmDec.ext_bad, 10);
    const uint16_t now10 = (uint16_t)(millis() / 10u);
    for (unsigned id = 0; id < WFRX_EXT_IDS; id++)
      for (unsigned sl = 0; sl < 16; sl++) {
        if (!stmDec.ext_t10[id][sl]) continue;
        usbOut.printf("STM ext id=%u sensor=%u index=%u value=%d age_ms=%u%c", id + 1, sl >> 3, sl & 7u, (int)stmDec.ext_val[id][sl],  // sensor = index bit 4
                      (unsigned)((uint16_t)(now10 - stmDec.ext_t10[id][sl]) * 10u), 10);
      }
  } else if (!strcmp(arg, "polar")) {
    // bench: the polarity detector stream (Rx STM32 2.0.67+, CID 52 on) and its record (body 0x37)
    const uint8_t* b = stmDec.polar;
    usbOut.printf("STM polar on=%d frames=%lu samples=%lu sent=%lu dropped=%lu lost_flags=%lu last: n=%u det=%u lost=%u gaps=%u out=%u in=%u%c",
                  (int)stmDec.polar_on, (unsigned long)polar_frames, (unsigned long)polar_samples, (unsigned long)polar_sent,
                  (unsigned long)polar_dropped, (unsigned long)polar_lost_flags, b[0], b[1] & 7u, (b[1] >> 7) & 1u,
                  (unsigned)(b[2] | b[3] << 8), (unsigned)(b[4] | b[5] << 8), (unsigned)(b[6] | b[7] << 8), 10);
  } else if (!strcmp(arg, "bins")) {
    // bench: the coil magnitudes (or x/y in um for a bin in xy mode) of all 8 bins, both lanes, last frame
    for (unsigned k = 0; k < WFRX_RESULTS; k++) {
      usbOut.printf("STMBINS lane%u", k + 1);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++)
        usbOut.printf(" %ld,%ld", (long)wfrx_int24_get(stmDec.coil[k][bn].v1), (long)wfrx_int24_get(stmDec.coil[k][bn].v2));
      usbOut.printf("%c", 10);
    }
  } else if (!strcmp(arg, "v2")) {
    // bench: the decoder state, the learned STM32 values and the last pages
    usbOut.printf("STMV2 ver=%u frames=%lu crc_err=%lu ver_err=%lu seq_lost=%lu lanes=%u status=0x%08lx wire_bin=%d,%d lost=%d,%d a=%.6f,%.6f b=%.6f,%.6f\n",
      (unsigned)stm_link_ver, (unsigned long)stmDec.st.frames, (unsigned long)stmDec.st.crc_err, (unsigned long)stmDec.st.ver_err,
      (unsigned long)stmDec.st.seq_lost, (unsigned)stmDec.lanes, (unsigned long)stmDec.status_bits, stmDec.wireBin(0), stmDec.wireBin(1),
      (int)stmDec.lost[0], (int)stmDec.lost[1], stmDec.a[0], stmDec.a[1], stmDec.b[0], stmDec.b[1]);
    for (int sn = 0; sn < 2; sn++) {
      usbOut.printf("STMV2 cal s%d R=%.6f phi=%.6f gain", sn + 1, stmDec.cal.radius[sn], stmDec.cal.angle[sn]);
      for (unsigned bn = 0; bn < WFRX_BINS; bn++) usbOut.printf(" %.6f", stmDec.cal.gain[sn][bn]);
      usbOut.printf("\n");
    }
    usbOut.printf("STMV2 thr weak=%.6f lost=%.6f regained=%.6f wait=%lu ms\n", stmDec.cal.weak[0], stmDec.cal.lost[0], stmDec.cal.regained[0], (unsigned long)stmDec.cal.regain_ms);
    const PageMeas& m = stmDec.meas;
    usbOut.printf("STMV2 meas vbatt=%u vcharge=%u icharge=%d v5v=%u v3v3=%u temp=%d timer=%d code=0x%02x\n", m.vbatt_mv, m.vcharge_mv,
      m.icharge_ma, m.v5v_mv, m.vesp_3v3_mv, m.temperature_dc, m.charge_timer_s, m.charge_code);
    const PageHealth& h = stmDec.health;
    usbOut.printf("STMV2 health work_end=%u/%u x10us l2_wait_min=%u isr_worst=%u overruns=%u hk_skip=%u cmd_ok=%u cmd_crc_err=%u uart_err=%u rx_restart=%u bad_length=%u\n",
      h.work_end_last, h.work_end_worst, h.l2_wait_min, h.isr_worst, h.overrun_count, h.hk_skip_count, h.cmd_ok, h.cmd_crc_err,
      h.uart_err, h.rx_restart, h.bad_length);
  } else if (!strcmp(arg, "iaphold")) {
    if (stm_link_ver == 2 && stm_cal_phase == CAL_IDLE) { stm_cal_then_hold = true; stmCalStart(CAL_READ); }  // copy first
    else { if (stm_cal_valid) stmCalSetPending(true); stmIapHoldSet(); }
  } else if (!strncmp(arg, "raw ", 4)) {
    stmRawStreamCommand(arg + 4);
  } else if (!strncmp(arg, "restart", 7)) {
    stmRestartCommand(arg + 7);
  } else if (!strcmp(arg, "cal")) {
    usbOut.printf("STM cal copy %s, restore %s:", stm_cal_valid ? "valid" : "none", stmCalPending() ? "pending" : "off");
    for (int k = 0; k < STM_CAL_N; k++) { float f; memcpy(&f, &stm_cal_val[k], 4); usbOut.printf(" %.7g", f); }
    usbOut.printf("\n");
  } else if (!strcmp(arg, "cal read") || !strcmp(arg, "cal restore")) {
    // bench: the copy without an update ("cal read"), or the restore at once ("cal restore")
    if (stm_link_ver != 2 || stm_cal_phase != CAL_IDLE) usbOut.println("STM cal: busy, or no v2 link");
    else if (arg[4] == 'r' && arg[5] == 'e' && arg[6] == 'a') stmCalStart(CAL_READ);
    else if (!stm_cal_valid) usbOut.println("STM cal: no stored copy");
    else stmCalStart(CAL_WRITE);
  } else {
    usbOut.println("STM usage: stm rom | stm peek | stm tx <hex> | stm off | stm iaphold | stm restart clear|add <hex>|go | stm url <url> <size> <sha256> | stm status | stm cmd <cid> <value> [index] | stm read <cid> [index] | stm dup <cid> <value> [index] | stm pos | stm v2");
  }
}

uint32_t usb_rx_ms = 0;  // 0.5.104: the last USB input byte (EV_USB "no input")
void benchSerial(){
  static char line[1200];  // "ota w <offset> <hex of up to 512 bytes>" fits
  static uint16_t n = 0;
  if (Serial.available()) usb_rx_ms = millis();
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      if (n < sizeof(line) - 1) line[n++] = c;
      continue;
    }
    if (n == 0) continue;
    line[n] = 0;
    n = 0;
    // 0.6.14: no USB reconnect in the vehicle role. A host that holds the port open and reads nothing fills the
    // output; a reconnect under the open port could make the host fail to enumerate the board again and remove its
    // power (both chips reset). The vehicle role needs no USB: undrained output is dropped and counted
    // (usb_guard stalls).
    if (stmUrlBusy() && !updateAllowed(line)) { usbOut.printf("STM refused: an Rx STM32 update runs (%s)\n", line); continue; }
    {
      String l;
      if (updateCommand(line, false, l)) continue;  // stm url, stm status, get rx_stm32_version; ota url refused during stm url
    }
    unsigned long hz = 0, size = 0, count = 0;
    unsigned tm[6];
    const int thru_args = sscanf(line, "thru %lu %lu %lu %x:%x:%x:%x:%x:%x", &hz, &size, &count,
                                 &tm[0], &tm[1], &tm[2], &tm[3], &tm[4], &tm[5]);
    unsigned long lt = 0, lr = 0;
    if ((thru_args == 3 || thru_args == 9) && hz >= 1 && hz <= 1000 && count > 0) {
      if (!espnow_started) { usbOut.println("BENCH espnow off (set espnow_enabled in config.json)"); continue; }
      for (int i = 0; i < 6; i++) espnow_thru_mac[i] = thru_args == 9 ? (uint8_t)tm[i] : 0xFF;
      espnow_thru_size = constrain(size, 9UL, (unsigned long)ESP_NOW_MAX_DATA_LEN);
      espnow_thru_period_us = 1000000UL / hz;
      espnow_thru_left = count;
      espnow_thru_seq = 0;
      espnow_send_err = 0;
      espnow_thru_last_us = micros();
      espnow_thru_active = true;
      usbOut.printf("BENCH thru start rate_hz=%lu size=%lu count=%lu to=%02x:%02x:%02x:%02x:%02x:%02x\n", hz,
        (unsigned long)espnow_thru_size, count, espnow_thru_mac[0], espnow_thru_mac[1], espnow_thru_mac[2],
        espnow_thru_mac[3], espnow_thru_mac[4], espnow_thru_mac[5]);
    } else if (!strcmp(line, "loss") || (sscanf(line, "loss %lu %lu", &lt, &lr) == 2 && lt <= 100 && lr <= 100)) {
      if (strcmp(line, "loss")) { sim_loss_tx = (uint8_t)lt; sim_loss_rx = (uint8_t)lr; }
      usbOut.printf("BENCH loss tx=%u rx=%u dropped_tx=%lu dropped_rx=%lu\n", (unsigned)sim_loss_tx, (unsigned)sim_loss_rx,
        (unsigned long)sim_dropped_tx, (unsigned long)sim_dropped_rx);
    } else if (!strcmp(line, "thru stop")) {
      espnow_thru_active = false;
      usbOut.printf("BENCH thru stopped sent=%lu\n", (unsigned long)espnow_thru_seq);
    } else if (!strcmp(line, "tasks")) {  // bench: the FreeRTOS tasks by name (the framework has no task list): priority, least free stack
      static const char* const names[] = {"loopTask", "usbd", "wifi", "tiT", "esp_timer", "sys_evt", "arduino_events",
                                          "mdns", "Tmr Svc", "IDLE"};
      usbOut.printf("TASKS count=%u (named below: %u)\n", (unsigned)uxTaskGetNumberOfTasks(), (unsigned)(sizeof(names) / sizeof(names[0])));
      for (const char* nm : names) {
        TaskHandle_t h = xTaskGetHandle(nm);
        if (!h) { usbOut.printf("TASK %s none\n", nm); continue; }
        usbOut.printf("TASK %s prio=%u stack_free_min=%u B\n", nm, (unsigned)uxTaskPriorityGet(h), (unsigned)uxTaskGetStackHighWaterMark(h));
      }
    } else if (!strcmp(line, "nvs")) {  // bench: NVS entry counts; a write adds used entries (the proof of no repeating writes)
      nvs_stats_t ns;
      if (nvs_get_stats(NULL, &ns) == ESP_OK)
        usbOut.printf("NVS used=%u free=%u total=%u namespaces=%u\n", (unsigned)ns.used_entries, (unsigned)ns.free_entries,
                      (unsigned)ns.total_entries, (unsigned)ns.namespace_count);
      else usbOut.println("NVS stats failed");
    } else if (!strcmp(line, "moving on") || !strcmp(line, "moving off")) {  // bench: the stationary rule with no ODrive
      extern bool bench_moving;
      bench_moving = !strcmp(line, "moving on");
      usbOut.printf("BENCH moving %s: the vehicle counts as %s\n", bench_moving ? "on" : "off", bench_moving ? "moving" : "it is");
    } else if (!strcmp(line, "timing on") || !strcmp(line, "timing off")) {
      bench_timing_print = !strcmp(line, "timing on");
      usbOut.printf("BENCH timing %s\n", bench_timing_print ? "on" : "off");
    } else if (!strncmp(line, "ota ", 4)) {
      update_last_ms = millis();
      usb_ota::command(line + 4);
    } else if (!strncmp(line, "flashtest ", 10)) {
      flashTestCommand(line + 10);
    } else if (!strncmp(line, "log", 3)) {
      telemetryUsbCommand(line);
    } else if (!strncmp(line, "send ", 5)) {
      benchSend(line + 5);
    } else if ((!strcmp(line, "restart") || !strcmp(line, "dld")) && !vehicleStationary()) {
      usbOut.printf("%s refused: Stop the vehicle first\n", line);  // never a restart while the vehicle moves
      telemetryEvent(rec::EV_RESTART_REFUSED, strcmp(line, "dld") ? 1 : 2, 0, "USB restart refused: the vehicle moves");
    } else if (!strcmp(line, "restart")) {
      usbOut.println("BENCH restart");  // software reset: the telemetry rings survive it
      restartApp();
    } else if (!strcmp(line, "monitor")) {
      // A restart before "ota confirm" makes the bootloader return to the previous image.
      esp_ota_img_states_t st;
      const bool unconfirmed = esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK &&
                               st == ESP_OTA_IMG_PENDING_VERIFY;
      if (!vehicleStationary()) usbOut.println("MONITOR refused: the vehicle moves");
      else if (unconfirmed) usbOut.println("MONITOR refused: the new image waits for ota confirm");
      else if (update_mode) usbOut.println("MONITOR refused: update mode");
      else {
        usbOut.println("MONITOR restart into the monitor role (this boot only)");
        monitor_once = MONITOR_ONCE_MAGIC;
        restartApp();
      }
    } else if (!strcmp(line, "dld")) {
      usbOut.println("BENCH restart into ROM download mode");  // for esptool, with no 1200 bps touch
      restartIntoRom();
    } else if (!strcmp(line, "stack")) {  // bench: the least free loop() stack since the start (FreeRTOS high-water mark)
      usbOut.printf("STACK loop_free_min=%u of %u B\n", (unsigned)uxTaskGetStackHighWaterMark(nullptr),
                    (unsigned)CONFIG_ARDUINO_LOOP_STACK_SIZE);
    } else if (!strcmp(line, "rtcwdt test")) {  // bench: stop the RTC watchdog feed; the system resets in 8 s
      if (!vehicleStationary()) usbOut.println("rtcwdt test refused: the vehicle moves");
      else { rtc_wdt_test = true; usbOut.println("BENCH rtcwdt test: no feed from now; reset in 8 s"); }
    } else if (!strcmp(line, "crash")) {  // bench: a forced panic, to check the record after a crash
      if (!vehicleStationary()) usbOut.println("crash refused: the vehicle moves");
      else { usbOut.println("BENCH crash: abort() now"); usbOut.drain(200); abort(); }
    } else if (!strncmp(line, "stm ", 4)) {
      stmRawCommand(line + 4);
    } else if (!strncmp(line, "uart", 4) && (line[4] == 0 || line[4] == ' ')) {  // bench: the Rx UART frame
      uartCommand(line[4] ? line + 5 : "");
    } else if (!strcmp(line, "btn0 down") || !strcmp(line, "btn0 up")) {  // bench: the 0 button pressed or released
      btn0_sim = !strcmp(line, "btn0 down");
      usbOut.printf("BTN0 sim=%d ms=%lu\n", (int)btn0_sim, (unsigned long)millis());
    } else if (!strcmp(line, "btn0 window")) {  // bench: restart with the erase window open, as at power-on
      btn_window_once = BTN_WINDOW_MAGIC;
      usbOut.println("BTN0 window: restart; the erase window opens for 3 s");
      usbOut.drain(200);
      restartApp();
    } else if (!strcmp(line, "boot")) {
      usbOut.printf("BOOT rover-firmware %s reset=%d setup_ms=%lu first_packet_ms=%lu espnow_start_ms=%lu espnow=%d mac=%s board=%s\n",
        ROVER_FW_VERSION, (int)esp_reset_reason(), (unsigned long)boot_setup_ms, (unsigned long)boot_first_packet_ms,
        (unsigned long)boot_espnow_ms, (int)espnow_started, WiFi.macAddress().c_str(), RX_BOARD);
      if (panic_line[0]) usbOut.printf("%s\n", panic_line);  // the panic before this boot (src/panic_record.cpp)
    } else if (!strcmp(line, "verbose on") || !strcmp(line, "verbose off")) {
      usb_verbose = line[9] == 'n';  // this boot only
      usbOut.printf("verbose=%d%c", (int)usb_verbose, 10);
    } else if (!strcmp(line, "drv")) {
      // bench: the drive chain for the PC Start/Stop trace: mode, rover state, the ODrive commands (CAN)
      // The commanded values come before the motor locks (gyro, arm switch, CAN): a bench board with no
      // gyro shows what rover mode and Start/Stop command, with rc_active 0 (motors locked).
      usbOut.printf("DRV ms=%lu mode=%d rover_enabled=%d rover_active=%d ap=%d out_enabled=%d drive=%.3f rate_req=%.1f dps steer=%.3f rc_active=%d gyro_valid=%d arm=%d can=%d%c",
        (unsigned long)millis(), (int)drive_mode_status, (int)rover_enabled, (int)rover_active, rover_active ? (int)roverAp.state : -1,
        (int)odrive_output_enabled, odrive_drive_command, steering_rate_target_dps, odrive_steer_command, (int)odrive_rc_active,
        (int)steeringRateGyroValid(), digitalRead(MOTOR_ARM_SWITCH_PIN), (int)can_init_ok, 10);
    } else if (!strcmp(line, "scan start")) {
      // channel scan: Tx off, magnitude mode on all bins, the vehicle's frame period; drive in manual mode
      if (stm_link_ver != 2) { usbOut.println("SCAN needs the v2 STM32 link"); }
      else {
        if (!scan_hist) scan_hist = (uint32_t (*)[SCAN_STEPS])malloc(sizeof(uint32_t) * WFRX_BINS * SCAN_STEPS);
        if (!scan_hist) { usbOut.println("SCAN failed: no memory"); }
        else {
          memset(scan_hist, 0, sizeof(uint32_t) * WFRX_BINS * SCAN_STEPS);
          for (unsigned bn = 0; bn < WFRX_BINS; bn++) scan_max[bn] = scan_max_drive[bn] = 0;
          scan_n = 0; scan_t0 = millis(); scan_on = true;
          usbOut.println("SCAN started: Tx off; drive idle, part throttle, full throttle and braking for 3000 frames or more; 'scan' reports, 'scan stop' ends");
        }
      }
    } else if (!strcmp(line, "scan stop")) {
      scan_on = false;
      scanReport();
    } else if (!strcmp(line, "scan")) {
      scanReport();
    } else if (!strncmp(line, "scan wire ", 10)) {
      const float w = strtof(line + 10, nullptr);
      if (w > 0) scan_wire = w;
      usbOut.printf("SCAN wire level %.0f\n", scan_wire);
    } else if (!strcmp(line, "rt")) {
      // bench: real-time path timing since the last "rt" (lib/rt_prof): frame edge -> control -> CAN, loop passes
      static int64_t rt_t0 = 0;
      static uint32_t rt_f0 = 0;
      const int64_t now = esp_timer_get_time();
      const uint32_t frames = stmDec.st.frames;
      // the mean period since the last "rt"; on the first "rt" after boot the period in effect (0.6.0 printed 10000 there)
      const double eff = g_dsp_eff[DSP_EFF_PERIOD];
      const uint32_t per = (rt_t0 && frames > rt_f0) ? (uint32_t)((now - rt_t0) / (frames - rt_f0))
                                                     : (isfinite(eff) && eff > 0 ? (uint32_t)eff : 10000u);
      rt_t0 = now; rt_f0 = frames;
      static char b[1100];
      rtprof::report(b, sizeof(b), SEC_NAMES, SEC_N, per);
      static uint32_t rt_sent0 = 0, rt_writes0 = 0, rt_retries0 = 0;
      const stmset::Counters& lc = stmLink.counters();
      usbOut.printf("%s | waits=%lu catchup=%lu usb_stalls=%lu usb_reconnects=%lu stm_cmds=%lu stm_writes=%lu stm_retries=%lu%c", b,
                    (unsigned long)loop_waits, (unsigned long)stm_catchup_frames,
                    (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts(), (unsigned long)(lc.sent - rt_sent0),
                    (unsigned long)(lc.writes - rt_writes0), (unsigned long)(lc.retries - rt_retries0), 10);
      rt_sent0 = lc.sent; rt_writes0 = lc.writes; rt_retries0 = lc.retries;
      stm_catchup_frames = 0;
      loop_waits = 0;
    } else if (!strncmp(line, "raw ", 4)) {
      rawCommand(line + 4);
    } else if (!strncmp(line, "usbload ", 8)) {
      unsigned long bps = 0, sec = 0;
      if (sscanf(line + 8, "%lu %lu", &bps, &sec) == 2 && bps <= 200000 && sec <= 7200) {
        usbload_bps = bps; usbload_t0 = millis(); usbload_end = usbload_t0 + sec * 1000; usbload_sent = 0;
        usbload_seq = 0; usbload_skip = 0;
        usbOut.printf("BENCH usbload start bps=%lu s=%lu dropped=%lu stalls=%lu reconnects=%lu tx_events=%lu\n", bps, sec,
                      (unsigned long)usbOut.dropped, (unsigned long)usb_guard::stalls(), (unsigned long)usb_guard::restarts(),
                      (unsigned long)usb_guard::tx_events());
      } else usbOut.println("BENCH usbload: usage usbload <bytes_per_s 0..200000> <seconds 0..7200>");
    } else if (!strncmp(line, "radio", 5)) {
      // bench: "radio slot <us>" | "radio slot off" | "radio": slot and send-done histogram (then cleared)
      if (!strncmp(line, "radio slot ", 11)) radio_slot_us = strcmp(line + 11, "off") ? atoi(line + 11) : -1;
      usbOut.printf("RADIO slot=%ld len=%lu period_us=%lu hist=", (long)radio_slot_us, (unsigned long)RADIO_SLOT_LEN_US,
                    (unsigned long)(stmFramePeriodS() * 1e6));
      for (int i = 0; i < 11; i++) { usbOut.printf("%lu%c", (unsigned long)radio_tx_hist[i], i < 10 ? ',' : 10); radio_tx_hist[i] = 0; }
    } else if (!strcmp(line, "rt bench")) {
      const double mhz = ESP.getCpuFreqMHz();
      usbOut.printf("RTBENCH live n=%lu read mean=%.1f max=%.1f us decode mean=%.1f max=%.1f us (answer mean=%.1f, after mean=%.1f)\n",
                    (unsigned long)rt_bench_n, rt_bench_n ? rt_read_cyc / mhz / rt_bench_n : 0.0, rt_read_max / mhz,
                    rt_bench_n ? rt_dec_cyc / mhz / rt_bench_n : 0.0, rt_dec_max / mhz,
                    rt_bench_n ? rt_ans_cyc / mhz / rt_bench_n : 0.0, rt_bench_n ? rt_post_cyc / mhz / rt_bench_n : 0.0);
      rt_read_cyc = rt_dec_cyc = rt_ans_cyc = rt_post_cyc = 0; rt_read_max = rt_dec_max = 0; rt_bench_n = 0;
      if (rt_last_ok) {
        static stmv2::Decoder d2;
        d2 = stmDec;  // same calibration and lanes; its counters do not touch the live decoder
        static __StmTxPacketStruct it;
        static stmv2::Answer an;
        uint32_t mx = 0; uint64_t sum = 0;
        for (int i = 0; i < 1000; i++) {
          const uint32_t a = ESP.getCycleCount();
          d2.decode(rt_last_frame, it, an);
          const uint32_t b = ESP.getCycleCount();
          sum += b - a; if (b - a > mx) mx = b - a;
        }
        uint32_t a = ESP.getCycleCount();
        volatile uint16_t cr = 0;
        for (int i = 0; i < 1000; i++) cr ^= stmv2::crc16(rt_last_frame, STM_FRAME_LEN - 2);
        const double crc_us = (ESP.getCycleCount() - a) / mhz / 1000.0;
        usbOut.printf("RTBENCH stored decode x1000 mean=%.1f max=%.1f us crc16(126 B)=%.1f us\n", sum / mhz / 1000.0, mx / mhz, crc_us);
      }
      volatile float x = 1.2345f, y = 0.789f, z = 0;
      volatile double dx = 1.2345, dy = 0.789, dz = 0;
      struct { const char* name; int op; } ops[] = {{"fmul", 0}, {"fdiv", 1}, {"sqrtf", 2}, {"atan2f", 3}, {"powf", 4}, {"sinf", 5},
                                                    {"dmul", 6}, {"ddiv", 7}, {"sqrt", 8}};
      char lb[300]; int m = snprintf(lb, sizeof(lb), "RTBENCH op ns:");
      for (auto& o : ops) {
        const uint32_t a = ESP.getCycleCount();
        for (int i = 0; i < 1000; i++) {
          switch (o.op) {
            case 0: z = x * y; break;
            case 1: z = x / y; break;
            case 2: z = sqrtf(x); break;
            case 3: z = atan2f(x, y); break;
            case 4: z = powf(x, -0.330478f); break;
            case 5: z = sinf(x); break;
            case 6: dz = dx * dy; break;
            case 7: dz = dx / dy; break;
            default: dz = sqrt(dx); break;
          }
        }
        m += snprintf(lb + m, sizeof(lb) - m, " %s=%.0f", o.name, (ESP.getCycleCount() - a) * 1000.0 / mhz / 1000.0);
      }
      (void)z; (void)dz;
      usbOut.printf("%s (loop overhead included)\n", lb);
    } else if (!strcmp(line, "cpu")) {
      // bench: CPU share per FreeRTOS task and per loop part since the last "cpu" (lib/cpu_prof)
      char b[900];
      cpuprof::report(b, sizeof(b), SEC_NAMES, SEC_N);
      usbOut.printf("%s%c", b, 10);
    } else if (!strcmp(line, "stat")) {
      benchPrintCounters("BENCH stat", 0, 0, 0);
      const stmset::Counters& c = stmLink.counters();
      usbOut.printf("STMCMD stat sent=%lu answers=%lu retries=%lu no_answer=%lu wrong_answer=%lu checks=%lu read_back_bad=%lu resends=%lu gaps=%lu queue_full=%lu events_lost=%lu iap_boots=%lu iap_fails=%lu\n",
        (unsigned long)c.sent, (unsigned long)c.replies, (unsigned long)c.retries, (unsigned long)c.no_reply,
        (unsigned long)c.bad_reply, (unsigned long)c.checks, (unsigned long)c.mismatches, (unsigned long)c.resyncs,
        (unsigned long)c.gaps, (unsigned long)c.queue_full, (unsigned long)c.events_lost,
        (unsigned long)stm_iap_boots, (unsigned long)stm_iap_fails);
      usbOut.printf("STMLINK ver=%u frames=%lu crc_err=%lu ver_err=%lu seq_lost=%lu refused=%lu stm_resets=%lu\n",
        (unsigned)stm_link_ver, (unsigned long)stmDec.st.frames, (unsigned long)stmDec.st.crc_err,
        (unsigned long)stmDec.st.ver_err, (unsigned long)stmDec.st.seq_lost, (unsigned long)c.refused, (unsigned long)c.stm_resets);
    } else if (!strcmp(line, "wifi") || !strncmp(line, "wifi ", 5)) {
      wifiUsbCommand(line);  // never echoes the line: it can hold the password
      memset(line, 0, sizeof(line));  // and the line buffer keeps no copy of it
    } else if (!strcmp(line, "list")) {
      String kv;
      int n_keys = 0;
      for (; msc_fat_key_at(&g_deviceConfig, n_keys, &kv); n_keys++) {
        if (usbOut.room() < 200) usbOut.drain(100);  // 84 lines overrun the USB text queue: let it empty
        usbOut.printf("CFG 0 %s\n", kv.c_str());
      }
      usbOut.printf("CFG 0 end %d def %d\n", n_keys, SETTINGS_DEF_VERSION);  // key count; settings_def.json version
    } else if (!strcmp(line, "list saved")) {
      // 0.6.15: each setting with the value it takes at the next start (stored, else the default)
      msc_fat_saved_config(&g_saved_config);
      String kv;
      int n_keys = 0;
      for (; msc_fat_key_pair_at(&g_deviceConfig, &g_saved_config, n_keys, &kv); n_keys++) {
        if (usbOut.room() < 200) usbOut.drain(100);
        usbOut.printf("CFG 0 %s\n", kv.c_str());
      }
      usbOut.printf("CFG 0 end %d def %d\n", n_keys, SETTINGS_DEF_VERSION);
    } else if (!strcmp(line, "pwm") || !strncmp(line, "pwm ", 4)) {
      // 0.5.104, bench pulse check (PWM OUT1 wired to PWM IN1): "pwm <us>" holds OUT1 at <us> (900-2100) while not
      // driving and clears the IN1 counts; "pwm read" prints the IN1 pulses since the last clear and clears them;
      // "pwm off" ends the hold. OUT1 is LEDC channel 1 (50 Hz, 14 bit: 1.22 us steps); IN1 is RMT channel 0 (0.1 us).
      const char* a = line + 3;
      while (*a == ' ') a++;
      pwm_stats_t st;
      if (!strcmp(a, "off")) {
        pwm_test_us = 0;
        usbOut.println("PWM off");
      } else if (!strcmp(a, "read") || !*a) {
        pwm_read_rmt_stats(0, &st, true);
        if (st.n) usbOut.printf("PWM out1=%lu in1 n=%lu width mean=%.1f min=%.1f max=%.1f us period mean=%.1f min=%lu max=%lu us\n",
                                (unsigned long)pwm_test_us, (unsigned long)st.n, st.w_sum / 10.0 / st.n, st.w_min / 10.0, st.w_max / 10.0,
                                st.n > 1 ? (double)st.p_sum / (st.n - 1) : 0.0, (unsigned long)st.p_min, (unsigned long)st.p_max);
        else usbOut.printf("PWM out1=%lu in1 n=0 (no pulse from 900 to 2100 us; last width %ld us)\n", (unsigned long)pwm_test_us,
                           (long)pwm_read_rmt_dur(0));
        // With no pulse counted only (the pin tests disturb OUT1, 2026-10-15 servo map check): the IN1 pin level itself,
        // sampled every 10 us for 45 ms (two 50 Hz periods): edges and the high share tell a wire or output fault (no
        // edges) from a capture fault (edges, but no count)
        if (!st.n) {
        // OUT1's own pad too: only its input path is switched on (gpio_set_direction would route the pad back to the GPIO
        // output register and cut LEDC). Edges there and none on IN1 point at the board path or the wire.
        PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PWM_OUT_1]);
        uint32_t hi = 0, edges = 0, n_s = 0, hi_o = 0, edges_o = 0;
        int prev = gpio_get_level(PWM_IN_1), prev_o = gpio_get_level(PWM_OUT_1);
        const uint32_t t_s = micros();
        while (micros() - t_s < 45000) {
          const int v = gpio_get_level(PWM_IN_1), vo = gpio_get_level(PWM_OUT_1);
          hi += v; edges += v != prev; prev = v;
          hi_o += vo; edges_o += vo != prev_o; prev_o = vo;
          n_s++;
          delayMicroseconds(10);
        }
        usbOut.printf("PWM pins: samples=%lu in1 high=%.1f%% edges=%lu | out1 high=%.1f%% edges=%lu\n", (unsigned long)n_s,
                      100.0 * hi / n_s, (unsigned long)edges, 100.0 * hi_o / n_s, (unsigned long)edges_o);
        // OUT1 as a plain pin for 2 x 5 ms (not while driving; this pass only): high, then low, each read back on OUT1
        // and IN1. A pad that does not follow points at the pin itself (another driver on the line, or the pin mux).
        if (drive_mode_status != 2) {
          ledcDetachPin(PWM_OUT_1);
          gpio_set_direction(PWM_OUT_1, GPIO_MODE_INPUT_OUTPUT);
          gpio_set_level(PWM_OUT_1, 1);
          delayMicroseconds(5000);
          const int oh = gpio_get_level(PWM_OUT_1), ih = gpio_get_level(PWM_IN_1);
          gpio_set_level(PWM_OUT_1, 0);
          delayMicroseconds(5000);
          const int ol = gpio_get_level(PWM_OUT_1), il = gpio_get_level(PWM_IN_1);
          ledcAttachPin(PWM_OUT_1, 1);  // the loop writes OUT1's width again on its next pass
          if (pwm_test_us) writePwmMicros(1, pwm_test_us);
          usbOut.printf("PWM out1 as a plain pin: set 1 -> out1=%d in1=%d | set 0 -> out1=%d in1=%d\n", oh, ih, ol, il);
        }
        pwm_read_rmt_stats(0, &st, true);  // the pulses the tests cut are not counted
        }
      } else {
        const long us = atol(a);
        if (us < 900 || us > 2100) usbOut.println("PWM usage: pwm <us 900-2100> | pwm read | pwm off");
        else if (drive_mode_status == 2 || !vehicleStationary()) usbOut.println("PWM refused: Stop the vehicle first");
        else {
          pwm_test_us = (uint32_t)us;
          writePwmMicros(1, pwm_test_us);
          pwm_read_rmt_stats(0, &st, true);
          usbOut.printf("PWM out1=%ld\n", us);
        }
      }
#ifdef SETTINGS_SELFTEST
    } else if (!strcmp(line, "cfgselftest")) {  // self-check build: the old settings code against the table code
      if (drive_mode_status == 2 || !vehicleStationary()) usbOut.println("CFGTEST refused: Stop the vehicle first");
      else msc_fat_selftest([](const char* s) {
        if (usbOut.room() < 300) usbOut.drain(100);
        usbOut.printf("CFGTEST %s\n", s);
      });
#endif
    } else if (!strncmp(line, "get ", 4) || !strncmp(line, "set ", 4) || !strcmp(line, "save")
               || !strcmp(line, "config erase") || !strcmp(line, "config erase wifi")
               || !strcmp(line, "run start") || !strcmp(line, "run stop")
               || !strncmp(line, "sweep ", 6) || !strncmp(line, "bin ", 4)  // 0.5.103: sweep and bin on USB too
               || !strcmp(line, "tlm")) {
      String reply;  // the same config commands as ESP-NOW CMD 0x10, for a board with ESP-NOW off
      const uint8_t r = configCommand(line, false, reply);
      if (period_ack_want) periodAckDefer(true, nullptr, 0);  // 0.5.104: "CFG 0 ..." after the Rx STM32 answer
      else usbOut.printf("CFG %u %s\n", (unsigned)r, reply.c_str());
    } else {
      usbOut.printf("BENCH unknown command: %s\n", line);
    }
  }
}


// Bench: print the captured frames while the USB queue has room (one frame = about 270 characters).
void stmCaptureService(){
  if (!stm_cap_dump || !stm_cap) return;
  static const char hx[] = "0123456789abcdef";
  while (stm_cap_out < stm_cap_n && usbOut.room() > 400) {
    char line[300];
    int k = snprintf(line, sizeof(line), "STMRAW %u ", (unsigned)stm_cap_out);
    const uint8_t* fr = stm_cap + (size_t)STM_FRAME_LEN * stm_cap_out;
    for (unsigned i = 0; i < STM_FRAME_LEN; i++) { line[k++] = hx[fr[i] >> 4]; line[k++] = hx[fr[i] & 15]; }
    line[k++] = '\n';
    usbOut.write((const uint8_t*)line, k);
    stm_cap_out++;
  }
  if (stm_cap_out >= stm_cap_n && usbOut.room() > 100) {
    usbOut.printf("STMRAW end %u\n", (unsigned)stm_cap_n);
    free(stm_cap);
    stm_cap = nullptr;
    stm_cap_dump = false;
  }
}

// Bench: flash stall measurement ----------------------------------------------------------------
// "flashtest erase <n>", "flashtest write <n>" or "flashtest idle <n>": n operations on the "tlog"
// partition (never another one), one each 500 ms from loop(). Each operation's duration is
// measured. At the end: the STM32 packet, CRC error and UART overrun counts over the test, and
// the packets expected at 100 Hz. "idle" does no flash operation (the baseline).
// A flash erase or write turns the cache off; the UART driver's interrupt is not in IRAM, so
// packets can be lost during it. This measures how many.
int ft_mode = 0;  // 0 off, 1 erase, 2 write, 3 idle
uint32_t ft_left = 0, ft_done = 0, ft_next_ms = 0, ft_start_ms = 0, ft_offset = 0;
uint32_t ft_pk0 = 0, ft_crc0 = 0, ft_ovr0 = 0, ft_us_max = 0, ft_us_sum = 0;
const esp_partition_t* ft_part = nullptr;

void flashTestCommand(const char* arg){
  char op[8] = {0};
  unsigned long n = 0;
  if (sscanf(arg, "%7s %lu", op, &n) != 2 || n == 0 || n > FLASHTEST_SECTORS) { usbOut.println("FLASHTEST usage: flashtest erase|write|idle <1..16>"); return; }
  ft_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "tlog");
  if (!ft_part) { usbOut.println("FLASHTEST no tlog partition (flash the new partition table)"); return; }
  ft_mode = !strcmp(op, "erase") ? 1 : !strcmp(op, "write") ? 2 : !strcmp(op, "idle") ? 3 : 0;
  if (!ft_mode) { usbOut.println("FLASHTEST unknown op"); return; }
  ft_left = n; ft_done = 0; ft_us_max = 0; ft_us_sum = 0;
  ft_offset = ft_part->size - FLASHTEST_SECTORS * 4096u;  // the start of the test area: n sectors erased, or n pages written
  ft_start_ms = millis(); ft_next_ms = ft_start_ms + 500;
  ft_pk0 = main_loop_counter; ft_crc0 = crc_error_count; ft_ovr0 = uart_overrun_count;
  usbOut.printf("FLASHTEST start %s n=%lu partition=0x%lx size=0x%lx\n", op, n, (unsigned long)ft_part->address, (unsigned long)ft_part->size);
}

void flashTestService(){
  if (!ft_mode || (int32_t)(millis() - ft_next_ms) < 0) return;
  ft_next_ms += 500;
  if (ft_left > 0) {
    static uint8_t page[256];
    for (int i = 0; i < 256; i++) page[i] = (uint8_t)(i ^ ft_done);
    const uint32_t t0 = micros();
    esp_err_t err = ESP_OK;
    if (ft_mode == 1) err = esp_partition_erase_range(ft_part, ft_offset & ~0xFFFu, 4096);
    else if (ft_mode == 2) err = esp_partition_write(ft_part, ft_offset, page, sizeof(page));
    const uint32_t us = micros() - t0;
    ft_offset += (ft_mode == 1) ? 4096 : (ft_mode == 2 ? 256 : 0);
    if (err != ESP_OK) usbOut.printf("FLASHTEST op error %d\n", (int)err);
    if (us > ft_us_max) ft_us_max = us;
    ft_us_sum += us;
    ft_done++;
    ft_left--;
    return;
  }
  // One more 500 ms after the last operation, then the result.
  const uint32_t ms = millis() - ft_start_ms;
  const uint32_t packets = (main_loop_counter - ft_pk0 + MAIN_LOOP_PERIOD) % MAIN_LOOP_PERIOD;
  usbOut.printf("FLASHTEST done mode=%d ops=%lu op_us_max=%lu op_us_mean=%lu ms=%lu packets=%lu expected=%lu crc_err=%lu overrun=%lu\n",
    ft_mode, (unsigned long)ft_done, (unsigned long)ft_us_max, (unsigned long)(ft_done ? ft_us_sum / ft_done : 0),
    (unsigned long)ms, (unsigned long)packets, (unsigned long)(ms / 10), (unsigned long)(crc_error_count - ft_crc0),
    (unsigned long)(uart_overrun_count - ft_ovr0));
  ft_mode = 0;
}

