// The radio: ESP-NOW and Wi-Fi telemetry, the text telemetry message, the monitor link.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

uint32_t last_charge_code = 0;
uint32_t last_status_bits = 0;
// Bench "radio slot": the radio part of loop() (all sends) runs only from radio_slot_us to radio_slot_us +
// RADIO_SLOT_LEN_US after the last nDATA_READY edge; -1 = no slot. radio_tx_hist: send-done times after the
// edge, in tenths of the frame period (10 = later than one period).
int32_t radio_slot_us = -1;
const uint32_t RADIO_SLOT_LEN_US = 1000;
uint32_t radio_tx_hist[11] = {};

// Telemetry FIFO queue for ESP-NOW messages (prevents flooding)
#define TELEMETRY_FIFO_LENGTH 10
typedef struct {
  String message;
} TelemetryMessage;
TelemetryMessage telemetryFifo[TELEMETRY_FIFO_LENGTH];
uint32_t telemetry_fifo_head = 0;
uint32_t telemetry_fifo_tail = 0;
uint32_t lastTelemetrySendTime = 0;  // Track last send time for 100ms throttling

uint32_t brodcastMillis = 0;
uint32_t telemetryBrodcastMillis = 0;

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) // Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

// ESP-NOW bench link (protocol agreed with the monitor role, little-endian, byte 0 = type):
//   0x01 THRU  u8 type, u32 seq, u32 sender_us, zero padding to the test size (the rover broadcasts)
//   0x02 PING  u8 type, u32 seq, u32 rxb_us (the monitor sends it to this board)
//   0x03 PONG  the PING bytes with byte 0 set to 0x03, sent back to the PING's source
// The callbacks run in the Wi-Fi task at priority 23 and interrupt loop(): they only copy
// and count. loop() sends the PONG and the THRU frames (espnowService()).
#define ESPNOW_TYPE_THRU 0x01
#define ESPNOW_TYPE_PING 0x02
#define ESPNOW_TYPE_PONG 0x03
bool espnow_started = false;
volatile uint32_t espnow_bad_key = 0;  // CMD or RESTART frames refused for a wrong or missing fleet key
bool espnow_failed = false;  // esp_now_init() failed once: not tried again
volatile bool espnow_ping_pending = false;
uint8_t espnow_ping_mac[6];
uint8_t espnow_ping_buf[16];
volatile uint8_t espnow_ping_len = 0;
volatile uint32_t espnow_rx_count = 0, espnow_ping_count = 0, espnow_ping_missed = 0;
volatile uint32_t espnow_tx_ok = 0, espnow_tx_fail = 0;
uint32_t espnow_send_err = 0, espnow_pong_count = 0;
bool espnow_thru_active = false;
uint32_t espnow_thru_period_us = 0, espnow_thru_size = 0, espnow_thru_left = 0, espnow_thru_seq = 0, espnow_thru_last_us = 0;
uint8_t espnow_thru_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Bench loss simulation (USB "loss <tx_pct> <rx_pct>", 0 at boot): the rover drops this share of
// its telemetry sends (live, INFO and backfill) and of the frames it receives (REQ, CMD, PING).
volatile uint8_t sim_loss_tx = 0, sim_loss_rx = 0;
volatile uint32_t sim_dropped_tx = 0, sim_dropped_rx = 0;

// Config commands (CMD 0x10 / ACK 0x11) and restarts (0x7F "RST!" / "DLD!").
#define ESPNOW_TYPE_CMD     0x10
#define ESPNOW_TYPE_ACK     0x11
#define ESPNOW_TYPE_STOP    0x12  // u8 type, u16 seq, u32 fleet key: stop at once (docs/espnow_protocol.md)
#define SETTINGS_DEF_VERSION 16  // docs/settings_def.json version (16: three rover groups, Roam mode, words from the table (0.7.0); 15: tx_dir default 1 (0.6.28); 14: rover ranges widened, every range from the table (0.6.27); 13: generated from include/settings_table.h by tools/gen_settings_def.py; 6: DSP settings dsp_*, groups for users; 7: servo map, drive_output, vehicle_name, steer_lpf_hz; 8: radio_mode, wifi_ssid, wifi_password; 9: dsp_sweep_bins; 10: dsp_frame_period_us min 2000; 11: min 2336; 12: min 2938)
#define ESPNOW_TYPE_RESTART 0x7F
#define ESPNOW_TYPE_MON     0x44  // monitor broadcast, once a second: u8 type, u8 version, u32 monitor ms, u64 PC unix ms (0 = unknown), u8 n
// The last MON frame (set in the Wi-Fi task, read in loop()).
volatile uint32_t mon_rx_ms = 0;       // millis() at the last MON frame; 0 = none since boot
volatile uint64_t mon_rx_unix_ms = 0;  // its PC time; 0 = the monitor has no PC time
uint8_t mon_mac[6] = {0};
volatile bool mon_new = false;
bool pc_time_known = false;    // a MON frame brought PC time
int64_t pc_offset_ms = 0;      // PC unix ms - millis()
uint32_t mon_blip_until = 0;   // LED inverted until this millis() (100 ms per MON frame)
bool mon_blip_on = false;      // the LED pin is inverted now
uint8_t mon_blip_level = 0;    // the level that the inversion wrote
#define ESPNOW_CMD_TEXT_MAX 200
volatile bool espnow_cmd_pending = false;
uint8_t espnow_cmd_mac[6];
uint16_t espnow_cmd_seq = 0;
char espnow_cmd_text[ESPNOW_CMD_TEXT_MAX + 1];
volatile bool espnow_busy_pending = false;   // a CMD arrived while one was in hand: answer "busy"
uint8_t espnow_busy_mac[6];
uint16_t espnow_busy_seq = 0;
volatile uint8_t espnow_restart_request = 0;  // 1 normal restart, 2 restart into ROM download mode
volatile int espnow_list_index = -1;          // >= 0 while a "list" answer is in progress
bool espnow_list_saved = false;               // 0.6.15: the answer in progress is "list saved"
DeviceConfig* g_saved_config = nullptr;       // 0.6.15: the settings of the next start, loaded at each "list saved"
                                              // (0.6.33: from the heap at the first "list saved"; static RAM is full)
DeviceConfig* savedConfigLoad(){
  if (!g_saved_config) g_saved_config = new (std::nothrow) DeviceConfig();
  if (g_saved_config) msc_fat_saved_config(g_saved_config);
  return g_saved_config;
}
volatile bool espnow_stop_request = false;    // STOP frame received; loop() stops at its next pass
uint8_t espnow_stop_mac[6];
uint16_t espnow_stop_seq = 0;
uint32_t espnow_list_ms = 0;

// ESP-NOW runs for the text status (espnow_enabled) or for the telemetry log (telemetry_enabled,
// on by default). Live frames are broadcast and nothing waits for a peer, so a monitor that is
// off costs the mower nothing.
// Radio mode (setting radio_mode), fixed at boot: ESP-NOW or a Wi-Fi network, never both. A change
// takes effect at the next start. In "wifi" mode the same flags (espnow_enabled, telemetry_enabled)
// switch the radio on, and the same frames go to the PC telemetry server as UDP (lib/wifi_telem).
bool radio_wifi = false;
// 0.5.103: the shortest update period for each link mode: the shortest setting with all counts zero and
// a mean CPU load under 90 % (0.5.102 table, 2026-10-14: every mode passes at the shortest setting tested, 2000 us =
// 2.38 ms frames; Wi-Fi 78 %, ESP-NOW 65 %, no link 58 %). A later link mode gets its own row.
// 0.5.105: 2938 us in every mode: Rx STM32 2.0.57 keeps only 7 % free time at 2354 us
// (GAPS run). The floor no longer follows the Rx STM32 shortest period (2354 / 2336 us), which sits below it.
// A "set" below the minimum of the link mode is refused with the minimum; dspApply sends at least the minimum.
#define PERIOD_MIN_NO_LINK_US 2938
#define PERIOD_MIN_ESPNOW_US 2938
#define PERIOD_MIN_WIFI_US 2938
enum { LINK_NONE, LINK_ESPNOW, LINK_WIFI };
static int linkModeOf(const DeviceConfig& c, bool wifi) {
  if (safe_mode || !(c.espnow_enabled || c.telemetry_enabled)) return LINK_NONE;
  return wifi ? LINK_WIFI : LINK_ESPNOW;
}
static int periodMinUs(int m) { return m == LINK_WIFI ? PERIOD_MIN_WIFI_US : m == LINK_ESPNOW ? PERIOD_MIN_ESPNOW_US : PERIOD_MIN_NO_LINK_US; }
// 0.5.104: the shortest period the Rx STM32 runs (2.0.57): 2354 us with the slow ADC on (CID 39
// = 1, the default), 2336 us with it off. A shorter request runs at that period with no notice, so the floor is the
// larger of the link mode row and this value.
#define STM_PERIOD_MIN_SLOW_US 2354
#define STM_PERIOD_MIN_FAST_US 2336
static int periodFloorUs(int m, const DeviceConfig& c) {
  const int stm = c.dsp_slow_adc ? STM_PERIOD_MIN_SLOW_US : STM_PERIOD_MIN_FAST_US;
  return periodMinUs(m) > stm ? periodMinUs(m) : stm;
}
static const char* linkModeName(int m) { return m == LINK_WIFI ? "Wi-Fi" : m == LINK_ESPNOW ? "ESP-NOW" : "no link"; }
// The link mode with the longer minimum: the one in use now, or the one of the next start (radio_mode).
static int linkModeStrict(const DeviceConfig& c) {
  const int now_m = linkModeOf(c, radio_wifi), next_m = linkModeOf(c, c.radio_mode == RADIO_MODE_WIFI);
  return periodMinUs(now_m) >= periodMinUs(next_m) ? now_m : next_m;
}
bool espnowWanted(){ return !safe_mode && !radio_wifi && (g_deviceConfig.espnow_enabled || g_deviceConfig.telemetry_enabled); }
bool wifiWanted(){ return !safe_mode && radio_wifi && (g_deviceConfig.espnow_enabled || g_deviceConfig.telemetry_enabled); }

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
// Called when data is received (Wi-Fi task): copy a PING, CMD or RESTART for loop(), nothing else
{
  if (sim_loss_rx && esp_random() % 100 < sim_loss_rx) { sim_dropped_rx++; return; }  // bench loss simulation
  espnow_rx_count++;
  telemlog_on_frame(macAddr, data, dataLen);  // REQ 0x41 only; it ignores every other frame
  // Commands and restarts carry the fleet key (cmd_key). The frame key must equal cmd_key; cmd_key 0
  // is a blank password and accepts key 0 (design choice 2026-10-10). Any MAC may send them: every Rx
  // board has its own MAC, so a dongle or PC can change without a change on the vehicles.
  const uint32_t key = (uint32_t)g_deviceConfig.cmd_key;
  if (dataLen >= 1 && (data[0] == ESPNOW_TYPE_RESTART || data[0] == ESPNOW_TYPE_CMD || data[0] == ESPNOW_TYPE_STOP)) {
    const int at = data[0] == ESPNOW_TYPE_RESTART ? 5 : 3;
    uint32_t got = 0;
    if (dataLen >= at + 4) memcpy(&got, &data[at], 4);
    if (dataLen < at + 4 || got != key) { espnow_bad_key++; return; }  // no answer, no effect
  }
  if (dataLen >= 9 && data[0] == ESPNOW_TYPE_RESTART) {
    if (!memcmp(&data[1], "RST!", 4)) espnow_restart_request = 1;
    else if (!memcmp(&data[1], "DLD!", 4)) espnow_restart_request = 2;
    return;
  }
  if (dataLen >= 7 && data[0] == ESPNOW_TYPE_STOP) {  // never busy, never cached: a flag for loop()
    memcpy(espnow_stop_mac, macAddr, 6);
    espnow_stop_seq = (uint16_t)(data[1] | (data[2] << 8));
    espnow_stop_request = true;
    return;
  }
  if (dataLen >= 7 && data[0] == ESPNOW_TYPE_CMD) {
    const uint16_t seq = (uint16_t)(data[1] | (data[2] << 8));
    if (espnow_cmd_pending || espnow_list_index >= 0) {  // one CMD at a time
      memcpy(espnow_busy_mac, macAddr, 6);
      espnow_busy_seq = seq;
      espnow_busy_pending = true;
      return;
    }
    const int n = min(dataLen - 7, ESPNOW_CMD_TEXT_MAX);
    memcpy(espnow_cmd_mac, macAddr, 6);
    espnow_cmd_seq = seq;
    memcpy(espnow_cmd_text, &data[7], n);
    espnow_cmd_text[n] = 0;
    espnow_cmd_pending = true;
    return;
  }
  if (dataLen >= 15 && data[0] == ESPNOW_TYPE_MON) {  // a monitor hears this radio channel; PC time
    memcpy(mon_mac, macAddr, 6);
    uint64_t u = 0;
    memcpy(&u, &data[6], 8);
    mon_rx_unix_ms = u;
    mon_rx_ms = millis() ? millis() : 1;
    mon_new = true;
    return;
  }
  if (dataLen >= 9 && dataLen <= (int)sizeof(espnow_ping_buf) && data[0] == ESPNOW_TYPE_PING) {
    if (espnow_ping_pending) {  // loop() has not answered the previous PING yet
      espnow_ping_missed++;
      return;
    }
    memcpy(espnow_ping_mac, macAddr, 6);
    memcpy(espnow_ping_buf, data, dataLen);
    espnow_ping_len = dataLen;
    espnow_ping_count++;
    espnow_ping_pending = true;
  }
}

void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status) // Called when data is sent
{
  if (status == ESP_NOW_SEND_SUCCESS) espnow_tx_ok++;
  else espnow_tx_fail++;
  const uint32_t per_us = (uint32_t)(stmFramePeriodS() * 1e6);
  const uint32_t dt = (uint32_t)esp_timer_get_time() - stm_edge_us;
  radio_tx_hist[per_us ? min<uint32_t>(10, dt * 10 / per_us) : 10]++;
}

void broadcast(const String &message) {
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);

  // Only add the peer if it's not already added
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // Print results to serial monitor
  if (result != ESP_OK) {
    usbOut.println("ESP-NOW send failed");
  }
}

// Bench radio slot (see radio_slot_us): true when the radio part of loop() may run now.
bool radioSlotOpen(){
  if (radio_slot_us < 0) return true;
  const uint32_t dt = micros() - stm_edge_us;
  return dt >= (uint32_t)radio_slot_us && dt < (uint32_t)radio_slot_us + RADIO_SLOT_LEN_US;
}

// Start ESP-NOW once; also called from loop() so a config.json change takes effect without a reset.
// Radio transmit power (setting espnow_tx_power_dbm), for every frame of this radio. Wi-Fi must run.
void applyTxPower(){
  wifi_mode_t m;
  if (esp_wifi_get_mode(&m) != ESP_OK || m == WIFI_MODE_NULL) return;  // no radio yet (both roles)
  esp_wifi_set_max_tx_power((int8_t)lround(g_deviceConfig.espnow_tx_power_dbm * 4.0));
}

// MON frames (docs/espnow_protocol.md): event 17 when a monitor is heard or not heard for 5 s;
// event 16 when the PC time first comes, or when it differs from the last one by more than 1 s.
uint8_t monitorAgeS(){
  const uint32_t r = mon_rx_ms;
  if (!r) return 255;
  const uint32_t s = (millis() - r) / 1000;
  return s > 254 ? 255 : (uint8_t)s;
}
void monitorLinkService(){
  static bool heard = false;
  if (mon_new) {
    mon_new = false;
    const uint32_t r = mon_rx_ms;
    mon_blip_until = millis() + 100;  // green LED: a 100 ms inversion per MON frame (a monitor hears this rover)
    const uint64_t u = mon_rx_unix_ms;
    char t[48];
    if (!heard) {
      heard = true;
      snprintf(t, sizeof(t), "monitor %02x:%02x:%02x:%02x:%02x:%02x", mon_mac[0], mon_mac[1], mon_mac[2], mon_mac[3], mon_mac[4], mon_mac[5]);
      telemetryEvent(rec::EV_MONITOR, 1, 0, t);
      usbOut.printf("MONITOR heard: %s\n", t);
    }
    if (u) {
      const int64_t off = (int64_t)u - (int64_t)r;
      if (!pc_time_known || llabs(off - pc_offset_ms) > 1000) {
        pc_time_known = true;
        pc_offset_ms = off;
        snprintf(t, sizeof(t), "unix_ms=%llu", (unsigned long long)u);
        telemetryEvent(rec::EV_TIME, (int32_t)(u / 1000), (int32_t)r, t);
        usbOut.printf("TIME %s at ms=%lu\n", t, (unsigned long)r);
      }
    }
  }
  if (heard && monitorAgeS() >= 5) {
    heard = false;
    telemetryEvent(rec::EV_MONITOR, 0, 5, "no MON frame for 5 s");
    usbOut.println("MONITOR lost: no MON frame for 5 s");
  }
}

// Green LED (nLED, GPIO1): invert the level that the drive-mode code set. The output register holds
// it (the pin is output only, so digitalRead() cannot read it).
uint8_t ledLevel(){ return (REG_READ(GPIO_OUT_REG) >> nLED) & 1; }
// Once per loop pass: inverted while a MON window runs (100 ms). When the drive-mode code writes the
// LED in the window, the next pass inverts its new level. At the window end, the pass restores it.
void monLedService(){
  const uint8_t cur = ledLevel();
  if ((int32_t)(millis() - mon_blip_until) < 0) {
    if (!mon_blip_on || cur != mon_blip_level) {
      mon_blip_level = cur ? 0 : 1;
      digitalWrite(nLED, mon_blip_level);
      mon_blip_on = true;
    }
  } else if (mon_blip_on) {
    if (cur == mon_blip_level) digitalWrite(nLED, cur ? 0 : 1);
    mon_blip_on = false;
  }
}

void espnowStart(){
  if (espnow_started || espnow_failed) return;
  const uint32_t t0 = millis();
  WiFi.persistent(false);  // the Wi-Fi driver keeps its settings in RAM: no write to its NVS area at a start
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station (no AP, default channel)
  if (esp_now_init() != ESP_OK) {
    usbOut.println("ESP-NOW Init Failed (ESP-NOW stays off until a restart)");
    espnow_failed = true;  // no retry, no restart: the vehicle keeps driving without the radio
    return;
  }
  esp_now_register_recv_cb(receiveCallback);  // Register receive callback
  esp_now_register_send_cb(sentCallback);     // Register send callback
  espnow_started = true;
  applyTxPower();
  boot_espnow_ms = millis() - t0;
  usbOut.printf("ESP-NOW Init Success, MAC %s, %lu ms\n", WiFi.macAddress().c_str(), (unsigned long)boot_espnow_ms);
}

// A unicast send needs the destination in the peer list.
void espnowEnsurePeer(const uint8_t* mac){
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  esp_now_add_peer(&peer);
}

// ACK 0x11: u8 type, u16 seq, u8 result, ASCII text.
void espnowSendAck(const uint8_t* mac, uint16_t seq, uint8_t result, const char* text){
  uint8_t frame[ESP_NOW_MAX_DATA_LEN];
  frame[0] = ESPNOW_TYPE_ACK;
  frame[1] = seq & 0xFF;
  frame[2] = seq >> 8;
  frame[3] = result;
  const size_t n = min(strlen(text), (size_t)(ESP_NOW_MAX_DATA_LEN - 4));
  memcpy(&frame[4], text, n);
  if (radio_wifi) {  // Wi-Fi mode: the ACK goes to the server as a datagram (docs/wifi_interface.md)
    if (!wifilink::send(frame, 4 + n)) espnow_send_err++;
    return;
  }
  espnowEnsurePeer(mac);
  if (esp_now_send(mac, frame, 4 + n) != ESP_OK) espnow_send_err++;
}

// Wi-Fi telemetry (radio_mode "wifi", lib/wifi_telem) ------------------------------------------------
// The same payloads as the ESP-NOW path: the log frames (live, INFO, backfill) and the text telemetry
// message. Each goes to the PC telemetry server as one unicast UDP datagram. Telemetry only: no
// command comes in over Wi-Fi.
void wifiEvent(int32_t a, int32_t b, const char* text){ telemetryEvent(rec::EV_WIFI, a, b, text); }
// A payload from the server: only a REQ (backfill request) is used, and the log takes it as from
// ESP-NOW. The MAC is a fixed stand-in: in wifi mode every send goes to the server.
void wifiFrame(const uint8_t* data, int len){
  static const uint8_t server_mac[6] = {0x02, 0x57, 0x49, 0x46, 0x49, 0x00};  // locally administered
  // 0.5.86: CMD, STOP, RESTART and REQ from the server take the ESP-NOW path (fleet key, one CMD at a time, retry
  // cache); the answers go back as datagrams. wifilink passes only datagrams from the known server address.
  receiveCallback(server_mac, data, len);
}
uint32_t web_upload_quiet_ms = 0;  // 0.6.15: millis() of the last update page grant; telemetry pauses 5 min from it
bool wifiTelemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len){
  (void)mac;  // the server address replaces it
  if (sim_loss_tx && esp_random() % 100 < sim_loss_tx) { sim_dropped_tx++; return true; }  // bench loss simulation, as telemetrySend()
  // 0.6.15: no telemetry datagrams during an update page upload. Each one takes and frees heap; over the upload the
  // largest free block fell to 2.3 KB, the Wi-Fi task could not take its 2.3 KB receive buffers (189 failures) and
  // the upload stopped for 30 s. The upload ends in a restart, or the pause in 5 min.
  if (web_upload_quiet_ms && millis() - web_upload_quiet_ms < 300000) return true;
  return wifilink::send(buf, len);  // false before the server is known: counted as unsent
}
// Radio start in wifi mode, once; stops loop() for about 220 ms, as espnowStart() does.
// The WPA2 key (PBKDF2-SHA1, 4096 rounds, 32 bytes) as 64 hex digits. Given a passphrase, the Wi-Fi driver calculates
// this key in its own task at the first association; that task stopped loop() for 190 to 417 ms. Given the key, the
// driver calculates nothing. False for an open network or a password that is already 64 hex digits.
bool wifiKeyFromPassphrase(const char* ssid, const char* pass, char out[65]){
  const size_t n = strlen(pass);
  if (n < 8 || n > 63) return false;
  uint8_t pmk[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const bool ok = mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA1), 1) == 0 &&
                  mbedtls_pkcs5_pbkdf2_hmac(&ctx, (const unsigned char*)pass, n, (const unsigned char*)ssid, strlen(ssid),
                                            4096, sizeof(pmk), pmk) == 0;
  mbedtls_md_free(&ctx);
  if (ok) for (int i = 0; i < 32; i++) snprintf(out + 2 * i, 3, "%02x", pmk[i]);
  memset(pmk, 0, sizeof(pmk));
  return ok;
}
// wifilink::begin() or setCredentials() with the key in place of the passphrase.
void wifiCredentials(const char* ssid, const char* pass){
  char key[65];
  const uint32_t t0 = millis();
  const bool derived = wifiKeyFromPassphrase(ssid, pass, key);
  if (derived) usbOut.printf("WIFI key calculated in %lu ms\n", (unsigned long)(millis() - t0));
  if (wifilink::started()) wifilink::setCredentials(ssid, derived ? key : pass);
  else wifilink::begin(ssid, derived ? key : pass);
  memset(key, 0, sizeof(key));
}
void wifiStart(){
  if (wifilink::started()) return;
  char pass[WIFI_PASS_MAX + 1];
  msc_fat_wifi_password(pass, sizeof(pass));
  wifiCredentials(msc_fat_wifi_ssid(), pass);
  memset(pass, 0, sizeof(pass));
  applyTxPower();  // espnow_tx_power_dbm sets the Wi-Fi transmit power too
}
// The text telemetry message that broadcastTelemetry() queues, over UDP (telemetryFifo_sendMessage() for Wi-Fi).
void wifiTelemetryFifoSend(){
  if (millis() - lastTelemetrySendTime < 100 || telemetry_fifo_head == telemetry_fifo_tail) return;
  const String& m = telemetryFifo[telemetry_fifo_tail].message;
  wifiTelemetrySend(nullptr, (const uint8_t*)m.c_str(), m.length());
  telemetry_fifo_tail = (telemetry_fifo_tail + 1) % TELEMETRY_FIFO_LENGTH;
  lastTelemetrySendTime = millis();
}

// USB "wifi test [seconds]": the setup page checks the Wi-Fi network before the user unplugs
// the Rx. Two results: the network join, then the telemetry server. Progress lines, then
//   WIFI test join start ssid=<ssid>
//   WIFI test join ok ssid=<ssid> rssi=<dBm> ch=<n> ip=<a.b.c.d> ms=<join time>
//   WIFI test join fail reason=no_network|wrong_password|not_found|no_address|radio_off|vehicle_moved
//   WIFI test server wait s=<limit>
//   WIFI test server ok ip=<a.b.c.d> port=<n> via=beacon|mdns ms=<time after the join>
//   WIFI test server fail reason=not_heard|join_lost|vehicle_moved s=<wait>
//   WIFI test end link=espnow|wifi
// No save, no restart, no flash write. In espnow mode the test starts the Wi-Fi link and stops it at the end; ESP-NOW
// frames are lost while the radio is on the network's channel, so the test needs a stopped vehicle. In wifi mode the
// test reads the running link. Times: the join limit 15 s (WIFI_TEST_JOIN_MS: the driver gives up an attempt in about
// 10 s; 15 s covers one attempt and a slow DHCP answer); the rest of the test time waits for the server (the server
// sends its beacon each second and the rover asks mDNS at once, so 30 s is many chances; the default 45 s total).
bool vehicleStationary();
#define WIFI_TEST_JOIN_MS 15000u
#define WIFI_TEST_DEFAULT_S 45
static struct {
  uint8_t step = 0;       // 0 off, 1 join, 2 server wait
  bool own_link = false;  // the test started the link (espnow mode): it stops it at the end
  uint8_t channel = 0;    // the ESP-NOW channel before the test
  uint32_t t0 = 0, t_join = 0, limit_ms = 0;
} wtest;
static volatile uint8_t wtest_disc_reason = 0;  // the last disconnect reason (Wi-Fi event task)
static volatile bool wtest_assoc = false;       // associated with the network (an address may still be missing)
static void wifiTestEvent(arduino_event_id_t ev, arduino_event_info_t info){
  if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) { wtest_disc_reason = info.wifi_sta_disconnected.reason; wtest_assoc = false; }
  else if (ev == ARDUINO_EVENT_WIFI_STA_CONNECTED) wtest_assoc = true;
}
static const char* wifiTestJoinReason(){
  switch (wtest_disc_reason) {
    case WIFI_REASON_NO_AP_FOUND: return "not_found";
    case WIFI_REASON_AUTH_FAIL: case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: case WIFI_REASON_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_AUTH_EXPIRE: case WIFI_REASON_MIC_FAILURE: return "wrong_password";
    default: return wtest_assoc ? "no_address" : "not_found";
  }
}
static void wifiTestEnd(){
  if (wtest.own_link) {
    wifilink::end();
    if (wtest.channel) esp_wifi_set_channel(wtest.channel, WIFI_SECOND_CHAN_NONE);  // ESP-NOW back on its channel
  }
  usbOut.printf("WIFI test end link=%s\n", radio_wifi ? "wifi" : "espnow");
  wtest.step = 0;
  wtest.own_link = false;
}
static void wifiTestStart(unsigned secs){
  if (wtest.step) { usbOut.println("WIFI test refused: a test runs"); return; }
  if (!msc_fat_wifi_ssid()[0]) { usbOut.println("WIFI test join fail reason=no_network"); usbOut.println("WIFI test end link=none"); return; }
  if (!radio_wifi && !espnow_started) { usbOut.println("WIFI test join fail reason=radio_off"); usbOut.println("WIFI test end link=none"); return; }
  // Wheels still, not drive mode: an Rx on USB with no RC transmitter starts in mode 2 (autonomous) and stands still.
  if (!radio_wifi && !vehicleStationary()) {
    usbOut.println("WIFI test refused: Stop the vehicle first (the test pauses ESP-NOW)");
    return;
  }
  if (radio_wifi && !wifilink::started()) { usbOut.println("WIFI test join fail reason=radio_off"); usbOut.println("WIFI test end link=none"); return; }
  static bool hooked = false;
  if (!hooked) {
    hooked = true;
    WiFi.onEvent(wifiTestEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(wifiTestEvent, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  }
  wtest_disc_reason = 0;
  wtest_assoc = wifilink::connected();
  wtest.t0 = millis();
  wtest.limit_ms = secs * 1000u;
  wtest.own_link = false;
  usbOut.printf("WIFI test join start ssid=%s\n", msc_fat_wifi_ssid());
  if (!radio_wifi) {
    wifi_second_chan_t sc;
    if (esp_wifi_get_channel(&wtest.channel, &sc) != ESP_OK) wtest.channel = 0;
    wtest.own_link = true;
    wifiStart();  // the stored network; the key is calculated as in wifi mode
  }
  wtest.step = 1;
}
static void wifiTestService(uint32_t now){
  if (!wtest.step) return;
  if (wtest.own_link) wifilink::service(now);  // in wifi mode radioService() runs it
  const uint32_t t = now - wtest.t0;
  if (wtest.own_link && !vehicleStationary()) {  // the wheels turn: ESP-NOW back at once
    usbOut.printf("WIFI test %s fail reason=vehicle_moved\n", wtest.step == 1 ? "join" : "server");
    wifiTestEnd();
    return;
  }
  if (wtest.step == 1) {
    if (wifilink::connected()) {
      const IPAddress ip = WiFi.localIP();
      usbOut.printf("WIFI test join ok ssid=%s rssi=%d ch=%d ip=%u.%u.%u.%u ms=%lu\n", msc_fat_wifi_ssid(), (int)WiFi.RSSI(),
                    (int)WiFi.channel(), ip[0], ip[1], ip[2], ip[3], (unsigned long)t);
      wtest.t_join = now;
      wtest.step = 2;
      const uint32_t left = wtest.limit_ms > t ? wtest.limit_ms - t : 0;
      usbOut.printf("WIFI test server wait s=%lu\n", (unsigned long)((left + 999) / 1000));
    } else if (t >= WIFI_TEST_JOIN_MS || t >= wtest.limit_ms) {
      usbOut.printf("WIFI test join fail reason=%s\n", wifiTestJoinReason());
      wifiTestEnd();
    }
    return;
  }
  uint8_t ip[4];
  uint16_t port;
  const char* via;
  if (wifilink::server(ip, &port, &via)) {
    usbOut.printf("WIFI test server ok ip=%u.%u.%u.%u port=%u via=%s ms=%lu\n", ip[0], ip[1], ip[2], ip[3], (unsigned)port,
                  via, (unsigned long)(now - wtest.t_join));
    wifiTestEnd();
  } else if (!wifilink::connected()) {
    usbOut.printf("WIFI test server fail reason=join_lost s=%lu\n", (unsigned long)((now - wtest.t_join) / 1000));
    wifiTestEnd();
  } else if (t >= wtest.limit_ms) {
    usbOut.printf("WIFI test server fail reason=not_heard s=%lu\n", (unsigned long)((now - wtest.t_join) / 1000));
    wifiTestEnd();
  }
}

// USB "wifi" commands. The password goes to NVS only; no line, event or log record carries it.
//   wifi                          status: mode, network, association, RSSI, IP, server, beacon age, counters
//   wifi scan                     one line per network: SSID, RSSI, channel, security
//   wifi set <ssid> <password>    "wifi set "<ssid with spaces>" <password>"; no password: an open network
//   wifi clear                    removes the network and the password
//   wifi blast <bytes> <per_s> <secs> [port]   throughput test to the server address (wifilink::blastStart)
//   wifi test [seconds]           join the stored network and find the telemetry server (wifiTestStart, above)
bool vehicleStationary();
void wifiUsbCommand(const char* line){
  const char* mode = radio_wifi ? "wifi" : "espnow";
  const char* setting = g_deviceConfig.radio_mode == RADIO_MODE_WIFI ? "wifi" : "espnow";
  if (!strcmp(line, "wifi")) {
    wifilink::printStatus(mode, setting, msc_fat_wifi_ssid());
    usbOut.printf("WIFI password=%s\n", wifitelem::passwordState(msc_fat_wifi_has_password()));
  } else if (!strcmp(line, "wifi scan")) {
    // In espnow mode the scan takes the radio off the ESP-NOW channel for about 2 s: not while the vehicle moves.
    if (!radio_wifi && !espnow_started) usbOut.println("WIFI scan refused: the radio is off (espnow_enabled and telemetry_enabled false)");
    else if (radio_wifi && !wifilink::started()) usbOut.println("WIFI scan refused: the radio is off (espnow_enabled and telemetry_enabled false)");
    else if (!radio_wifi && !vehicleStationary()) usbOut.println("WIFI scan refused: Stop the vehicle first (a scan pauses ESP-NOW for about 2 s)");
    else wifilink::scanStart();
  } else if (!strcmp(line, "wifi test") || !strncmp(line, "wifi test ", 10)) {
    unsigned secs = WIFI_TEST_DEFAULT_S;
    if (line[9] && (sscanf(line + 10, "%u", &secs) != 1 || secs < 20 || secs > 120))
      usbOut.println("WIFI usage: wifi test [seconds 20-120, default 45]");
    else wifiTestStart(secs);
  } else if (!strcmp(line, "wifi blast stop")) {
    wifilink::blastStop();
  } else if (!strncmp(line, "wifi blast ", 11)) {
    unsigned b = 0, r = 0, s = 0, port = 47702;
    if (sscanf(line + 11, "%u %u %u %u", &b, &r, &s, &port) < 3 || b < 16 || b > 1472 || !r || r > 5000 || !s || s > 600 || port > 65535)
      usbOut.println("WIFI usage: wifi blast <bytes 16-1472> <per_s 1-5000> <secs 1-600> [port, default 47702] | wifi blast stop");
    else if (!wifilink::blastStart(b, r, s, port)) usbOut.println("WIFI blast refused: no association or no server");
  } else if (!strncmp(line, "wifi set ", 9) || !strcmp(line, "wifi clear")) {
    if (!vehicleStationary() || drive_mode_status == 2) { usbOut.println("WIFI refused: Stop the vehicle first"); return; }
    if (!strcmp(line, "wifi clear")) {
      msc_fat_wifi_clear();
      if (wifilink::started()) wifilink::setCredentials("", "");
      usbOut.println("WIFI cleared: no network stored");
      telemetryEvent(rec::EV_SETTING, 0, 0, "wifi clear");
      return;
    }
    char ssid[WIFI_SSID_MAX + 1], pass[WIFI_PASS_MAX + 1];
    const wifitelem::SetResult r = wifitelem::parseWifiSet(line + 9, ssid, pass);
    if (r == wifitelem::SET_USAGE) usbOut.println("WIFI usage: wifi set <ssid> <password> | wifi set \"<ssid>\" <password>");
    else if (r == wifitelem::SET_SSID_LENGTH) usbOut.println("WIFI refused: the SSID must be 1 to 32 bytes");
    else if (r == wifitelem::SET_PASS_LENGTH) usbOut.println("WIFI refused: the password must be 8 to 63 characters, 64 hex digits, or none");
    else if (!msc_fat_wifi_set(ssid, pass)) usbOut.println("WIFI NVS write failed");
    else {
      if (wifilink::started()) wifiCredentials(ssid, pass);
      usbOut.printf("WIFI stored ssid=\"%s\" password=%s%s\n", ssid, wifitelem::passwordState(pass[0] != 0),
                    radio_wifi ? "" : " (radio_mode is espnow: set radio_mode wifi, save, restart)");
      telemetryEvent(rec::EV_SETTING, 0, 0, "wifi set (value not logged)");
    }
    memset(pass, 0, sizeof(pass));
  } else {
    usbOut.println("WIFI usage: wifi | wifi scan | wifi test [seconds] | wifi set <ssid> <password> | wifi clear");
  }
}

// Answer a PING and send THRU frames. Runs in loop(), never in a callback.
void espnowService(){
  if (espnow_ping_pending) {
    uint8_t mac[6], buf[sizeof(espnow_ping_buf)];
    const uint8_t len = espnow_ping_len;
    memcpy(mac, espnow_ping_mac, 6);
    memcpy(buf, espnow_ping_buf, len);
    espnow_ping_pending = false;
    espnowEnsurePeer(mac);
    buf[0] = ESPNOW_TYPE_PONG;
    if (esp_now_send(mac, buf, len) == ESP_OK) espnow_pong_count++;
    else espnow_send_err++;
  }
  if (espnow_thru_active) {
    const uint32_t now = micros();
    if (now - espnow_thru_last_us >= espnow_thru_period_us) {
      // Keep the rate; after a long stall restart the schedule instead of sending a burst.
      espnow_thru_last_us = (now - espnow_thru_last_us > 10 * espnow_thru_period_us) ? now : espnow_thru_last_us + espnow_thru_period_us;
      espnowEnsurePeer(espnow_thru_mac);
      uint8_t frame[ESP_NOW_MAX_DATA_LEN] = {0};
      frame[0] = ESPNOW_TYPE_THRU;
      memcpy(&frame[1], &espnow_thru_seq, 4);
      memcpy(&frame[5], &now, 4);
      if (esp_now_send(espnow_thru_mac, frame, espnow_thru_size) != ESP_OK) espnow_send_err++;
      espnow_thru_seq++;  // one number per attempt: a refused send shows as a gap and in send_err
      if (--espnow_thru_left == 0) {
        espnow_thru_active = false;
        usbOut.printf("BENCH thru done sent=%lu send_err=%lu\n", (unsigned long)espnow_thru_seq, (unsigned long)espnow_send_err);
      }
    }
  }
}

void broadcastTelemetry(){
  if (   // Check if we should queue a telemetry message
      (millis() - telemetryBrodcastMillis >= 1000) // if at least 1 second since last broadcast
      | (stmPacket.items.charge_code != last_charge_code) //or charge code has changed
      | (stmPacket.items.status_bits != last_status_bits) //or status bits have changed
   ){
    String message = "- " +
      String(vehicleID) + " " +
      String(vehicleStatus) + " " +
      String(millis()) + " " +
      String(stmPacket.items.vbatt, 2) + " " +
      String(stmPacket.items.vcharge, 2) + " " +
      String(stmPacket.items.icharge, 2) + " " +
      String(charge_status) + " " +
      String(stmPacket.items.status_bits) + " " +
      String(stmPacket.items.charge_code) + " " +
      String(stmPacket.items.temperature) + " " +
      String(stmPacket.items.s1_radius) + " " +
      String(stmPacket.items.v5v, 2) + " " +
      String(steering_output) + " " +
      String(lapFlag)
      //+ " " + String(uart_overrun_count) // remove this after debugging stm <-> esp communication
      //+ " " + String(crc_error_count) // remove this after debugging stm <-> esp communication
      ;
    telemetryFifo_addMessage(message);  // Queue message instead of sending immediately
    telemetryBrodcastMillis = millis();
    lapFlag = false;  //reset lap flag after telemetry broadcast
  }
  last_charge_code = stmPacket.items.charge_code;
  last_status_bits = stmPacket.items.status_bits;
}

void telemetryFifo_addMessage(const String &message){ //Queue a telemetry message
  telemetryFifo[telemetry_fifo_head].message = message;
  telemetry_fifo_head++;
  if(telemetry_fifo_head == TELEMETRY_FIFO_LENGTH){
    telemetry_fifo_head = 0;
  }
}

void telemetryFifo_sendMessage(){ // Send one telemetry message from the queue (throttled to 100ms)
  // Only send if at least 100ms has passed since last send
  if(millis() - lastTelemetrySendTime >= 100){
    if(telemetry_fifo_head != telemetry_fifo_tail){
      // Queue has messages, send one
      broadcast(telemetryFifo[telemetry_fifo_tail].message);
      telemetry_fifo_tail++;
      if(telemetry_fifo_tail == TELEMETRY_FIFO_LENGTH){
        telemetry_fifo_tail = 0;
      }
      lastTelemetrySendTime = millis();
    }
  }
}


// setup(): the radio of the settings (radio_mode, espnow_enabled, telemetry_enabled).
void radioStart(){
  if (espnowWanted()) {
    TRACE(10);
    espnowStart();  // ESP-NOW Setup
  }
  if (wifiWanted()) wifiStart();  // radio_mode "wifi": station start; association runs on in the Wi-Fi task
}

// loop(): the radio part.
void radioService(){
  mark(SEC_RADIO);
  // 0.5.100: the radio part at most every RADIO_PERIOD_US (250 Hz). It cost 0.48-0.65 ms a pass, run on every pass
  // (about 27 % of the CPU at 2.35 ms frames). 250 Hz covers every send rate (telemetry, sweep records, answers).
  static uint32_t radio_last_us = 0;
  const bool radio_due = (uint32_t)(micros() - radio_last_us) >= RADIO_PERIOD_US;
  if (radio_due) radio_last_us = micros();
  if (radio_due && espnowWanted() && radioSlotOpen()) {
    espnowStart();  // no-op once started; starts it after a config.json change
    broadcastTelemetry();  // Queue telemetry message if needed
    telemetryFifo_sendMessage();  // Send one telemetry message from queue (throttled to 100ms)
    stage(ST_RADIO);
    espnowService();  // PONG answers and THRU test frames
    espnowCommands();  // config commands (CMD/ACK) and restarts
    monitorLinkService();  // MON frames: "monitor heard" and PC time events
    if (espnow_started) telemlog_poll(millis(), telemetrySend);  // live log frames, INFO, backfill
  }
  if (radio_due) wifiTestService(millis());  // "wifi test": its steps and lines
  if (radio_due && wifiWanted() && radioSlotOpen()) {  // radio_mode "wifi": the same frames as UDP; nothing here waits for the network
    wifiStart();  // no-op once started; starts it after a config.json change
    stage(ST_RADIO);
    wifilink::service(millis());  // association watch, reconnect, server beacon, mDNS answer, scan lines
    web::service(millis());  // starts the page server once the station is connected
    broadcastTelemetry();  // Queue telemetry message if needed
    wifiTelemetryFifoSend();  // ... and send one from the queue over UDP (throttled to 100ms)
    espnowCommands();  // config commands (CMD/ACK), STOP and restarts from the server, as over ESP-NOW
    webcfg::service(millis());  // a settings page request (src/web_settings.cpp)
    webtlm::service(millis());  // a telemetry page request: a copy of the newest values (src/web_telemetry.cpp)
    // Log frames only while a send can reach the server. telemlog moves its live position on a failed send, so a
    // poll before the server is known lost the first records of each start (event 19 "associated") and every record
    // of an association loss. Held back, the SLOW stream (events, status) goes out in order once the server is
    // known, as far as its RAM ring reaches (TELEMLOG_SLOW_SLOTS); the FAST stream skips to its newest frames.
    if (wifilink::ready()) telemlog_poll(millis(), wifiTelemetrySend);  // live log frames, INFO, backfill
  } else if (!radio_wifi) {
    wifilink::scanService();  // espnow mode: the lines of a "wifi scan"
  }
}
