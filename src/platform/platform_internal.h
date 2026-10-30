// The platform translation unit (src/rx_platform.cpp) includes this header, then the area files of src/platform/.
// The declarations below let each area file use what another one defines.
#pragma once
#include <Arduino.h>
#include <math.h>
#include <memory>        // 0.6.33: std::unique_ptr for bench buffers from the heap
#include <Wire.h>
#include <MPU6050_light.h>
#include "rt_prof.h"       // real-time path timing: STM32 frame -> control -> CAN (lib/rt_prof; bench "rt")
#include <driver/twai.h>
#include <driver/uart.h>  // raw stream test: UART loopback (uart_set_loop_back)
#include "hal/uart_ll.h"   // 0.5.126: the Rx UART frame from the frame edge interrupt (uart_ll_write_txfifo)
#include "wfrx_stm32_comms.h"
#include "rctf_wfrx_status_bits.h"
#include "rctf_wfrx_exp32.h"
#include "pwm_read_rmt.h"
#include "usb_msc_fat.h"
#include <WiFi.h>
#include <esp_wifi.h>  // bench RSSI experiment (promiscuous receive) and transmit power
#include <esp_now.h>
#include "esp_log.h"
#include "rover_autopilot.h"
#include "esp32-hal-tinyusb.h"  // usb_persist_restart(): restart into ROM download mode
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "usb_ota.h"    // OTA self-update over USB, with rollback (lib/usb_ota)
#include "usb_guard.h"  // no bootloader from DTR/RTS, USB transmit-stall re-enumeration (lib/usb_guard)
#include "monitor_role.h"  // USB command "monitor": this boot only, the same image runs as the PC monitor
#include "esp_partition.h"
#include "telemlog.h"       // numbered telemetry log: rings, live broadcast, backfill (monitor role)
#include "rover_records.h"  // record bodies: 100 Hz samples, status, events
#include "decimator.h"     // anti-alias filter and decimation of the samples
#include "tlm.h"           // table values: one line in include/telemetry_table.h defines a logged value (lib/tlm)
#include "stm_settings.h"  // STM32 commands: answer check, retry, read-back of the settings (lib/stm_settings)
#include "stm_v2.h"        // STM32 link version 2: 98 B frames, 8 B command frames with seq and CRC (lib/stm_v2)
#include <stdarg.h>
#include "freertos/task_snapshot.h"  // diag_cpu: bench "tasks" lists every task (pxTaskGetNext)
#include "cpu_prof.h"      // CPU share per FreeRTOS task and per loop part, by sampling (lib/cpu_prof; bench "cpu")
#include "nvs.h"            // bench "nvs": nvs_get_stats
#include "soc/rtc_wdt.h"     // 0.5.94: RTC watchdog (a stop with interrupts off resets the system)
#include "wifi_link.h"        // radio_mode "wifi": telemetry over a Wi-Fi network as UDP (lib/wifi_telem)
#include "mbedtls/pkcs5.h"     // the WPA2 key from the passphrase (wifiKeyFromPassphrase)
#include "wifi_telem_proto.h" // the "wifi set" arguments and the log text of a command (lib/wifi_telem)
#include "web_server.h"         // 0.5.116: pages on the ESP32 in Wi-Fi mode (src/web_server.cpp)
#include "web_settings.h"       // 0.5.116-webcfg: settings page /settings (src/web_settings.cpp)
#include "web_update.h"         // 0.5.116: the update page /update (src/web_update.cpp)
#include "web_telemetry.h"      // 0.5.117: the telemetry page /telemetry (src/web_telemetry.cpp)
#include "tlm_page.h"           // 0.5.117: the telemetry page fields and values from the telemetry table
#include "firmware_version.h"   // ROVER_FW_VERSION, RX_BOARD
#include "rx_platform.h"        // what src/main.cpp uses
#include "vehicle_values.h"     // what the platform uses from src/main.cpp

static const char* TAG = "wfrx";

// Steps of the Rx STM32 calibration copy (updates.inc) and of the Rx STM32 restart command 41 (updates.inc)
enum { CAL_IDLE, CAL_READ, CAL_WRITE, CAL_SAVE, CAL_VERIFY };
enum { RS_IDLE, RS_CAL, RS_STOP, RS_START, RS_DATA, RS_CHECK, RS_RESTART };

// board.inc: watchdog, crash counter, start times
static inline bool due(uint32_t& last, uint32_t period_us);
extern uint32_t loop_waits;
extern char panic_line[200];
extern bool rtc_wdt_test;
extern uint8_t cpu_busy_pct, cpu_loop_pct, cpu_drive_pct;
extern uint32_t main_loop_counter;
extern uint32_t boot_setup_ms, boot_first_packet_ms, boot_espnow_ms;
extern bool monitor_role_on;
extern uint32_t crash_magic;
extern uint32_t crash_count;
void rtcWdtStart();
void mainLoopCounter();

// usb.inc: USB text queue
extern UsbOut usbOut;

// settings.inc: settings store and erase gesture
extern bool erase_window;
extern uint32_t btn_window_once;
extern bool erase_hold;
extern uint32_t erase_hold_ms;
extern bool btn0_sim;

// imu.inc: gyro
extern bool mpu_ok;
extern int16_t imu_roll16;
extern double gyro_heading_deg;
void mpuReport(bool ok, unsigned status);

// motors.inc: motor controllers and servo outputs
extern uint32_t odrive_heartbeat_millis[ODRIVE_NODE_COUNT];
extern uint8_t odrive_axis_states[ODRIVE_NODE_COUNT];
extern float odrive_node0_vbus;
extern float odrive_node0_ibus;
extern float odrive_pos_rev[ODRIVE_NODE_COUNT];
bool odriveCan_init();
void odrive_printErrorNames(uint32_t errors);
extern uint32_t pwm_test_us;

// rx_link.inc: Rx STM32 link and frame decode
extern volatile uint32_t data_ready;
extern TaskHandle_t loop_task_handle;
extern volatile uint32_t stm_edge_us;
extern uint32_t uart_overrun_count;
extern bool stm_v2_resync;
extern uint32_t uart_skip_catchup;
extern uint32_t stm_catchup_frames;
extern uint32_t control_last_us;
extern uint32_t crc_error_count;
extern int stm_v2_len;
extern stmset::Link stmLink;
extern uint8_t rt_last_frame[STM_FRAME_LEN];
extern bool rt_last_ok;
extern uint64_t rt_read_cyc, rt_dec_cyc, rt_ans_cyc, rt_post_cyc;
extern uint32_t rt_read_max, rt_dec_max, rt_bench_n;
extern uint8_t stm_link_ver;
extern uint32_t stm_bad_frames_run;
extern bool stm_v2_cfg_sent;
extern uint8_t stm_v2_lanes;
extern bool stm_v2_save_ok;
extern bool stm_v2_autorun;
extern uint32_t stm_v2_run_ms;
extern uint32_t stm_ans_print_until;
extern uint8_t* stm_cap;
extern uint16_t stm_cap_n, stm_cap_want, stm_cap_out;
extern uint32_t stm_stream_end, stm_stream_n, stm_stream_drop;
extern bool stm_cap_dump;
extern uint32_t stm_rtt_hist[16];
extern uint8_t stm_dup_frame[ESP_FRAME_LEN];
extern int stm_dup_step;
extern uint32_t stm_last_packet_ms;
extern bool stm_rom_used;
extern uint32_t stm_iap_boots;
extern uint32_t stm_iap_fails;
static void iap_encode_frame(uint16_t cid, uint8_t *buf);
static bool iap_read_frame(uint32_t timeout_ms, uint16_t *cid_out, uint8_t *payload_out, uint8_t *payload_len_out);
void stm32_boot_handshake();
size_t serial1Drain();
void stmFastFrameGuard();
int stm32_iap_boot(int hello_tries, char* hello_text, size_t text_len, uint8_t* hello_len);
void stmRecoveryService();
void receiveStmUart();
static uint16_t crc16_ibm(const uint8_t *data, size_t len);
extern bool uart_force_old;
extern uint32_t uart_pause_until;
extern uint32_t uart_frames, uart_frames_cmd, uart_frames_bad, uart_frames_old;
extern uint8_t uart_blk[2][ESP_UART_FRAME_LEN];
extern uint8_t uart_cmdf[ESP_UART_FRAME_LEN];
extern volatile uint8_t uart_blk_i;
extern volatile bool uart_blk_ok;
extern volatile bool uart_cmd_ready;
extern volatile bool uart_isr_go;
extern volatile uint32_t uart_isr_none;
extern volatile uint32_t uart_isr_fifo;
extern volatile uint32_t uart_isr_repeat;
extern volatile uint32_t uart_edge_seq;
extern volatile bool uart_blk_sent;
void stmTxBlockOnly();
void uartCommand(const char* a);
void espPacketFifo_sendCmd();
extern uint32_t sweep_updates, sweep_sent, sweep_dropped;
extern int32_t sweep_sent_bins;
extern uint32_t polar_frames, polar_samples, polar_sent, polar_dropped, polar_lost_flags;
extern const char* dsp_apply_reason;
void dspApply();
void dspOnAnswer(const stmv2::Answer& a);
void stmV2Config();
int stmFrameLen();
bool stmReadFrame();
void stmLinkEvents();

// radio.inc: ESP-NOW, Wi-Fi, text telemetry
extern int32_t radio_slot_us;
extern const uint32_t RADIO_SLOT_LEN_US;
extern uint32_t radio_tx_hist[11];
extern bool espnow_started;
extern volatile uint32_t espnow_bad_key;
extern volatile uint32_t espnow_rx_count, espnow_ping_count, espnow_ping_missed;
extern volatile uint32_t espnow_tx_ok, espnow_tx_fail;
extern uint32_t espnow_send_err, espnow_pong_count;
extern bool espnow_thru_active;
extern uint32_t espnow_thru_period_us, espnow_thru_size, espnow_thru_left, espnow_thru_seq, espnow_thru_last_us;
extern uint8_t espnow_thru_mac[6];
extern volatile uint8_t sim_loss_tx, sim_loss_rx;
extern volatile uint32_t sim_dropped_tx, sim_dropped_rx;
extern bool pc_time_known;
extern int64_t pc_offset_ms;
extern volatile bool espnow_cmd_pending;
extern uint8_t espnow_cmd_mac[6];
extern uint16_t espnow_cmd_seq;
extern char espnow_cmd_text[];
extern volatile bool espnow_busy_pending;
extern uint8_t espnow_busy_mac[6];
extern uint16_t espnow_busy_seq;
extern volatile uint8_t espnow_restart_request;
extern volatile int espnow_list_index;
extern bool espnow_list_saved;
extern DeviceConfig* g_saved_config;  // nullptr until the first "list saved"
DeviceConfig* savedConfigLoad();      // load the settings of the next start into g_saved_config; nullptr: no heap
extern volatile bool espnow_stop_request;
extern uint8_t espnow_stop_mac[6];
extern uint16_t espnow_stop_seq;
extern uint32_t espnow_list_ms;
extern bool radio_wifi;
static int linkModeOf(const DeviceConfig& c, bool wifi);
static int periodFloorUs(int m, const DeviceConfig& c);
static const char* linkModeName(int m);
static int linkModeStrict(const DeviceConfig& c);
bool espnowWanted();
bool wifiWanted();
bool radioSlotOpen();
uint8_t monitorAgeS();
void monitorLinkService();
void espnowStart();
void espnowEnsurePeer(const uint8_t* mac);
void espnowSendAck(const uint8_t* mac, uint16_t seq, uint8_t result, const char* text);
void wifiEvent(int32_t a, int32_t b, const char* text);
void wifiFrame(const uint8_t* data, int len);
extern uint32_t web_upload_quiet_ms;
bool wifiTelemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len);
void wifiStart();
void wifiTelemetryFifoSend();
void wifiUsbCommand(const char* line);
void espnowService();
void broadcastTelemetry();
void telemetryFifo_addMessage(const String &message);
void telemetryFifo_sendMessage();

// telemetry.inc: telemetry log
extern uint32_t telem_loop_max_us;
extern int32_t dt_decoded;
extern uint32_t dt_decoded_ms;
void telemetryBins(int inputs = 1);  // inputs: one per 10 ms of update period (1 at 10 ms)
void dtTelemetry();
void dtPollService();
extern bool bench_moving;
bool vehicleStationary();
bool telemetryEraseAllowed();
uint8_t telemetryApByte();
bool telemetrySend(const uint8_t mac[6], const uint8_t* buf, size_t len);
void telemetryEvent(uint16_t code, int32_t a, int32_t b, const char* text);
bool telemetryLogStarted();  // 0.6.34: the log takes events (after the boot event)
void telemetryBegin();
void telemetrySample();
void telemetryUsbCommand(const char* line);
void telemetryService();

// commands.inc: settings and run commands
void espnowCommands();
uint8_t runCommand(bool start, const char* source, String& reply);
uint8_t configCommand(const char* t, bool over_espnow, String& reply);
extern bool period_ack_want;
extern uint32_t period_ack_answers0, dsp_period_answers;
void periodAckDefer(bool usb, const uint8_t* mac, uint16_t seq);
void periodAckService();
void binPollService();
uint8_t configCommandRun(const char* t, bool over_espnow, String& reply);

// updates.inc: restarts and updates
extern bool update_mode;
extern uint32_t update_last_ms;
void usbDetachForRestart();
void restartIntoRom();
void restartApp();
bool otaMayStart();
bool otaNetReady();
extern bool stm_raw;
extern bool stm_bridge;
extern uint32_t stm_bridge_start_ms, stm_bridge_rx_ms, stm_bridge_up, stm_bridge_down;
bool stmIapHoldTake();
void stmIapHoldSet();
extern uint8_t stm_cal_phase;
extern bool stm_cal_then_hold;
extern uint32_t stm_cal_val[];
extern bool stm_cal_valid;
void stmCalLoad();
bool stmCalPending();
void stmCalSetPending(bool on);
void stmCalStart(uint8_t phase);
void stmCalOnAnswer(const stmv2::Answer& a);
void stmCalService(uint32_t now);
void stmBridgeService();
extern uint8_t stm_rs_phase;
void stmRestartCommand(const char* arg);
void stmRestartBegin();
void stmRestartOnAnswer(const stmv2::Answer& a);
void stmRestartService(uint32_t now);
void stmRestartEnter();
extern bool sf_on;
bool stmUrlBusy();
bool stmUrlOn41Pass();
void stmUrlOn41Fail(const char* why, long a, long b);
void stmUrlService();
extern uint32_t rx_stm32_ver, rx_stm32_ver_link_ms, rx_stm32_ver_ask_ms;
extern bool rx_stm32_ver_ok;
void stmVersionService(uint32_t now);
bool updateAllowed(const char* t);
bool updateCommand(const char* t, bool radio, String& line);
uint8_t updateResult(const String& line);
void stmFeedLoop();

// bench.inc: bench commands
void usbLoadService();
void stmStreamFrame(const uint8_t* f);
extern uint32_t stm_acc_want, stm_acc_n, stm_acc_ovl, stm_acc_lost;
extern uint32_t stm_coil_ms, stm_coil_t0, stm_coil_n;
extern float stm_bin_hz[WFRX_BINS];
void stmCoilFrame();
extern bool scan_on;
void scanFrame();
void stmAccFrame();
extern bool bench_timing_print;
void benchPrintCounters(const char* tag, uint32_t loops, uint32_t min_us, uint32_t max_us);
extern uint8_t raw_mode;
extern uint8_t raw_stm_state;
// 0.6.29: no byte may go to the Rx STM32: command 42 went (state 0xFD: the Rx STM32 ends a stream on any byte, and
// the Rx UART frame goes at each frame edge, also at the edge of the answer frame) or the stream runs (1, 2). While
// "stm raw" waits to send (0xFE) the idle mode and CID 42 commands must go out: up to 0.6.28 the send paths tested
// raw_stm_state != 0, so these commands never went and every later command waited until a restart.
inline bool rawStreamRuns() { return raw_stm_state == 1 || raw_stm_state == 2 || raw_stm_state == 0xFD; }
void stmRawStreamCommand(const char* arg);
void stmRawStreamOnAnswer(const stmv2::Answer& a);
void rawService();
void stmRawStreamTimeout();
void stmRawService();
extern uint32_t usb_rx_ms;
void benchSerial();
void stmCaptureService();
void flashTestCommand(const char* arg);
void flashTestService();
