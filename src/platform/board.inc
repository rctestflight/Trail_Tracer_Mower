// Watchdog, crash counter, the record of the last state before a reset, start times, loop timing.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

extern "C" const char* rover_fw_version() { return ROVER_FW_VERSION; }  // for src/web_server.cpp
// Main loop configuration
#define MAIN_LOOP_PERIOD 360000  // 1hr rollover
#define RADIO_PERIOD_US 4000  // 0.5.100: the radio part runs at most at 250 Hz
#define USB_PERIOD_US 4000    // 0.5.101: USB output service at most at 250 Hz (4096 B CDC buffer: 1 MB/s at most)
#define SERVICE_PERIOD_US 10000  // 0.5.101: Rx STM32 link watch, USB drive, telemetry service, bench input: 100 Hz
// true once every period_us; last holds the time of the last true (0.5.101: services at set rates)
static inline bool due(uint32_t& last, uint32_t period_us) {
  const uint32_t t = micros();
  if ((uint32_t)(t - last) < period_us) return false;
  last = t;
  return true;
}
static bool loop_svc_due = false;  // the services at SERVICE_PERIOD_US run in this loop() pass (platformLoopStart)
#define LOOP_WAIT_MS 5  // 0.5.99: longest end-of-pass wait; a frame edge wakes loop() at once
uint32_t loop_waits = 0;  // 0.5.98: end-of-pass waits (bench "rt" prints them)
bool panicRecordTake(char* out, size_t n);  // src/panic_record.cpp
char panic_line[200] = "";
bool rtc_wdt_test = false;
uint8_t cpu_busy_pct = 0, cpu_loop_pct = 0, cpu_drive_pct = 0;  // 0.5.97: CPU load of the last second (HB line, status record)  // bench "rtcwdt test": the RTC watchdog is not fed  // the PANIC line of the boot before, if it panicked
uint32_t main_loop_counter;  // counter incremented on each new STM32 data packet
// Startup times for the USB "boot" command (the boot lines go out before a PC opens the port).
uint32_t boot_setup_ms = 0, boot_first_packet_ms = 0, boot_espnow_ms = 0;
bool monitor_role_on = false;    // this boot runs the monitor role: monitor_role::loop(), no vehicle code
// USB command "monitor": the next boot only runs the monitor role (PC-tethered monitor role). The flag
// is in RTC memory and is cleared at once at boot, so every board starts as a vehicle, and any later
// restart, crash or power cycle returns it to the vehicle role. The PC side sends "monitor" again.
static RTC_NOINIT_ATTR uint32_t monitor_once;
constexpr uint32_t MONITOR_ONCE_MAGIC = 0x4D4F4E31u;  // "MON1"
// Crash-loop guard: after 3 crash resets in a row (panic or watchdog), the firmware starts in a
// safe mode with no telemetry log and no ESP-NOW. USB, settings and "dld" stay, so a board whose
// new code crashes at boot can still be reflashed without the BOOT button. The counter is in RTC
// memory: a power-on clears it, and 60 s of running clears it.
RTC_NOINIT_ATTR uint32_t crash_magic;
RTC_NOINIT_ATTR uint32_t crash_count;
// The last state before a reset (struct LastState, stage() and mark() in rx_platform.h).
const char* const STAGE_NAMES[] = {"setup", "usb", "update", "packet", "radio", "log", "bench", "idle"};
RTC_NOINIT_ATTR LastState last_state;
const uint32_t LAST_STATE_MAGIC = 0x4C535431;  // "LST1"
const char* const SEC_NAMES[SEC_N] = {"usb", "update", "bridge", "monitor", "drive", "msc", "packet", "rover", "sample",
                                      "stmcmd", "radio", "log", "bench", "debug", "mpu"};
bool safe_mode = false;
// Void Setup --------------------------------------------------------------------------------------------------------------
#ifdef BOOT_TRACE
// Diagnostic build (env boot_trace): USB serial first, the reset reason, then one line per start-up
// step, each flushed, so the last line before an early restart shows the step.
#define TRACE(n) do { Serial.printf("TRACE %d ms=%lu\n", (n), (unsigned long)millis()); Serial.flush(); delay(20); } while (0)
#else
#define TRACE(n) do {} while (0)
#endif

// 0.5.94: the RTC watchdog resets the system (not the RTC memory, so PREV, PANIC and the Monitor Rx record survive) if
// loop() is not fed for 8 s, also with interrupts off or a task above loop() that never yields. 0.6.7: both roles.
void rtcWdtStart() {
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  rtc_wdt_set_time(RTC_WDT_STAGE0, 8000);
  rtc_wdt_enable();
  rtc_wdt_protect_on();
}

// Increment the main loop counter
void mainLoopCounter(){
  main_loop_counter++;
  if(main_loop_counter == MAIN_LOOP_PERIOD){
    main_loop_counter = 0;
  }
}
