// The platform parts of setup() and loop() in src/main.cpp, in the order main.cpp calls them.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

// ===== setup() =====

static LastState prev_state{};  // last_state of the boot before (valid when magic == LAST_STATE_MAGIC)

bool platformBegin(){
  // The ESP32 TX line to the STM32 (GPIO17) high at once: a low glitch while the ESP32 starts can
  // lock a blank STM32's ROM loader on a wrong baud rate (bench test).
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
#ifdef BOOT_TRACE
  Serial.begin(115200);
  for (uint32_t t0 = millis(); !tud_cdc_connected() && millis() - t0 < 4000;) delay(10);
  Serial.printf("TRACE start reset=%d\n", (int)esp_reset_reason());
  Serial.flush();
#endif
  cpuprof::begin();  // CPU profiler: loopTask is the task that runs setup()
  loop_task_handle = xTaskGetCurrentTaskHandle();  // 0.5.98: the frame edge wakes loop()
  const bool monitor_boot = monitor_once == MONITOR_ONCE_MAGIC && esp_reset_reason() == ESP_RST_SW;
  monitor_once = 0;  // never sticky
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const bool crash = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
    if (crash_magic != 0x5AFE0001u || !crash) { crash_magic = 0x5AFE0001u; crash_count = 0; }
    if (crash) crash_count++;
    safe_mode = crash_count >= 2;  // two crash resets, each within 60 s of its start: no telemetry, no rover mode, STM32 at 10 ms
    prev_state = last_state;  // the state before this reset (no power loss: RTC memory kept it)
    if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT) prev_state.magic = 0;
    last_state = LastState{LAST_STATE_MAGIC, 0, 0xFFFFFFFFu, ST_SETUP, 0, 0, 0};
  }

  // Initialise USB MSC + FAT filesystem and load config.
  TRACE(1);
  msc_fat_init(&g_deviceConfig);
  if (monitor_boot && !safe_mode) {
    // Monitor role, this boot only: the board is the PC monitor, not a vehicle. No vehicle hardware
    // starts (no CAN, no outputs, no STM32 link).
    monitor_role_on = true;
    last_state.magic = 0;  // the monitor role keeps no vehicle state
    monitor_role::setup(ROVER_FW_VERSION);
    rtcWdtStart();  // 0.6.7: the Monitor Rx too (fed at the top of loop()); a stop wedged it until a power cycle
    return true;
  }
  TRACE(2);
  return false;
}

void platformStart(){
  radio_wifi = g_deviceConfig.radio_mode == RADIO_MODE_WIFI;  // fixed for this boot: ESP-NOW or Wi-Fi, never both
  wifilink::attach(wifilink::Hooks{&usbOut, wifiEvent, wifiFrame});  // no radio action
  webcfg::begin();  // the settings page on the web server (Wi-Fi mode)
  if (!safe_mode) telemetryBegin();  // boot_id, the log rings (kept over a warm reset), the boot event

  // Start Serial
  TRACE(3);
  usb_guard::begin();  // before the host opens the port
  usb_guard::set_before_detach(msc_fat_eject);  // a guard reconnect (monitor role only) ejects the drive first: the DTR/RTS reboot sequence is off
  Serial.setRxBufferSize(2048);  // STM32 bridge: one IAP chunk frame is 1048 B
  Serial.begin(115200);  // Start USB VCP
  Serial.setTxTimeoutMs(0);  // Never block if no CDC host is connected
  usbOut.printf("BOOT rover-firmware %s reset=%d board=%s\n", ROVER_FW_VERSION, (int)esp_reset_reason(), RX_BOARD);  // banner: shows each reboot
  if (prev_state.magic == LAST_STATE_MAGIC) {  // what the board did before this reset
    char t[80];
    snprintf(t, sizeof(t), "stage=%s sec=%s mode=%u ap=%u heap_min=%lu crashes=%lu",
             STAGE_NAMES[prev_state.stage < 8 ? prev_state.stage : 0],
             prev_state.section < SEC_N ? SEC_NAMES[prev_state.section] : "?", prev_state.drive_mode, prev_state.ap_state,
             (unsigned long)prev_state.heap_min, (unsigned long)crash_count);
    usbOut.printf("PREV reset=%d uptime_ms=%lu %s\n", (int)esp_reset_reason(), (unsigned long)prev_state.uptime_ms, t);
    if (panicRecordTake(panic_line, sizeof(panic_line))) usbOut.printf("%s\n", panic_line);  // also on USB "boot"
    if (!safe_mode) telemetryEvent(rec::EV_PREV_BOOT, (int32_t)esp_reset_reason(), (int32_t)prev_state.uptime_ms, t);
  }
  usb_ota::begin(usb_ota::Hooks{&usbOut, otaMayStart, restartApp, otaNetReady});
  webupd::setup();  // the update page joins the web server (it starts in Wi-Fi mode once the station is connected)
  webtlm::begin();  // the telemetry page joins the web server
#ifdef BENCH_DRILL_PANIC
  // Bench drill 1: a new OTA image that crashes at boot. Expect the bootloader to start the
  // previous image. The drill panics once only: if this image starts again (no rollback), it runs
  // normally, so OTA still works and there is no lockout.
  {
    static RTC_NOINIT_ATTR uint32_t drill_magic;
    if (drill_magic != 0xD1A11001u) {
      drill_magic = 0xD1A11001u;
      usbOut.println("DRILL panic in setup (once)");
      usbDetachForRestart();
      abort();
    }
    drill_magic = 0;
    usbOut.println("DRILL FAIL: drill image started again, no rollback");
  }
#endif
  TRACE(4);
  Serial0.setTxBufferSize(1024);  // Prevents log writes from blocking at 1 Mbaud
  Serial0.begin(1000000);  // Start debug UART (visible on tag-connect)
  Serial1.setRxBufferSize(2048);  // ~176 ms headroom at 100 Hz × 116 bytes/packet
  Serial1.begin(2000000);  // Start ESP32 <-> STM32 UART
  Serial1.setRxFIFOFull(64);  // v2.0 layout 2 frame is 128 B = the hardware FIFO: interrupt at 64 B, not 120 B
  pinMode(STM_nDATA_READY, INPUT_PULLUP);
}

void canBegin(){
  TRACE(5);
  TRACE(6);
  can_init_ok = odriveCan_init();
}

void rcInputsBegin(){
  // Set up PWM inputs (in progress)
  uint8_t pwm_input_pins[] = {PWM_IN_1, PWM_IN_2, PWM_IN_3, PWM_IN_4};
  TRACE(7);
  pwm_read_rmt_init(pwm_input_pins, PWM_IN_NUM);
}

void setupDone(){
  TRACE(11);
  enableLoopWDT();  // loop() must run at least each 5 s (task watchdog, panic restart), as in the monitor role
  // 0.5.94: the RTC watchdog too. The task watchdog needs interrupts; a board that stops with interrupts off (or a
  // task above loop() that never yields) made no restart at all (no USB, no radio until a power
  // cycle). The RTC watchdog resets the system (not the RTC memory, so PREV and PANIC survive) if loop() is not
  // fed for 8 s. Fed at the top of each loop() pass.
  rtcWdtStart();
  boot_setup_ms = millis();
  usbOut.printf("SETUP ms=%lu\n", (unsigned long)boot_setup_ms);  // startup time: nothing above waits for a peer
}

// ===== loop() =====

bool platformLoopStart(){
  if (!rtc_wdt_test) rtc_wdt_feed();  // 0.5.94: the RTC watchdog (setup); bench "rtcwdt test" stops the feed
  if (monitor_role_on) { mark(SEC_MONITOR); monitor_role::loop(); return true; }
#ifdef OTA_TEST_CRASH
  // OTA test image (env ota_test_crash): a new image that panics 2 s after start in the vehicle boot. It panics only
  // while it runs "pending verify": if the bootloader did not roll back, it runs on normally and the next update works.
  {
    static int8_t crash_armed = -1;
    if (crash_armed < 0) {
      esp_ota_img_states_t st;
      crash_armed = esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
      usbOut.printf("OTA test crash image: %s\n", crash_armed ? "panic at 2 s" : "not pending verify, no panic");
    }
    if (crash_armed && millis() >= 2000) {
      usbOut.println("OTA test crash image: abort() now");
      usbOut.drain(200);
      abort();
    }
  }
#endif
  stage(ST_USB);
  mark(SEC_USB);
  // 0.5.101: the service parts run at set rates, not on every pass (cpu part shares, 2026-10-13: usb 5 %, log 4 %,
  // stmcmd 3 %, msc 1 %, bench 1.5 % of the CPU at 2.35 ms frames). Raw stream, bridge and update keep their own pace.
  static uint32_t usb_last_us = 0, svc_last_us = 0;
  loop_svc_due = due(svc_last_us, SERVICE_PERIOD_US);
  if (due(usb_last_us, USB_PERIOD_US) || raw_mode || stm_bridge || update_mode || stm_rs_phase == RS_RESTART || sf_on) {
    usbOut.service();  // queued USB text, never blocking
    stmCaptureService();  // bench: captured STM32 frames, printed at the USB pace
    usb_guard::service(millis(), vehicleStationary());  // USB output stop: a USB reconnect, no restart (lib/usb_guard)
    usbLoadService();  // bench: paced USB output
  }
  stmUrlService();  // "stm url": download result, time limits, progress lines
  webupd::loopService();  // the update page: an upload start, granted in loop() context
  if (stm_rs_phase == RS_RESTART) { mark(SEC_BRIDGE); stmRestartEnter(); return true; }  // command 41 passed: bridge to the boot checker
  if (sf_on) { mark(SEC_BRIDGE); stmFeedLoop(); return true; }  // "stm url": the ESP32 feeds the boot checker; nothing else runs
  stmRawStreamTimeout();  // 0.6.29: "stm raw" with no answer in 2 s ends
  if (raw_mode) { mark(SEC_BRIDGE); rawService(); return true; }  // raw stream pass-through or rate test: nothing else runs
  if (stm_bridge) { mark(SEC_BRIDGE); stmBridgeService(); return true; }  // STM32 app update through the boot checker: nothing else runs
  if (update_mode) {
    stage(ST_UPDATE);
    mark(SEC_UPDATE);
    // Update mode: from "ota begin" the board serves only USB and the update. No STM32 handling,
    // no telemetry, no radio, no flash log, no autopilot: a busy loop and a full output path made
    // the upload fail and the host lose the board (bench test). Ends with the restart into
    // the new image, or 120 s after the last "ota" line.
    if (millis() - update_last_ms < 120000 || usb_ota::busy()) {  // "ota url": update mode holds while the download runs
      // 0.6.17: during an "ota url" download (the app update) the web server and mDNS stop until the restart: their
      // memory goes to the Wi-Fi receive path. The largest free block fell to 2.3 KB and a download stalled at 517 of
      // 1100 KB. Not during a page upload: the web server carries that one.
      // 0.6.19: fetching(), not busy(): busy() is true during a page upload too, and the web server stop then waited for
      // the upload handler, more than 8 s (an RTC watchdog reset, or "no answer from the main loop"). mDNS stops for a page
      // upload too: the ESP32 image start needs a 4 KB block, and the largest free block was 2.3 KB.
      if (usb_ota::fetching() && web::running()) { web::stop(); usbOut.println("UPDATE web server stopped for the download"); }
      if (usb_ota::busy()) wifilink::mdnsStop();
      usb_ota::service();
      benchSerial();
      static uint32_t update_hb_ms = 0, drained = 0;
      // Radio commands go on ("ota status" during "ota url"); every datagram is read, as a held one holds a Wi-Fi
      // receive buffer (bench 2026-10-16: "ota url" stopped at 10.7 KB free heap).
      if (radio_wifi) drained += wifilink::pollRx(millis());
      espnowCommands();
      if (millis() - update_hb_ms >= 1000) {
        update_hb_ms = millis();
        usbOut.printf("HB ms=%lu update=1 drained=%lu heap=%lu\n", (unsigned long)millis(), (unsigned long)drained,
                      (unsigned long)ESP.getFreeHeap());
      }
      // 0.6.15: a wait every pass, not only while "ota url" downloads. The update page's upload runs in the web server
      // task, below loop(): with no wait it got no CPU, the ESP32 image stalled at 0 bytes after the Rx STM32 part and
      // the browser gave up. The download task (loop() priority) also gets the CPU here.
      delay(1);
      return true;
    }
    update_mode = false;
    enableLoopWDT();
    usbOut.println("UPDATE mode off: 120 s without an ota line; loop watchdog on");
  }

  mark(SEC_DRIVE);
  return false;
}

void logService(){
  stage(ST_LOG);
  mark(SEC_LOG);
  if (!safe_mode && loop_svc_due) {
    telemetryService();  // 1 s status record, events, USB log dump
    flashTestService();  // bench: flash erase/write stall measurement
  }
  if (crash_count && millis() > 60000) crash_count = 0;  // 60 s without a crash: not a crash loop
  if (loop_svc_due) usb_ota::service();  // a new OTA image: valid after 30 s, or after "ota confirm" (rollback at 120 s)
}

void benchService(){
  stage(ST_BENCH);
  mark(SEC_BENCH);
  static uint32_t bench_last_us = 0;
  if (Serial.available() || due(bench_last_us, SERVICE_PERIOD_US)) benchSerial();  // USB serial commands for the bench tests
  usbEventService();
  static uint32_t bench_print_ms = 0;
  if (millis() - bench_print_ms >= 1000) {
    bench_print_ms = millis();
    if (bench_timing_print) {  // the 1 s ESP-IDF log further down is commented out
      benchPrintCounters("TIMING", loop_exec_count, loop_min_us, loop_max_us);
      loop_min_us = 0xFFFFFFFF;
      loop_max_us = 0;
      loop_exec_count = 0;
    } else {
      // Heartbeat: keeps the USB link busy (an idle ESP32-S2 USB link can stop for good, see
      // bench tests) and shows that loop() runs and STM32 packets arrive.
      static uint32_t hb_last_packets = 0;
      if (safe_mode) usbOut.printf("SAFE MODE after %lu crash resets: no telemetry, no ESP-NOW\n", (unsigned long)crash_count);
      if (imu_halt) usbOut.println("IMU HALT: no drive (MPU6050 failed at start-up, imu_required=true)");
      // 0.5.97: packets = good Rx STM32 frames this second (was control passes; control runs at most 100 Hz now),
      // control = control passes; cpu, loop, drive = CPU load % of the last second (lib/cpu_prof load()).
      cpuprof::load((1u << SEC_DRIVE) | (1u << SEC_PACKET) | (1u << SEC_ROVER) | (1u << SEC_MPU), &cpu_busy_pct, &cpu_loop_pct, &cpu_drive_pct);
      static uint32_t hb_last_frames = 0;
      usbOut.printf("HB ms=%lu packets=%lu control=%lu mode=%d espnow=%d gyro=%d heap=%lu maxblk=%lu psram=%lu dtr=%d cpu=%u loop=%u drive=%u\n",
        (unsigned long)millis(), (unsigned long)(stmDec.st.frames - hb_last_frames),
        (unsigned long)((main_loop_counter - hb_last_packets + MAIN_LOOP_PERIOD) % MAIN_LOOP_PERIOD), (int)drive_mode_status, (int)espnow_started, (int)mpu_ok,
        (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)ESP.getPsramSize(),
        (int)tud_cdc_connected(), (unsigned)cpu_busy_pct, (unsigned)cpu_loop_pct, (unsigned)cpu_drive_pct);
      hb_last_frames = stmDec.st.frames;
      if (usb_verbose && (usbOut.dropped || usb_guard::restarts() || usb_guard::full_resets()))
        usbOut.printf("USB dropped=%lu guard_reconnects=%lu full_resets=%lu stalls=%lu\n", (unsigned long)usbOut.dropped,
                      (unsigned long)usb_guard::restarts(), (unsigned long)usb_guard::full_resets(),
                      (unsigned long)usb_guard::stalls());
      hb_last_packets = main_loop_counter;
      last_state.heap_min = ESP.getMinFreeHeap();
    }
  }
}

void loopEnd(){
  mark(SEC_MPU);
  monLedService();  // green LED: 100 ms inversion per MON frame
  // Process STM32 packets first, then do at most one sensor sample.
  serviceMpu();
  // 0.5.98: wait for the next frame edge, at most LOOP_WAIT_MS (0.5.99: 5 ms; with 1 ms the loop woke about 580 times
  // a second at 10 ms frames and spent about 1.4 ms of service polling each time), when nothing waits (no frame edge, no whole frame, no USB
  // input). loop() polled without a pause before, so the idle task never ran and the CPU load read 100 %; now the
  // load is the real work. Raw stream, bridge and monitor modes return earlier and keep polling.
  if (data_ready == 0 && Serial1.available() < stm_v2_len && !Serial.available()) {
    loop_waits++;
    ulTaskNotifyTake(pdTRUE, LOOP_WAIT_MS);
  }
}
