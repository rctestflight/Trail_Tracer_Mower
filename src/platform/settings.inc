// The settings store in RAM (g_deviceConfig), the config.json check and the settings erase gesture.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

// Erase gesture: press the 0 button within the first 3 s after power-on and hold it 5 s. Every
// setting and the Wi-Fi details go to their defaults, then the board restarts. A press held at power-on starts ROM
// download mode instead (the chip reads GPIO0 at reset), so the press must start after power-on. The LED flashes at
// 10 Hz during the hold. Outside the window, and after a release before 5 s, the button acts as before.
bool erase_window = false;       // setup(): this start is a power-on, or "btn0 window" asked for it
RTC_NOINIT_ATTR uint32_t btn_window_once;  // "btn0 window": the next software restart opens the erase window
#define BTN_WINDOW_MAGIC 0x42574E44u
bool erase_hold = false;         // the gesture owns the button
uint32_t erase_hold_ms = 0;
#if 1
bool btn0_sim = false;           // bench: USB "btn0 down" / "btn0 up" stands for the button (the bench cannot press it)
#endif
// USB MSC / config management
DeviceConfig g_deviceConfig;                     // Runtime config struct (MSC + NVS backed)

// setup(): the erase window opens on a power-on, or after "btn0 window" and a software restart.
void eraseWindowOpen(){
  erase_window = esp_reset_reason() == ESP_RST_POWERON || esp_reset_reason() == ESP_RST_BROWNOUT ||
                 (btn_window_once == BTN_WINDOW_MAGIC && esp_reset_reason() == ESP_RST_SW);
  btn_window_once = 0;
}

// The button, pressed = 1.
uint8_t buttonRead(){
  uint8_t gpio_state = !digitalRead(BUTTON_PIN);
  if (btn0_sim) gpio_state = 1;  // bench: USB "btn0 down"
  return gpio_state;
}

// Erase gesture: a press that starts in the first 3 s after power-on
int buttonEraseGesture(uint8_t gpio_state){
  if (!erase_hold && erase_window && gpio_state && millis() < 3000) {
    erase_hold = true;
    erase_hold_ms = millis();
    usbOut.printf("BUTTON erase gesture: hold 5 s to erase every setting and the Wi-Fi details\n");
    buttonEvent(BTN_ERASE_START, 0, "erase hold start (power-on press)");
  }
  if (erase_window && millis() >= 3000 && !erase_hold) erase_window = false;
  if (erase_hold) {
    if (!gpio_state) {  // released before 5 s: nothing happens, and the press makes no click or calibration
      erase_hold = false;
      erase_window = false;
      usbOut.printf("BUTTON erase gesture: released after %lu ms, nothing erased\n", (unsigned long)(millis() - erase_hold_ms));
      buttonEvent(BTN_ERASE_CANCEL, millis() - erase_hold_ms, "erase cancelled: released before 5 s");
      return ERASE_RELEASED;
    }
    digitalWrite(nLED, (millis() / 50) & 1);  // 10 Hz flash during the hold
    if (millis() - erase_hold_ms >= 5000) {
      buttonEvent(BTN_ERASE_DONE, millis() - erase_hold_ms, "erase: every setting and the Wi-Fi details; restart");  // can be lost in the restart
      usbOut.drain(100);
      const bool ok = msc_fat_erase(&g_deviceConfig, true, false);
      usbOut.printf("BUTTON erase: every setting and the Wi-Fi details at defaults%s; restart\n",
                    ok ? "" : " (config.json write failed)");
      usbOut.drain(200);
      restartApp();
    }
    return ERASE_HOLD;  // the gesture owns the button: no click, no calibration, no 6 s action
  }
  return ERASE_NONE;
}

// Check if USB MSC host has finished editing config.json (3 s idle window).
bool settingsFileChanged(){
  mark(SEC_MSC);
  return loop_svc_due && msc_fat_loop(&g_deviceConfig);
}

// A value from config.json or NVS that took its default or a limit, and (0.6.30) a key that config.json changed: one
// USB line and one event each
void settingsBadValues(){
  if (loop_svc_due) {
    char bad[96];
    int src;
    while (msc_fat_bad_value_pop(bad, sizeof(bad), &src)) {
      usbOut.printf("SETTING bad value %s\n", bad);
      if (src != 2) telemetryEvent(rec::EV_SETTING, 2, 2, (String("bad value ") + bad).c_str());
    }
    while (msc_fat_change_pop(bad, sizeof(bad))) {
      usbOut.printf("SETTING changed %s\n", bad);
      telemetryEvent(rec::EV_SETTING, 0, 2, bad);
    }
  }
}
