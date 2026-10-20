// The USB text queue (usbOut) and the USB link events.
// Part of the platform translation unit (src/rx_platform.cpp); shared declarations in platform_internal.h.

UsbOut usbOut;
bool usb_verbose = false;  // USB "verbose on": the bench lines each second (rover state, USB counters); off at start

// 0.5.104: USB link state changes as events (EV_USB), so a USB stop shows over the radio log (
// USB silent both ways for 3 min, the radio and loop() on; nothing on the radio said why).
void usbEventService(){
  {
    static uint32_t usb_ev_ms = 0, st0 = 0, re0 = 0, tx0 = 0, rx_quiet_n = 0;
    static int dtr0 = -1;
    static bool rx_quiet = false;
    const uint32_t rx_ms = usb_rx_ms;
    if (millis() - usb_ev_ms >= 1000) {
      usb_ev_ms = millis();
      const uint32_t st = usb_guard::stalls(), re = usb_guard::restarts(), tx = usb_guard::tx_events();
      const int dtr = tud_cdc_connected() ? 1 : 0;
      char t[80];  // an event body holds 73 B of text
      snprintf(t, sizeof(t), "stalls=%lu reconn=%lu tx/s=%lu q=%u drop=%lu dtr=%d rx_age=%lus",
               (unsigned long)st, (unsigned long)re, (unsigned long)(tx - tx0), (unsigned)usbOut.queued(),
               (unsigned long)usbOut.dropped, dtr, (unsigned long)((millis() - rx_ms) / 1000));
      if (st != st0) telemetryEvent(rec::EV_USB, 1, (int32_t)st, t);
      if (re != re0) telemetryEvent(rec::EV_USB, 2, (int32_t)re, t);
      if (dtr0 >= 0 && dtr != dtr0) telemetryEvent(rec::EV_USB, dtr ? 3 : 4, 0, t);
      const bool quiet = dtr && millis() - rx_ms >= 60000;
      if (quiet && !rx_quiet) telemetryEvent(rec::EV_USB, 5, (int32_t)++rx_quiet_n, t);
      rx_quiet = quiet;
      st0 = st; re0 = re; tx0 = tx; dtr0 = dtr;
    }
  }
}
