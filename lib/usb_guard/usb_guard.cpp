#include "usb_guard.h"
#include "USB.h"

namespace usb_guard {
namespace {

volatile uint32_t tx_count = 0;
uint32_t reconnect_count = 0;
uint32_t stall_count = 0;
uint32_t seen_tx = 0;
uint32_t last_progress_ms = 0;
bool detached = false;
uint32_t detach_ms = 0;
uint32_t grace_until_ms = 0;

void on_tx(void*, esp_event_base_t, int32_t, void*) { tx_count++; }

}  // namespace

void begin() {
  Serial.enableReboot(false);
  Serial.onEvent(ARDUINO_USB_CDC_TX_EVENT, on_tx);
}

void service(uint32_t now, bool) {
  if (detached) {
    if (now - detach_ms >= DETACH_MS) {
      tud_connect();
      detached = false;
      grace_until_ms = now + GRACE_MS;
      last_progress_ms = now;
    }
    return;
  }
  const uint32_t tx = tx_count;
  if (tx != seen_tx || !tud_mounted() || tud_suspended() || !tud_cdc_connected() ||
      tud_cdc_n_write_available(0) > 0 || (int32_t)(now - grace_until_ms) < 0) {
    seen_tx = tx;
    last_progress_ms = now;
    return;
  }
  if (now - last_progress_ms >= STUCK_MS) {
    stall_count++;
    reconnect_count++;
    tud_disconnect();
    detached = true;
    detach_ms = now;
  }
}

bool reconnecting(uint32_t now) { return detached || (int32_t)(now - grace_until_ms) < 0; }
uint32_t restarts() { return reconnect_count; }
uint32_t stalls() { return stall_count; }
uint32_t tx_events() { return tx_count; }

}  // namespace usb_guard
