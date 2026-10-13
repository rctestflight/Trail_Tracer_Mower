#include "usb_guard.h"
#include "USB.h"
#include "soc/usb_struct.h"

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
uint32_t kick_count = 0;   // flushes sent to a full TX FIFO with no progress (KICK_MS)
bool kicked = false;
bool reconnect_on = false; // set_reconnect(): the monitor role after the PC program connected; never the vehicle role

void on_tx(void*, esp_event_base_t, int32_t, void*) { tx_count++; }

constexpr int EPN = 7;  // ESP32-S2 USB OTG: endpoints 0 to 6
struct Snap {
  uint32_t ms, gintsts, gintmsk, dctl, dsts, daint, daintmsk, empmsk;
  uint32_t in_ctl[EPN], in_siz[EPN], in_fs[EPN], in_int[EPN], out_ctl[EPN], out_siz[EPN], out_int[EPN];
  uint32_t cdc_tx_free, cdc_rx_avail, arduino_rx_avail, tx_events;
  bool valid;
} snap{};

}  // namespace

void snapshot() {
  snap.ms = millis();
  snap.gintsts = USB0.gintsts; snap.gintmsk = USB0.gintmsk; snap.dctl = USB0.dctl; snap.dsts = USB0.dsts;
  snap.daint = USB0.daint; snap.daintmsk = USB0.daintmsk; snap.empmsk = USB0.dtknqr4_fifoemptymsk;
  for (int n = 0; n < EPN; n++) {
    snap.in_ctl[n] = USB0.in_ep_reg[n].diepctl; snap.in_siz[n] = USB0.in_ep_reg[n].dieptsiz;
    snap.in_fs[n] = USB0.in_ep_reg[n].dtxfsts; snap.in_int[n] = USB0.in_ep_reg[n].diepint;
    snap.out_ctl[n] = USB0.out_ep_reg[n].doepctl; snap.out_siz[n] = USB0.out_ep_reg[n].doeptsiz;
    snap.out_int[n] = USB0.out_ep_reg[n].doepint;
  }
  snap.cdc_tx_free = tud_cdc_n_write_available(0); snap.cdc_rx_avail = tud_cdc_n_available(0);
  snap.arduino_rx_avail = Serial.available(); snap.tx_events = tx_count;
  snap.valid = true;
}

bool snapshot_valid() { return snap.valid; }

namespace {
constexpr uint32_t SAVED_MAGIC = 0x55524547;  // "UREG"
RTC_NOINIT_ATTR char saved_line[900];
RTC_NOINIT_ATTR uint32_t saved_magic;
}  // namespace

void save_for_next_boot(const char* line) {
  strncpy(saved_line, line, sizeof(saved_line) - 1);
  saved_line[sizeof(saved_line) - 1] = 0;
  saved_magic = SAVED_MAGIC;
}

bool take_saved(char* buf, int n) {
  if (saved_magic != SAVED_MAGIC) return false;
  saved_magic = 0;
  strncpy(buf, saved_line, n - 1);
  buf[n - 1] = 0;
  return true;
}

int format_snapshot(char* buf, int n) {
  int k = snprintf(buf, n, "usbregs ms=%lu gintsts=%08lx gintmsk=%08lx dctl=%08lx dsts=%08lx daint=%08lx daintmsk=%08lx empmsk=%04lx"
                   " cdc_tx_free=%lu cdc_rx=%lu ard_rx=%lu txev=%lu",
                   (unsigned long)snap.ms, (unsigned long)snap.gintsts, (unsigned long)snap.gintmsk, (unsigned long)snap.dctl,
                   (unsigned long)snap.dsts, (unsigned long)snap.daint, (unsigned long)snap.daintmsk, (unsigned long)snap.empmsk,
                   (unsigned long)snap.cdc_tx_free, (unsigned long)snap.cdc_rx_avail, (unsigned long)snap.arduino_rx_avail,
                   (unsigned long)snap.tx_events);
  for (int e = 0; e < EPN && k < n; e++)
    k += snprintf(buf + k, n - k, " in%d=%08lx/%08lx/%04lx/%04lx", e, (unsigned long)snap.in_ctl[e], (unsigned long)snap.in_siz[e],
                  (unsigned long)snap.in_fs[e], (unsigned long)snap.in_int[e]);
  for (int e = 0; e < EPN && k < n; e++)
    k += snprintf(buf + k, n - k, " out%d=%08lx/%08lx/%04lx", e, (unsigned long)snap.out_ctl[e], (unsigned long)snap.out_siz[e],
                  (unsigned long)snap.out_int[e]);
  return k < n ? k : n - 1;
}

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
    kicked = false;
    return;
  }
  if (!reconnect_on) return;  // vehicle role, or monitor before the PC program connected: drop output, touch nothing
  // A full TX FIFO with no completed transfer for KICK_MS: one flush first. The IN transfer of the data in the
  // FIFO may not have started (monitor role at about 14 KB/s, 2026-10-12); a flush starts it, with no reconnect.
  if (!kicked && now - last_progress_ms >= KICK_MS) {
    kicked = true;
    kick_count++;
    tud_cdc_n_write_flush(0);
  }
  if (now - last_progress_ms >= STUCK_MS) {
    snapshot();  // the USB core state at the stop, before the reconnect resets it
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
uint32_t kicks() { return kick_count; }
void set_reconnect(bool on) { reconnect_on = on; }
bool reconnect_allowed() { return reconnect_on; }

}  // namespace usb_guard
