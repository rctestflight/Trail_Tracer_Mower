// The USB text queue: every USB print of the firmware goes through usbOut, so a host that stops reading never
// stops loop().
#pragma once
#include <Arduino.h>
#include "esp32-hal-tinyusb.h"
#include "usb_guard.h"


// USB text output that never blocks loop(). Serial.write() on the ESP32-S2 repeats until all
// bytes are in the 64-byte CDC transmit buffer while a host holds the port (DTR set), even with
// setTxTimeoutMs(0); a host that stops reading would then stop loop(). Every print goes into this
// queue instead (a print that does not fit is dropped and counted), and service() hands at most
// the free buffer space to Serial each loop().
// 0.5.2: back to Serial (the Arduino USB lock). 0.5.1 called tud_cdc_write() and
// tud_cdc_write_flush() from loop() without that lock, beside the Arduino USB task; after each
// warm restart (OTA end) the host disabled the port 1.3 s into enumeration and the board never
// came back on USB (bench test). A cold start worked.
class UsbOut : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* p, size_t n) override {
    if (n > sizeof(q_) - len_) { dropped++; return 0; }
    for (size_t i = 0; i < n; i++) q_[(head_ + len_ + i) % sizeof(q_)] = p[i];
    len_ += n;
    return n;
  }
  size_t room() const { return sizeof(q_) - len_; }
  void service() {
    // 0.5.62: USB is optional. With no host holding the port (DTR off), the queued text is dropped, so nothing
    // waits in the USB stack for a host, and a host that opens the port later gets live output only.
    if (!tud_cdc_connected()) {
      if (len_) { dropped++; head_ = 0; len_ = 0; }
      return;
    }
    int free = Serial.availableForWrite();  // the free CDC buffer: never more is handed over, so no write blocks
    while (free > 0 && len_ > 0) {
      size_t n = min((size_t)free, min(len_, sizeof(q_) - head_));
      Serial.write(q_ + head_, n);
      usb_guard::progress();  // the buffer took bytes: no stall
      head_ = (head_ + n) % sizeof(q_);
      len_ -= n;
      free -= (int)n;
    }
  }
  void drain(uint32_t ms) {  // before a restart only: wait up to ms for the queue to go out
    const uint32_t t0 = millis();
    while (len_ > 0 && millis() - t0 < ms) { service(); delay(1); }
  }
  uint32_t dropped = 0;
  size_t queued() const { return len_; }  // 0.5.104: bytes waiting (USB event text)
 private:
  uint8_t q_[1792];  // 0.3.8: 256 B less (dram0_0_seg); update mode keeps the output small
  size_t head_ = 0, len_ = 0;
};

/**
 * @brief The USB text output. Use it as Serial: usbOut.printf(...), usbOut.println(...).
 *
 * A line that does not fit the queue is dropped and counted (usbOut.dropped). With no PC holding the USB port,
 * the text is dropped.
 */
extern UsbOut usbOut;
