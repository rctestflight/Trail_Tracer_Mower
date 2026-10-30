// i2c_nb: I2C master transfers that never hold the loop (ESP32-S2, controller I2C0, register level).
//
// The Arduino Wire library waits on the ESP-IDF driver for each transfer; arduino-esp32 2.0.17 (ESP-IDF 4.4) has no
// asynchronous driver. Here the loop puts a register read or write in a queue and returns. The controller runs the
// whole transfer from its command list (start, address and register, restart, address, the data, stop); the bytes
// fit its 32-byte FIFO. The transfer-done interrupt copies the bytes and the time, and starts the next queued job.
// The loop takes the newest result per tag.
//
// Errors: a NACK, the controller timeout (the bus held), lost arbitration, and a transfer that does not end in
// STALL_US are counted per type. After an error service() resets the controller and, when a device holds SDA low,
// gives up to 9 clocks and a stop (about 100 us, error case only), then starts the queue again. The failed job
// returns its status; it is not repeated.
#pragma once
#include <stdint.h>

namespace i2cnb {

enum Status : uint8_t { ST_NONE = 0, ST_OK, ST_NACK, ST_TIMEOUT, ST_ARB, ST_STALL };
const int MAX_READ = 16;  // bytes per read job
const int N_TAGS = 8;     // result slots: each job names the slot for its result
const uint32_t STALL_US = 3000;  // a transfer still on the bus after this is reset (a 14-byte read takes about 0.5 ms at 400 kHz)

struct Result {
  uint32_t seq;      // counts up with each result in this slot
  uint8_t status, n; // Status; bytes read
  uint8_t data[MAX_READ];
  int64_t t_us;      // esp_timer time at the end of the transfer
  uint32_t xfer_us;  // start to end of the transfer
};
struct Counts {
  uint32_t ok, nack, timeout, arb, stall;  // results by status
  uint32_t recover;                        // stuck-bus recoveries (SDA held low)
  uint32_t resets;                         // controller resets after an error
  uint32_t full;                           // jobs refused: queue full
  uint32_t isr_n, isr_cyc_max;             // transfer-done interrupts and the most CPU cycles in one
  uint64_t isr_cyc_sum;
  uint32_t xfer_us_max; uint64_t xfer_us_sum;  // OK transfers: start to end
};

// Takes the pins and controller I2C0 at hz. Gives the recovery clocks first if a device holds SDA low.
bool begin(int sda, int scl, uint32_t hz);
uint32_t clockHz();
// Queues a read of n bytes (1..MAX_READ) from register reg, or a one-byte register write. Starts it at once when the
// bus is free. false: the queue is full (counted) or n is out of range.
bool read(uint8_t tag, uint8_t addr, uint8_t reg, uint8_t n);
bool write(uint8_t tag, uint8_t addr, uint8_t reg, uint8_t val);
// A result for tag newer than *seen: copies it and sets *seen to its seq.
bool take(uint8_t tag, uint32_t* seen, Result* out);
// A job with this tag in the queue or on the bus.
bool pending(uint8_t tag);
// Loop: the stall watchdog, the reset after an error, the queue start after a reset. Returns at once otherwise.
void service();
void counts(Counts* c, bool reset);

}  // namespace i2cnb
