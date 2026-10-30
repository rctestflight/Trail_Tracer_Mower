// stm_settings: the command link from the ESP32 to the STM32 application, with reply checks.
// No Arduino dependency: builds on a PC (test/telemetry_host/stmset_test.cpp).
//
// The ESP32 sends an 8-byte command (u32 command ID, 32-bit value). Bit 31 of the ID is the write
// bit. The STM32 application (TrailTracer-Rx-STM32 RCTF_TT_RX_REVA_G070CBx, rctf_wfrx_stm_comms.c)
// runs one command in each 10 ms cycle and answers in the packet of that cycle:
//   - WFRX_STATUS_CMD_ACK (status bit 0) is set in the packet of the cycle that ran a command, and
//     clear in a cycle with no command.
//   - rid = the command ID as sent, for every write and for a read of 1, 2 and 3. A read of 6, 7 and
//     8 does not set rid: it holds 0, or the ID of the command of the cycle before.
//   - response: write 1, 2: the channel; write 3: 1; write 4, 5: the new angle offset (float);
//     write 6, 7: the stored value (float, at most 16.0 V and 6.0 A); write 8: the stored limit in
//     10 ms cycles (min(value, 3600) x 100); write 9: the value. Read 1, 2: the channel; read 6, 7:
//     the stored float; read 8: the stored limit in s. A read of 4, 5 or 9 gets no answer.
// The STM32 holds commands 1, 2, 6, 7 and 8 in RAM only; after an STM32 start they are 0.
//
// This link sends one command and waits for its answer before it sends the next. So the first
// packet with CMD_ACK after a send is the answer, also for a read of 6, 7 and 8 that has no rid.
// A command with no answer is sent again (at most MAX_TRIES sends), except 3, 4 and 5: a second 3
// is a second STM32 flash write, and 4 and 5 add the angle again. Each failure is counted and
// becomes an event. The link keeps the value that each write of 1, 2, 6, 7 and 8 gave the STM32,
// reads one of the five back every CHECK_PERIOD_MS, and sends all five again when a read-back
// differs, or when packets come back after a gap of GAP_MS or more (an STM32 start loses them).
//
// Link version 2 (include/rctf_wfrx_comms_v2.h): setV2(true). The command frame carries seq and a CRC
// (the firmware encodes it from next() and seq()); the answer is resp_seq, resp_cid, resp_status and
// resp_value (onAnswerV2). A command with no answer goes again with the same seq after
// REPLY_TIMEOUT_V2_MS; the STM32 answers a repeat within 10 frames from its stored answer and does
// not run it again, so 4 and 5 are retried too (3 is not: its frame stop can pass the 10 frames).
// Values in v2: 1, 2 = loop words (the v1 channel value becomes a loop of length 1); 8 = seconds in
// both forms; 11, 12 = detail formats and 13 = lane inputs, tracked as settings like 1, 2, 6, 7, 8.
// Status bit 21 (STM32 reset) sends all tracked settings again.
// 0.5.91, v2 only (rule: a setting goes to the Rx STM32 only on a change or a user command): no periodic
// read-back, and no resend after a packet gap or a read-back difference. A write of 13 or 18 with an unchanged value
// restarted the Rx STM32 DSP (2.0.54), and frame gaps at 2.35 ms made the link resend them. The resend at status
// bit 21 (an Rx STM32 start: its settings are gone) stays. A v1 STM32 keeps the old behaviour.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace stmset {

const uint32_t WRITE = 0x80000000u;
const uint32_t STATUS_CMD_ACK = 1u;      // WFRX_STATUS_CMD_ACK
const float VSTOP_MAX_V = 16.0f;         // WFRX_CHARGE_VOLTAGE_MAX
const float IMAX_MAX_A = 6.0f;           // WFRX_CHARGE_CURRENT_MAX
const int32_t TIME_MAX_S = 3600;         // WFRX_CHARGE_TIME_MAX_SECONDS

const uint32_t REPLY_TIMEOUT_MS = 100;   // 10 packets at 10 ms (setPeriodRatio: the same packets at any update period)
const uint32_t REPLY_TIMEOUT_V2_MS = 35; // v2: 3 frames, so the third send stays in the 10-frame repeat window (at 10 ms;
                                         // setPeriodRatio scales it)
const uint32_t SAVE_TIMEOUT_MS = 300;    // command 3: STM32 flash page erase and write
const uint8_t MAX_TRIES = 3;             // sends of one command, the first included
const uint32_t CHECK_PERIOD_MS = 2000;   // one read-back each 2 s: the five settings in 10 s
const uint32_t CHECK_PERIOD_MAX_MS = 60000;  // a read-back with no answer doubles the period, to this
const uint32_t GAP_MS = 500;             // no valid packet this long, then packets again: send all five
const int QUEUE_LEN = 128;               // at start: calibration reads (32), v1 settings, layout 2 reads, DSP settings (42)
const int EVENT_LEN = 8;

// Event kinds (vehicle log event 18, field a)
enum Kind : int32_t {
  K_NO_REPLY = 1,    // no answer after the last send; b = command ID
  K_BAD_REPLY = 2,   // an answer with a different value; b = command ID
  K_MISMATCH = 3,    // a read-back differs from the value sent; all five go again; b = command ID
  K_GAP = 4,         // packets again after a gap; all five go again; b = gap in ms
  K_QUEUE_FULL = 5,  // a command was dropped; b = command ID
  K_IAP_BOOT = 6,    // set by the firmware, not by this link: STM32 silent, HELLO/BOOT sent again
  K_STM_RESET = 7,   // v2 status bit 21: the STM32 started again; all tracked settings go again
  K_REFUSED = 8,     // v2: the STM32 refused the command (b = command ID, got = resp_status)
  K_CAL_RESTORE = 9, // set by the firmware: calibration copy written back after an IAP update; b = values read back equal (of 32)
};

struct Event {
  int32_t kind = 0;
  uint32_t cid = 0;     // command ID, or the gap in ms for K_GAP
  uint32_t got = 0;     // raw 32 bits of the answer
  uint32_t want = 0;    // raw 32 bits that were expected
  uint8_t tries = 0;
  uint8_t idx = 0;      // command index (v2 layout 2)
};

struct Counters {
  uint32_t sent = 0;        // sends, retries included
  uint32_t replies = 0;     // matching answers
  uint32_t retries = 0;
  uint32_t no_reply = 0;    // commands given up with no answer
  uint32_t bad_reply = 0;   // answers with a wrong value
  uint32_t checks = 0;      // read-backs answered
  uint32_t mismatches = 0;  // read-backs that differed
  uint32_t resyncs = 0;     // times all five were queued again
  uint32_t writes = 0;      // write sends, retries included (0.5.91: the proof of no repeat writes)
  uint32_t gaps = 0;
  uint32_t queue_full = 0;
  uint32_t refused = 0;     // v2: answers with a status other than OK or OK_CLAMPED
  uint32_t stm_resets = 0;  // v2: status bit 21 seen
  uint32_t events_lost = 0; // events dropped: the event queue was full
  uint32_t failures() const { return no_reply + bad_reply + mismatches + queue_full + refused; }
};

// v1 channel value (bits 0..6 channel, bit 7 sensor off) -> v2 loop word of length 1 (7 = off).
uint32_t loopWord(uint32_t v1);

class Link {
 public:
  // Queue a command (the old espPacketFifo_addCmd*). A write of 1, 2, 6, 7 or 8 also sets the value
  // that the read-back expects. false: the queue is full, the command is dropped and counted.
  // idx: the index byte of the v2.0 layout 2 command frame (bin, sensor or item; 0 for v1).
  bool push(uint32_t cid, uint32_t raw, uint8_t idx = 0);
  bool pushInt(uint32_t cid, int32_t v, uint8_t idx = 0);
  bool pushFloat(uint32_t cid, float v, uint8_t idx = 0);
  // One valid STM32 packet (CRC good), at time now_ms.
  void onPacket(uint32_t now_ms, uint32_t status_bits, uint32_t rid, uint32_t response);
  // v2: one valid frame and its answer fields (present = resp_cid != 0).
  void onAnswerV2(uint32_t now_ms, uint32_t status_bits, bool present, uint8_t seq, uint8_t cid8, uint8_t index, uint8_t status, uint32_t value);
  void setV2(bool v2) { v2_ = v2; }
  // The period factor (update period / 10 ms, lib/frame_pid/frame_period.h): the answer timeouts, counted in packets,
  // scale with it. The save timeout (a flash write time) does not.
  void setPeriodRatio(double r) { period_r_ = r > 0 ? r : 1.0; }
  bool v2() const { return v2_; }
  void setSeq(uint8_t s) { seq_ = s; }      // a random start: an ESP32 restart then cannot repeat the last seq
  uint8_t seq() const { return cur_seq_; }  // seq of the command that next() gave (v2)
  uint8_t idx() const { return cur_.idx; }  // index of the command that next() gave (v2)
  // v2 layout 2: the firmware handles the command that next() gave without the STM32 (wire channel
  // select, a save with nothing to save): it counts as answered.
  void doneLocal();
  // v2 layout 2: the firmware sends another command in place of the one next() gave (centre
  // calibration 4, 5 -> angle calibration 28); the answer then matches the new one.
  void rewrite(uint32_t cid, uint32_t raw, uint8_t idx);
  void resyncAll() { resync(); }            // send all tracked settings again
  // The value the STM32 reports for a command in v2 (resp_value), from the value as pushed.
  static uint32_t v2Value(uint32_t id, uint32_t raw);
  // After onPacket: the command to send now, if any. At most one a packet, as before.
  bool next(uint32_t now_ms, uint32_t* cid, uint32_t* raw);
  bool popEvent(Event* e);
  const Counters& counters() const { return c_; }
  bool busy() const { return waiting_ || head_ != tail_; }

  // The value the STM32 holds after a write: the STM32 limits, and command 8 in s.
  static uint32_t heldValue(uint32_t id, uint32_t raw);

 private:
  struct Cmd { uint32_t cid; uint32_t raw; uint8_t idx; };
  static const int SLOTS = 8;
  static int slotOf(uint32_t id);  // 1, 2, 6, 7, 8, 11, 12, 13 -> 0..7, else -1
  void gapCheck(uint32_t now_ms);
  void checkDone(uint32_t got);
  bool replyMatches(const Cmd& c, uint32_t rid) const;
  bool replyValueOk(const Cmd& c, uint32_t response) const;
  void finish();
  void resync();
  void event(int32_t kind, uint32_t cid, uint32_t got, uint32_t want, uint8_t tries);

  Cmd q_[QUEUE_LEN];
  int head_ = 0, tail_ = 0;
  bool waiting_ = false;
  Cmd cur_{0, 0};
  uint8_t tries_ = 0;
  uint32_t sent_ms_ = 0;
  bool is_check_ = false;
  bool resend_now_ = false;  // a wrong answer: send again at the next packet

  bool have_[SLOTS] = {};
  uint32_t sent_raw_[SLOTS] = {};   // the value of the last write, as sent
  uint32_t held_[SLOTS] = {};       // heldValue() of it
  bool v2_ = false;
  double period_r_ = 1.0;
  uint8_t seq_ = 0, cur_seq_ = 0;
  bool reset_seen_ = false;
  uint32_t check_want_ = 0;  // the value a read-back expects, taken when it is sent
  int next_check_ = 0;
  uint32_t check_period_ms_ = CHECK_PERIOD_MS;
  uint32_t last_check_ms_ = 0;
  bool check_started_ = false;

  bool have_packet_ = false;
  uint32_t last_packet_ms_ = 0;

  Event ev_[EVENT_LEN];
  int ev_head_ = 0, ev_tail_ = 0;
  Counters c_;
};

}  // namespace stmset
