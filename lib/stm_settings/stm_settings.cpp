// stm_settings: see stm_settings.h.
#include "stm_settings.h"

#include <string.h>

namespace stmset {

static uint32_t idOf(uint32_t cid) { return cid & 0x7FFFFFu; }  // as the STM32 selects the command
static bool isWrite(uint32_t cid) { return (cid & WRITE) != 0; }
uint32_t loopWord(uint32_t v1) {
  const uint32_t ch = (v1 & 0x80u) ? 7u : (v1 & 0x7Fu);
  return ch << 3;  // length 1: bits 0..2 = 0; entry 0 in bits 3..5
}
static float asFloat(uint32_t raw) { float f; memcpy(&f, &raw, 4); return f; }
static uint32_t asRaw(float f) { uint32_t r; memcpy(&r, &f, 4); return r; }
// 3 writes the STM32 flash; 4 and 5 add the present angle to the stored offset: never sent twice.
// 3 (save) and 36 (debug fault: the STM32 resets before it answers; a resend faults it again, bench
// test 2026-10-11: 3 faults per trigger) go once only.
static bool retryable(uint32_t cid, bool v2) { const uint32_t id = idOf(cid); return id != 3 && id != 36 && (v2 || (id != 4 && id != 5)); }
static const uint32_t RESYNC_ORDER[8] = {6, 7, 8, 1, 2, 13, 11, 12};  // the start-up order without 3, then the v2 settings
static const uint32_t SLOT_ID[8] = {1, 2, 6, 7, 8, 11, 12, 13};

int Link::slotOf(uint32_t id) {
  switch (id) {
    case 1: return 0;
    case 2: return 1;
    case 6: return 2;
    case 7: return 3;
    case 8: return 4;
    case 11: return 5;
    case 12: return 6;
    case 13: return 7;
    default: return -1;
  }
}

uint32_t Link::heldValue(uint32_t id, uint32_t raw) {
  switch (id) {
    case 1:
    case 2: return raw & 0xFFu;
    case 6: { const float f = asFloat(raw); return asRaw(f <= VSTOP_MAX_V ? f : VSTOP_MAX_V); }
    case 7: { const float f = asFloat(raw); return asRaw(f <= IMAX_MAX_A ? f : IMAX_MAX_A); }
    case 8: { const int32_t v = (int32_t)raw; return (uint32_t)(v <= TIME_MAX_S ? v : TIME_MAX_S); }
    default: return raw;
  }
}

uint32_t Link::v2Value(uint32_t id, uint32_t raw) {
  switch (id) {
    case 6:
    case 7:
    case 8: return heldValue(id, raw);
    default: return raw;
  }
}

bool Link::push(uint32_t cid, uint32_t raw, uint8_t idx) {
  const int h = (head_ + 1) % QUEUE_LEN;
  if (h == tail_) {
    c_.queue_full++;
    event(K_QUEUE_FULL, cid, raw, 0, 0);
    return false;
  }
  q_[head_] = Cmd{cid, raw, idx};
  head_ = h;
  const int s = slotOf(idOf(cid));
  if (isWrite(cid) && s >= 0 && idx == 0) {
    have_[s] = true;
    sent_raw_[s] = raw;
    held_[s] = heldValue(idOf(cid), raw);
  }
  return true;
}

bool Link::pushInt(uint32_t cid, int32_t v, uint8_t idx) { return push(cid, (uint32_t)v, idx); }
bool Link::pushFloat(uint32_t cid, float v, uint8_t idx) { return push(cid, asRaw(v), idx); }

void Link::doneLocal() {
  if (!waiting_) return;
  c_.replies++;
  if (is_check_) { c_.checks++; check_period_ms_ = CHECK_PERIOD_MS; }
  finish();
}

void Link::rewrite(uint32_t cid, uint32_t raw, uint8_t idx) { cur_ = Cmd{cid, raw, idx}; }

bool Link::replyMatches(const Cmd& c, uint32_t rid) const {
  const uint32_t id = idOf(c.cid);
  const bool sets_rid = isWrite(c.cid) || id == 1 || id == 2 || id == 3;
  return sets_rid ? rid == c.cid : true;  // a read of 6, 7, 8: the first CMD_ACK after the send
}

bool Link::replyValueOk(const Cmd& c, uint32_t response) const {
  if (!isWrite(c.cid)) return true;
  const uint32_t id = idOf(c.cid);
  switch (id) {
    case 1:
    case 2: return response == (c.raw & 0xFFu);
    case 3: return response == 1u;
    case 6:
    case 7: return asFloat(response) == asFloat(heldValue(id, c.raw));
    case 8: return response == heldValue(id, c.raw) * 100u;
    case 9: return response == c.raw;
    default: return true;  // 4, 5: the new offset
  }
}

void Link::finish() {
  waiting_ = false;
  resend_now_ = false;
  is_check_ = false;
}

void Link::event(int32_t kind, uint32_t cid, uint32_t got, uint32_t want, uint8_t tries) {
  const int h = (ev_head_ + 1) % EVENT_LEN;
  if (h == ev_tail_) { c_.events_lost++; return; }
  Event& e = ev_[ev_head_];
  e.kind = kind; e.cid = cid; e.got = got; e.want = want; e.tries = tries; e.idx = cur_.idx;
  ev_head_ = h;
}

bool Link::popEvent(Event* e) {
  if (ev_tail_ == ev_head_) return false;
  *e = ev_[ev_tail_];
  ev_tail_ = (ev_tail_ + 1) % EVENT_LEN;
  return true;
}

void Link::resync() {
  bool any = false;
  for (uint32_t id : RESYNC_ORDER) {
    const int s = slotOf(id);
    if (!have_[s]) continue;
    bool queued = false;  // a write of this command still waits: no second copy
    for (int i = tail_; i != head_; i = (i + 1) % QUEUE_LEN)
      if (q_[i].cid == (WRITE | id)) { queued = true; break; }
    if (queued) continue;
    if (push(WRITE | id, sent_raw_[s])) any = true;
  }
  if (any) c_.resyncs++;
}

void Link::gapCheck(uint32_t now_ms) {
  if (have_packet_ && !v2_ && now_ms - last_packet_ms_ >= GAP_MS) {  // v2: no resend after a gap (0.5.91)
    c_.gaps++;
    event(K_GAP, now_ms - last_packet_ms_, 0, 0, 0);
    resync();
  }
  have_packet_ = true;
  last_packet_ms_ = now_ms;
}

void Link::checkDone(uint32_t got) {  // a read-back answered: compare with the value held
  const int s = slotOf(idOf(cur_.cid));
  c_.replies++;
  c_.checks++;
  check_period_ms_ = CHECK_PERIOD_MS;
  const uint32_t want = check_want_;  // taken when the read was sent: a write pushed meanwhile is not a difference
  const bool is_float = SLOT_ID[s] == 6 || SLOT_ID[s] == 7;
  const bool same = is_float ? asFloat(got) == asFloat(want) : got == want;
  const Cmd done = cur_;
  finish();
  if (!same) {
    c_.mismatches++;
    event(K_MISMATCH, done.cid, got, want, 1);
    resync();
  }
}

void Link::onAnswerV2(uint32_t now_ms, uint32_t status_bits, bool present, uint8_t seq, uint8_t cid8, uint8_t index, uint8_t status, uint32_t value) {
  gapCheck(now_ms);
  const bool stm_reset = (status_bits & (1u << 21)) != 0;  // WFRX_STATUS_STM_RESET
  if (stm_reset && !reset_seen_) {
    c_.stm_resets++;
    event(K_STM_RESET, 0, 0, 0, 0);
    resync();
  }
  reset_seen_ = stm_reset;
  const uint8_t want_cid = (uint8_t)((idOf(cur_.cid) & 0x7Fu) | (isWrite(cur_.cid) ? 0x80u : 0u));
  if (!waiting_ || !present || seq != cur_seq_ || cid8 != want_cid || index != cur_.idx) return;
  if (status != 1 && status != 2) {  // RSP_OK, RSP_OK_CLAMPED
    c_.refused++;
    event(K_REFUSED, cur_.cid, status, value, tries_);
    const uint32_t id = idOf(cur_.cid);
    const int s = slotOf(id);
    if (isWrite(cur_.cid) && s >= 0) {  // a refused setting: the read-back expects what the STM32 holds
      sent_raw_[s] = value;
      held_[s] = heldValue(id, sent_raw_[s]);
    }
    finish();
    return;
  }
  if (is_check_) { checkDone(value); return; }
  const uint32_t id = idOf(cur_.cid);
  const bool check_value = isWrite(cur_.cid) && slotOf(id) >= 0;
  const uint32_t want = v2Value(id, cur_.raw);
  const bool ok = !check_value || ((id == 6 || id == 7) ? asFloat(value) == asFloat(want) : value == want);
  if (ok) { c_.replies++; finish(); return; }
  c_.bad_reply++;
  event(K_BAD_REPLY, cur_.cid, value, want, tries_);
  finish();  // the STM32 answered: a repeat with the same seq gets the same answer
}

void Link::onPacket(uint32_t now_ms, uint32_t status_bits, uint32_t rid, uint32_t response) {
  gapCheck(now_ms);

  if (!waiting_ || !(status_bits & STATUS_CMD_ACK) || !replyMatches(cur_, rid)) return;
  if (is_check_) { checkDone(response); return; }
  if (replyValueOk(cur_, response)) {
    c_.replies++;
    finish();
    return;
  }
  c_.bad_reply++;
  event(K_BAD_REPLY, cur_.cid, response, isWrite(cur_.cid) ? heldValue(idOf(cur_.cid), cur_.raw) : 0, tries_);
  if (retryable(cur_.cid, v2_) && tries_ < MAX_TRIES) resend_now_ = true;
  else finish();
}

bool Link::next(uint32_t now_ms, uint32_t* cid, uint32_t* raw) {
  if (waiting_) {
    const uint32_t timeout = idOf(cur_.cid) == 3 ? SAVE_TIMEOUT_MS : v2_ ? REPLY_TIMEOUT_V2_MS : REPLY_TIMEOUT_MS;
    if (!resend_now_ && now_ms - sent_ms_ < timeout) return false;
    if (resend_now_ || (retryable(cur_.cid, v2_) && tries_ < MAX_TRIES)) {
      resend_now_ = false;
      tries_++;
      c_.retries++;
      c_.sent++;
      sent_ms_ = now_ms;
      *cid = cur_.cid;
      *raw = cur_.raw;
      return true;
    }
    c_.no_reply++;
    event(K_NO_REPLY, cur_.cid, 0, 0, tries_);
    if (is_check_) check_period_ms_ = check_period_ms_ * 2 < CHECK_PERIOD_MAX_MS ? check_period_ms_ * 2 : CHECK_PERIOD_MAX_MS;
    finish();
  }

  if (head_ != tail_) {
    cur_ = q_[tail_];
    tail_ = (tail_ + 1) % QUEUE_LEN;
    is_check_ = false;
  } else {
    // Idle: read back one of the five settings each check period (v1 only; 0.5.91: v2 sends nothing unasked).
    if (v2_) return false;
    if (!check_started_) { check_started_ = true; last_check_ms_ = now_ms; }
    if (now_ms - last_check_ms_ < check_period_ms_) return false;
    int s = -1;
    for (int k = 0; k < SLOTS; k++) {
      const int cand = slotOf(RESYNC_ORDER[(next_check_ + k) % SLOTS]);
      if (have_[cand]) { s = cand; next_check_ = (next_check_ + k + 1) % SLOTS; break; }
    }
    if (s < 0) return false;
    last_check_ms_ = now_ms;
    cur_ = Cmd{SLOT_ID[s], 0, 0};  // no write bit: a read
    check_want_ = v2_ ? v2Value(SLOT_ID[s], sent_raw_[s]) : held_[s];
    is_check_ = true;
  }
  waiting_ = true;
  resend_now_ = false;
  tries_ = 1;
  cur_seq_ = ++seq_;  // a new command, a new seq; a retry keeps it
  c_.sent++;
  if (cur_.cid & WRITE) c_.writes++;
  sent_ms_ = now_ms;
  *cid = cur_.cid;
  *raw = cur_.raw;
  return true;
}

}  // namespace stmset
