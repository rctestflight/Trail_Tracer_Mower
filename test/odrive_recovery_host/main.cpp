#include "odrive_recovery.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

using Recovery = OdriveRecovery;

static Recovery::Action step(Recovery& r, uint32_t now, bool allowed, bool requested,
                             bool fault, bool complete, Recovery::Sample* samples) {
  return r.update(now, allowed, requested, fault, complete, samples);
}

int main() {
  Recovery::Sample s[Recovery::NODE_COUNT] = {};
  s[0] = {10, 20, true, true};
  s[1] = {30, 0, true, false};

  Recovery r;
  assert(step(r, 0, false, true, true, false, s) == Recovery::NONE);
  assert(step(r, 1, true, true, true, false, s) == Recovery::BEGIN);
  assert(r.attempts() == 1 && r.clearing());
  assert(step(r, 2, true, true, true, true, s) == Recovery::NONE);
  assert(step(r, 3, true, true, false, false, s) == Recovery::NONE);
  ++s[0].heartbeat_count;
  ++s[0].error_count;
  assert(step(r, 4, true, true, false, false, s) == Recovery::NONE);
  ++s[1].heartbeat_count;
  assert(step(r, 5, true, true, true, false, s) == Recovery::NONE);
  assert(step(r, 6, true, true, false, false, s) == Recovery::CLEARED);
  assert(r.attempts() == 1);

  // A later fault uses the same switch-cycle budget.
  assert(step(r, 7, true, true, true, false, s) == Recovery::BEGIN);
  assert(r.attempts() == 2);
  assert(step(r, 8, true, true, true, true, s) == Recovery::NONE);
  assert(step(r, 1507, true, true, true, false, s) == Recovery::NONE);
  assert(step(r, 1508, true, true, true, false, s) == Recovery::BEGIN);
  assert(r.attempts() == 3);
  assert(step(r, 1509, true, true, true, true, s) == Recovery::NONE);
  assert(step(r, 3009, true, true, true, false, s) == Recovery::EXHAUSTED);
  assert(step(r, 5000, true, true, true, false, s) == Recovery::NONE);
  r.reset();
  assert(step(r, 5001, true, true, true, false, s) == Recovery::BEGIN);

  // Interlocks abort an in-progress attempt without restoring the budget.
  assert(step(r, 5002, false, true, true, false, s) == Recovery::ABORTED);
  assert(!r.inProgress() && r.attempts() == 1);
  assert(step(r, 5003, false, true, true, false, s) == Recovery::NONE);
  assert(step(r, 5004, true, true, true, false, s) == Recovery::BEGIN);
  assert(r.attempts() == 2);
  assert(step(r, 5005, true, true, true, true, s) == Recovery::NONE);
  assert(step(r, 5006, false, true, false, false, s) == Recovery::ABORTED);

  // Success on the third attempt does not grant an unbounded fourth attempt.
  Recovery last;
  for (uint8_t attempt = 0; attempt < Recovery::MAX_ATTEMPTS; ++attempt) {
    assert(step(last, attempt * 3, true, true, true, false, s) == Recovery::BEGIN);
    assert(step(last, attempt * 3 + 1, true, true, true, true, s) == Recovery::NONE);
    ++s[0].heartbeat_count;
    ++s[0].error_count;
    ++s[1].heartbeat_count;
    assert(step(last, attempt * 3 + 2, true, true, false, false, s) == Recovery::CLEARED);
  }
  assert(step(last, 10, true, true, true, false, s) == Recovery::EXHAUSTED);

  // No response cannot confirm a clear, even when the cached fault is zero.
  Recovery empty;
  Recovery::Sample no_reports[Recovery::NODE_COUNT] = {};
  assert(step(empty, 0, true, true, true, false, no_reports) == Recovery::BEGIN);
  assert(step(empty, 1, true, true, false, true, no_reports) == Recovery::NONE);
  assert(step(empty, 1501, true, true, false, false, no_reports) == Recovery::BEGIN);

  // Timeout arithmetic survives millis() rollover.
  Recovery wrap;
  assert(step(wrap, UINT32_MAX - 100, true, true, true, false, s) == Recovery::BEGIN);
  assert(step(wrap, UINT32_MAX - 99, true, true, true, true, s) == Recovery::NONE);
  assert(step(wrap, 1399, true, true, true, false, s) == Recovery::NONE);
  assert(step(wrap, 1400, true, true, true, false, s) == Recovery::BEGIN);

  puts("ODrive recovery host tests passed");
}
