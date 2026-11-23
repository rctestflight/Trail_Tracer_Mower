#pragma once

#include <stdint.h>

class OdriveRecovery {
 public:
  static const uint8_t NODE_COUNT = 6;
  static const uint8_t MAX_ATTEMPTS = 3;
  static const uint32_t RESPONSE_TIMEOUT_MS = 1500;

  struct Sample {
    uint32_t heartbeat_count;
    uint32_t error_count;
    bool heartbeat_seen;
    bool error_seen;
  };

  enum Action { NONE, BEGIN, CLEARED, ABORTED, EXHAUSTED };

  void reset() {
    state_ = IDLE;
    attempts_ = 0;
  }

  bool clearing() const { return state_ == CLEARING; }
  bool inProgress() const { return state_ == CLEARING || state_ == WAITING; }
  uint8_t attempts() const { return attempts_; }

  Action update(uint32_t now, bool allowed, bool requested, bool fault_present,
                bool clear_complete, const Sample samples[NODE_COUNT]) {
    if (!allowed) {
      if (inProgress()) {
        state_ = IDLE;
        return ABORTED;
      }
      return NONE;
    }

    if (state_ == CLEARING && clear_complete) {
      for (uint8_t n = 0; n < NODE_COUNT; ++n) baseline_[n] = samples[n];
      started_ms_ = now;
      state_ = WAITING;
    }
    if (state_ == WAITING) {
      bool observed = false;
      bool fresh = true;
      for (uint8_t n = 0; n < NODE_COUNT; ++n) {
        if (baseline_[n].heartbeat_seen) {
          observed = true;
          if (baseline_[n].heartbeat_count == samples[n].heartbeat_count) fresh = false;
        }
        if (baseline_[n].error_seen) {
          observed = true;
          if (baseline_[n].error_count == samples[n].error_count) fresh = false;
        }
      }
      if (observed && fresh && !fault_present) {
        state_ = IDLE;
        return CLEARED;
      }
      if ((uint32_t)(now - started_ms_) < RESPONSE_TIMEOUT_MS) return NONE;
      state_ = IDLE;
    }
    if (state_ != IDLE || !requested) return NONE;
    if (attempts_ >= MAX_ATTEMPTS) {
      state_ = HELD;
      return EXHAUSTED;
    }
    ++attempts_;
    state_ = CLEARING;
    return BEGIN;
  }

 private:
  enum State { IDLE, CLEARING, WAITING, HELD };
  State state_ = IDLE;
  uint8_t attempts_ = 0;
  uint32_t started_ms_ = 0;
  Sample baseline_[NODE_COUNT] = {};
};
