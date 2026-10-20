// telemlog: see telemlog.h.
#include "telemlog.h"

#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include <esp_attr.h>
#define TELEMLOG_NOINIT __NOINIT_ATTR
#else
#define TELEMLOG_NOINIT  // a PC test keeps the statics between telemlog_init() calls
#endif

#define RING_MAGIC 0x544C4F47u  // "TLOG"
#define RING_VERSION 1u

namespace {

struct Seg {
  uint32_t boot_id;
  uint32_t gidx0;  // global index of this boot's seq 0
  uint32_t valid;
};

// Ring header. gidx counts records across boots; slot = gidx % slots.
struct Ring {
  uint32_t magic, version, slots, slot_bytes;
  uint32_t gidx_next;
  Seg seg[2];  // [0] previous boot, [1] current boot
  uint32_t crc;
};

struct Stream {
  Ring *ring;
  uint8_t *slots;
  uint32_t nslots, slot_bytes, body_max;
  uint32_t live_next;  // next gidx to send live
};

TELEMLOG_NOINIT Ring g_ring[2];
TELEMLOG_NOINIT uint8_t g_fast[TELEMLOG_FAST_SLOTS][TELEMLOG_FAST_SLOT_BYTES];
TELEMLOG_NOINIT uint8_t g_slow[TELEMLOG_SLOW_SLOTS][TELEMLOG_SLOW_SLOT_BYTES];

Stream g_st[2];

struct Req {
  uint8_t mac[6];
  uint8_t stream;
  uint32_t boot_id, from_seq;
  uint64_t mask;
};

// Single producer (receive callback) and single consumer (loop). Full: the new REQ is dropped.
Req g_q[TELEMLOG_REQ_QUEUE];
volatile uint32_t g_q_head = 0, g_q_tail = 0;
volatile uint32_t g_req_received = 0, g_req_dropped = 0;

bool g_job_active = false;
Req g_job;
int g_job_bit = 0;

uint32_t g_next_backfill_ms = 0, g_next_info_ms = 0;
uint32_t g_second_start_ms = 0, g_backfill_this_s = 0;
bool g_started = false;
telemlog_counters_t g_c;

const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint16_t crc16(const uint8_t *p, size_t n, uint16_t crc = 0xFFFF) {  // CRC-16/CCITT-FALSE
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

uint32_t ring_crc(const Ring &r) {
  return 0x5A000000u ^ crc16((const uint8_t *)&r, offsetof(Ring, crc));
}

void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

uint32_t gidx_first(const Stream &s) {
  return s.ring->gidx_next > s.nslots ? s.ring->gidx_next - s.nslots : 0;
}

// Range of global indexes held for segment k: [lo, hi). Empty if lo >= hi.
void seg_span(const Stream &s, int k, uint32_t *lo, uint32_t *hi) {
  const Ring &r = *s.ring;
  *lo = *hi = 0;
  if (!r.seg[k].valid) return;
  uint32_t end = (k == 0) ? r.seg[1].gidx0 : r.gidx_next;
  uint32_t start = r.seg[k].gidx0;
  uint32_t first = gidx_first(s);
  if (start < first) start = first;
  if (start < end) { *lo = start; *hi = end; }
}

uint8_t *slot_of(const Stream &s, uint32_t gidx) { return s.slots + (size_t)(gidx % s.nslots) * s.slot_bytes; }

// Reads the record at gidx, checks it, and builds its LOG frame (header + body) in frame.
size_t read_frame(int stream, uint32_t gidx, uint32_t boot_id, uint32_t seq, uint8_t *frame, size_t cap) {
  const Stream &s = g_st[stream];
  const uint8_t *p = slot_of(s, gidx);
  uint32_t len = p[12] | p[13] << 8;
  if (get32(p) != boot_id || get32(p + 4) != seq || len == 0 || len > s.body_max) return 0;
  uint16_t crc = p[14] | p[15] << 8;
  if (crc16(p, 14, crc16(p + TELEMLOG_SLOT_HDR, len)) != crc) return 0;
  if (TELEMLOG_LOG_HDR + len > cap) return 0;
  frame[0] = TELEMLOG_T_LOG;
  frame[1] = (uint8_t)stream;
  put32(frame + 2, boot_id);
  put32(frame + 6, seq);
  memcpy(frame + 10, p + 8, 4);  // ms
  memcpy(frame + TELEMLOG_LOG_HDR, p + TELEMLOG_SLOT_HDR, len);
  return TELEMLOG_LOG_HDR + len;
}

// Maps (boot_id, seq) to a held global index.
bool locate(int stream, uint32_t boot_id, uint32_t seq, uint32_t *gidx) {
  const Stream &s = g_st[stream];
  for (int k = 0; k < 2; k++) {
    if (!s.ring->seg[k].valid || s.ring->seg[k].boot_id != boot_id) continue;
    uint32_t lo, hi;
    seg_span(s, k, &lo, &hi);
    uint32_t g = s.ring->seg[k].gidx0 + seq;
    if (seq < 0x80000000u && g >= lo && g < hi) { *gidx = g; return true; }
  }
  return false;
}

void send_counted(telemlog_send_fn send, const uint8_t *mac, const uint8_t *buf, size_t len, bool *ok) {
  *ok = send(mac, buf, len);
  if (!*ok) g_c.send_fail++;
}

}  // namespace

void telemlog_init(uint32_t boot_id, bool keep_previous) {
  g_st[0] = {&g_ring[0], &g_fast[0][0], TELEMLOG_FAST_SLOTS, TELEMLOG_FAST_SLOT_BYTES, TELEMLOG_FAST_BODY_MAX, 0};
  g_st[1] = {&g_ring[1], &g_slow[0][0], TELEMLOG_SLOW_SLOTS, TELEMLOG_SLOW_SLOT_BYTES, TELEMLOG_SLOW_BODY_MAX, 0};
  memset(&g_c, 0, sizeof(g_c));
  bool kept = true;
  for (int i = 0; i < 2; i++) {
    Stream &s = g_st[i];
    Ring &r = *s.ring;
    bool valid = keep_previous && r.magic == RING_MAGIC && r.version == RING_VERSION &&
                 r.slots == s.nslots && r.slot_bytes == s.slot_bytes && r.crc == ring_crc(r) &&
                 r.seg[1].valid && r.seg[1].boot_id != boot_id && r.seg[1].gidx0 <= r.gidx_next;
    if (valid && r.gidx_next > 0xF0000000u) valid = false;  // start over long before gidx wraps
    if (valid) {
      r.seg[0] = r.seg[1];
      if (r.seg[0].gidx0 == r.gidx_next) r.seg[0].valid = 0;  // the previous boot made no record
    } else {
      kept = false;
      memset(&r, 0, sizeof(r));
      r.magic = RING_MAGIC;
      r.version = RING_VERSION;
      r.slots = s.nslots;
      r.slot_bytes = s.slot_bytes;
    }
    r.seg[1] = {boot_id, r.gidx_next, 1};
    r.crc = ring_crc(r);
    s.live_next = r.gidx_next;
  }
  g_c.prev_kept = kept && (g_ring[0].seg[0].valid || g_ring[1].seg[0].valid);
  g_q_head = g_q_tail = 0;
  g_job_active = false;
  g_started = false;
}

uint32_t telemlog_add(int stream, uint32_t ms, const uint8_t *body, size_t len) {
  if (stream < 0 || stream > 1 || len == 0 || len > g_st[stream].body_max) {
    g_c.add_refused++;
    return TELEMLOG_NO_SEQ;
  }
  Stream &s = g_st[stream];
  Ring &r = *s.ring;
  uint32_t seq = r.gidx_next - r.seg[1].gidx0;
  uint8_t *p = slot_of(s, r.gidx_next);
  put32(p, r.seg[1].boot_id);
  put32(p + 4, seq);
  put32(p + 8, ms);
  p[12] = (uint8_t)len;
  p[13] = (uint8_t)(len >> 8);
  memcpy(p + TELEMLOG_SLOT_HDR, body, len);
  uint16_t crc = crc16(p, 14, crc16(body, len));
  p[14] = (uint8_t)crc;
  p[15] = (uint8_t)(crc >> 8);
  // The record is complete before the header counts it, so a reset in between loses only it.
  r.gidx_next++;
  r.crc = ring_crc(r);
  return seq;
}

size_t telemlog_get(int stream, uint32_t boot_id, uint32_t seq, uint8_t *buf, size_t cap, uint32_t *ms) {
  uint32_t gidx;
  uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_FAST_BODY_MAX];
  if (stream < 0 || stream > 1) return 0;
  size_t n = 0;
  if (locate(stream, boot_id, seq, &gidx)) n = read_frame(stream, gidx, boot_id, seq, frame, sizeof(frame));
  if (n == 0 || n - TELEMLOG_LOG_HDR > cap) return 0;
  if (ms) *ms = get32(frame + 10);
  memcpy(buf, frame + TELEMLOG_LOG_HDR, n - TELEMLOG_LOG_HDR);
  return n - TELEMLOG_LOG_HDR;
}

void telemlog_info(telemlog_info_t *out) {
  out->boot_id = g_ring[1].seg[1].boot_id;
  out->prev_boot_id = 0;
  for (int st = 0; st < 2; st++) {
    const Stream &s = g_st[st];
    for (int k = 0; k < 2; k++) {  // k 0 = current boot = seg[1]
      const Seg &sg = s.ring->seg[1 - k];
      telemlog_range_t &rg = out->r[st][k];
      rg.oldest = rg.newest = TELEMLOG_NO_SEQ;
      uint32_t end = (k == 1) ? s.ring->seg[1].gidx0 : s.ring->gidx_next;
      if (!sg.valid || end <= sg.gidx0) continue;  // no record made in this boot
      if (k == 1) out->prev_boot_id = sg.boot_id;
      // newest = the last record made, held or not; oldest = newest + 1 if none is held any
      // more, so a collector always learns how far the boot went.
      rg.newest = end - 1 - sg.gidx0;
      uint32_t lo, hi;
      seg_span(s, 1 - k, &lo, &hi);
      rg.oldest = (lo < hi) ? lo - sg.gidx0 : rg.newest + 1;
    }
  }
}

size_t telemlog_info_frame(uint8_t *buf, size_t cap) {
  if (cap < TELEMLOG_INFO_LEN) return 0;
  telemlog_info_t in;
  telemlog_info(&in);
  buf[0] = TELEMLOG_T_INFO;
  put32(buf + 1, in.boot_id);
  put32(buf + 5, in.prev_boot_id);
  // Order as in the protocol: s0 cur, s1 cur, s0 prev, s1 prev.
  const telemlog_range_t *order[4] = {&in.r[0][0], &in.r[1][0], &in.r[0][1], &in.r[1][1]};
  for (int i = 0; i < 4; i++) {
    put32(buf + 9 + 8 * i, order[i]->oldest);
    put32(buf + 13 + 8 * i, order[i]->newest);
  }
  return TELEMLOG_INFO_LEN;
}

void telemlog_counters(telemlog_counters_t *out) {
  *out = g_c;
  out->req_received = g_req_received;
  out->req_dropped = g_req_dropped;
}

void telemlog_on_frame(const uint8_t mac[6], const uint8_t *data, int len) {
  if (len < 1 || data[0] != TELEMLOG_T_REQ) return;
  g_req_received++;
  if (len != TELEMLOG_REQ_LEN || data[1] > 1) { g_req_dropped++; return; }
  uint32_t head = g_q_head;
  if (head - g_q_tail >= TELEMLOG_REQ_QUEUE) { g_req_dropped++; return; }
  Req &q = g_q[head % TELEMLOG_REQ_QUEUE];
  memcpy(q.mac, mac, 6);
  q.stream = data[1];
  q.boot_id = get32(data + 2);
  q.from_seq = get32(data + 6);
  q.mask = (uint64_t)get32(data + 10) | (uint64_t)get32(data + 14) << 32;
  g_q_head = head + 1;  // publish after the entry is written
}

void telemlog_poll(uint32_t now_ms, telemlog_send_fn send) {
  uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_FAST_BODY_MAX];
  bool ok;
  if (!g_started) {
    g_started = true;
    g_next_backfill_ms = g_next_info_ms = g_second_start_ms = now_ms;
  }
  if ((int32_t)(now_ms - g_second_start_ms) >= 1000) {
    g_c.backfill_sent_last_s = g_backfill_this_s;
    g_backfill_this_s = 0;
    g_second_start_ms = now_ms;
  }

  // Live: one frame per stream per call; a FAST backlog beyond the limit is skipped.
  for (int st = 0; st < 2; st++) {
    Stream &s = g_st[st];
    uint32_t next = s.ring->gidx_next;
    if (st == TELEMLOG_FAST && next - s.live_next > TELEMLOG_LIVE_FAST_MAX_BACKLOG) {
      g_c.live_skipped += next - s.live_next - 1;
      s.live_next = next - 1;
    }
    if (s.live_next < gidx_first(s)) s.live_next = gidx_first(s);
    if (s.live_next < next) {
      uint32_t g = s.live_next++;
      uint32_t seq = g - s.ring->seg[1].gidx0;
      size_t n = read_frame(st, g, s.ring->seg[1].boot_id, seq, frame, sizeof(frame));
      if (n) {
        send_counted(send, BROADCAST, frame, n, &ok);
        if (ok) g_c.live_sent++;
      }
    }
  }

  if ((int32_t)(now_ms - g_next_info_ms) >= 0) {
    g_next_info_ms = now_ms + TELEMLOG_INFO_PERIOD_MS;
    size_t n = telemlog_info_frame(frame, sizeof(frame));
    send_counted(send, BROADCAST, frame, n, &ok);
  }

  // Backfill: one held record per call, at most one per TELEMLOG_BACKFILL_PERIOD_MS.
  if ((int32_t)(now_ms - g_next_backfill_ms) < 0) return;
  for (;;) {
    if (!g_job_active) {
      if (g_q_tail == g_q_head) return;
      g_job = g_q[g_q_tail % TELEMLOG_REQ_QUEUE];
      g_q_tail = g_q_tail + 1;
      g_job_active = true;
      g_job_bit = 0;
    }
    while (g_job_bit < 64 && !((g_job.mask >> g_job_bit) & 1)) g_job_bit++;
    if (g_job_bit >= 64) { g_job_active = false; continue; }
    uint32_t seq = g_job.from_seq + (uint32_t)g_job_bit++;
    uint32_t gidx;
    size_t n = 0;
    if (locate(g_job.stream, g_job.boot_id, seq, &gidx))
      n = read_frame(g_job.stream, gidx, g_job.boot_id, seq, frame, sizeof(frame));
    if (!n) continue;  // not held: skip, no rate cost
    send_counted(send, g_job.mac, frame, n, &ok);
    if (ok) { g_c.backfill_sent++; g_backfill_this_s++; }
    g_next_backfill_ms = now_ms + TELEMLOG_BACKFILL_PERIOD_MS;
    return;
  }
}

#if !defined(ESP_PLATFORM)
// PC tests only: a power cycle leaves arbitrary bytes in the no-init RAM.
extern "C" void telemlog_test_wipe() {
  uint8_t *areas[3] = {(uint8_t *)g_ring, &g_fast[0][0], &g_slow[0][0]};
  size_t sizes[3] = {sizeof(g_ring), sizeof(g_fast), sizeof(g_slow)};
  uint32_t x = 0x12345678u;
  for (int a = 0; a < 3; a++)
    for (size_t i = 0; i < sizes[a]; i++) { x = x * 1103515245u + 12345u; areas[a][i] = (uint8_t)(x >> 16); }
}
#endif
