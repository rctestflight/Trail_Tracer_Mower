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

// ---- SLOW flash ring ---------------------------------------------------------------------------
// Each boot starts writing in a fresh sector (the attach scan never appends to a sector it found),
// so one sector holds records of one boot only. Sectors are used in circular order.
namespace {

#define FL_PAGES (TELEMLOG_FLASH_SECTOR / TELEMLOG_FLASH_PAGE)
#define FL_PAGE_HDR 8

enum FState : uint8_t { FS_UNKNOWN, FS_ERASED, FS_USED, FS_DIRTY };

struct FSec {
  uint32_t page_seq0;  // page_seq of the sector's first page
  uint32_t boot_id, first_seq, last_seq;
  uint8_t state, pages;
};

struct Stage {
  uint8_t buf[TELEMLOG_FLASH_PAGE];
  uint16_t len;
  uint32_t first_seq, last_seq;
  bool full;
};

const telemlog_flash_t *g_fl = nullptr;
FSec *g_fs = nullptr;
uint32_t g_fs_n = 0;
uint32_t g_scan_next = 0;
bool g_scan_done = false;
uint32_t g_page_seq_next = 1;
uint32_t g_wsec = 0, g_wpage = FL_PAGES;  // FL_PAGES: no open sector yet
Stage g_stage[2];
int g_stage_cur = 0;
uint32_t g_next_flashinfo_ms = 0;
telemlog_flash_counters_t g_fc;
uint8_t g_page[TELEMLOG_FLASH_PAGE];

void stage_reset(Stage &s) {
  memset(s.buf, 0xFF, sizeof(s.buf));
  s.len = FL_PAGE_HDR;
  s.full = false;
}

// Parses one page. Calls fn(frame, n) for each good entry; stops at the first bad one.
template <typename F>
void page_entries(const uint8_t *p, F fn) {
  uint32_t off = FL_PAGE_HDR;
  while (off + 3 <= TELEMLOG_FLASH_PAGE) {
    uint32_t n = p[off];
    if (n == 0xFF || n < TELEMLOG_LOG_HDR || off + 1 + n + 2 > TELEMLOG_FLASH_PAGE) return;
    uint16_t crc = p[off + 1 + n] | p[off + 2 + n] << 8;
    if (crc16(p + off + 1, n) != crc) return;
    fn(p + off + 1, n);
    off += 1 + n + 2;
  }
}

bool page_hdr_ok(const uint8_t *p, uint32_t *seq) {
  uint32_t a = get32(p), b = get32(p + 4);
  if (a != ~b || a == 0xFFFFFFFFu) return false;
  *seq = a;
  return true;
}

// Scans one sector: state, boot, seq range, pages used. One call per sector, bounded cost.
void scan_sector(uint32_t sec) {
  FSec &f = g_fs[sec];
  f = {0, 0, 0, 0, FS_ERASED, 0};
  bool all_ff = true;
  for (uint32_t pg = 0; pg < FL_PAGES; pg++) {
    if (!g_fl->read(sec * TELEMLOG_FLASH_SECTOR + pg * TELEMLOG_FLASH_PAGE, g_page, sizeof(g_page))) {
      f.state = FS_DIRTY;
      return;
    }
    uint32_t ps;
    if (page_hdr_ok(g_page, &ps) && (pg == 0 || f.state == FS_USED)) {
      if (pg == 0) f.page_seq0 = ps;
      if (ps + 1 > g_page_seq_next) g_page_seq_next = ps + 1;
      f.state = FS_USED;
      f.pages = (uint8_t)(pg + 1);
      page_entries(g_page, [&](const uint8_t *fr, uint32_t) {
        uint32_t boot = get32(fr + 2), seq = get32(fr + 6);
        if (f.boot_id == 0) { f.boot_id = boot; f.first_seq = seq; }
        if (boot == f.boot_id) f.last_seq = seq;
      });
      continue;
    }
    for (uint32_t i = 0; i < sizeof(g_page) && all_ff; i++) all_ff = g_page[i] == 0xFF;
  }
  if (f.state == FS_USED && f.boot_id == 0) f.state = FS_DIRTY;  // pages without a good entry
  else if (f.state != FS_USED && !all_ff) f.state = FS_DIRTY;
}

void scan_finish() {
  g_scan_done = true;
  // Write next after the newest used sector, in a fresh sector.
  uint32_t newest = 0;
  bool any = false;
  for (uint32_t s = 0; s < g_fs_n; s++) {
    if (g_fs[s].state == FS_USED && (!any || g_fs[s].page_seq0 > g_fs[newest].page_seq0)) {
      newest = s;
      any = true;
    }
  }
  g_wsec = any ? (newest + 1) % g_fs_n : 0;
  g_wpage = 0;
}

void flash_stage(const uint8_t *frame, uint32_t n, uint32_t seq) {
  if (!g_fl) return;
  Stage *s = &g_stage[g_stage_cur];
  if (s->full || s->len + 1 + n + 2 > TELEMLOG_FLASH_PAGE) {
    if (s->len > FL_PAGE_HDR) s->full = true;
    Stage *o = &g_stage[1 - g_stage_cur];
    if (o->full) { g_fc.entries_skipped++; return; }  // both pages wait for a program
    g_stage_cur = 1 - g_stage_cur;
    s = o;
    stage_reset(*s);
  }
  if (s->len == FL_PAGE_HDR) s->first_seq = seq;
  s->last_seq = seq;
  s->buf[s->len] = (uint8_t)n;
  memcpy(s->buf + s->len + 1, frame, n);
  uint16_t crc = crc16(frame, n);
  s->buf[s->len + 1 + n] = (uint8_t)crc;
  s->buf[s->len + 2 + n] = (uint8_t)(crc >> 8);
  s->len = (uint16_t)(s->len + 1 + n + 2);
  if (s->len + 1 + TELEMLOG_LOG_HDR + 1 + 2 > TELEMLOG_FLASH_PAGE) s->full = true;
}

// Programs one full staged page if a page is ready. Returns true if it used the flash.
bool flash_program() {
  Stage *s = g_stage[1 - g_stage_cur].full ? &g_stage[1 - g_stage_cur]
           : g_stage[g_stage_cur].full     ? &g_stage[g_stage_cur] : nullptr;
  if (!s) return false;
  if (g_wpage >= FL_PAGES) {  // open the next sector
    uint32_t next = (g_wpage == FL_PAGES && g_fs[g_wsec].state == FS_USED) ? (g_wsec + 1) % g_fs_n : g_wsec;
    if (g_fs[next].state != FS_ERASED) return false;  // wait for an erase; staging fills, then skips
    g_wsec = next;
    g_wpage = 0;
  }
  if (g_wpage == 0 && g_fs[g_wsec].state != FS_ERASED) return false;
  uint32_t ps = g_page_seq_next++;
  put32(s->buf, ps);
  put32(s->buf + 4, ~ps);
  if (!g_fl->write(g_wsec * TELEMLOG_FLASH_SECTOR + g_wpage * TELEMLOG_FLASH_PAGE, s->buf, TELEMLOG_FLASH_PAGE)) {
    g_fc.write_fail++;
    g_fs[g_wsec].state = FS_DIRTY;  // never write this sector again before an erase
    g_wpage = FL_PAGES;
    return true;
  }
  FSec &f = g_fs[g_wsec];
  if (g_wpage == 0) f = {ps, g_ring[1].seg[1].boot_id, s->first_seq, s->last_seq, FS_USED, 0};
  f.last_seq = s->last_seq;
  f.pages = (uint8_t)(g_wpage + 1);
  g_wpage++;
  g_fc.pages_written++;
  stage_reset(*s);
  return true;
}

// Erases the first sector, in write order, among the next TELEMLOG_FLASH_ERASED_AHEAD that is not erased.
bool flash_erase_ahead() {
  uint32_t start = (g_wpage == 0 && g_fs[g_wsec].state != FS_ERASED) ? 0 : 1;
  if (g_wpage >= FL_PAGES && g_fs[g_wsec].state != FS_USED) start = 0;
  for (uint32_t k = start; k <= TELEMLOG_FLASH_ERASED_AHEAD && k < g_fs_n; k++) {
    uint32_t sec = (g_wsec + k) % g_fs_n;
    if (sec == g_wsec && g_wpage > 0 && g_wpage < FL_PAGES) continue;  // the open sector
    if (g_fs[sec].state == FS_ERASED) continue;
    if (g_fl->erase_sector(sec)) {
      g_fs[sec] = {0, 0, 0, 0, FS_ERASED, 0};
      g_fc.sectors_erased++;
    } else {
      g_fc.erase_fail++;
      g_fs[sec].state = FS_DIRTY;
    }
    return true;
  }
  return false;
}

// Finds a SLOW record in flash and copies its LOG frame. Returns the frame length or 0.
size_t flash_fetch(uint32_t boot_id, uint32_t seq, uint8_t *frame, size_t cap) {
  if (!g_fl || !g_scan_done) return 0;
  for (uint32_t s = 0; s < g_fs_n; s++) {
    const FSec &f = g_fs[s];
    if (f.state != FS_USED || f.boot_id != boot_id || seq < f.first_seq || seq > f.last_seq) continue;
    for (uint32_t pg = 0; pg < f.pages; pg++) {
      if (!g_fl->read(s * TELEMLOG_FLASH_SECTOR + pg * TELEMLOG_FLASH_PAGE, g_page, sizeof(g_page))) return 0;
      size_t got = 0;
      page_entries(g_page, [&](const uint8_t *fr, uint32_t n) {
        if (!got && get32(fr + 2) == boot_id && get32(fr + 6) == seq && n <= cap) {
          memcpy(frame, fr, n);
          got = n;
        }
      });
      if (got) return got;
    }
  }
  return 0;
}

}  // namespace

bool telemlog_flash_attach(const telemlog_flash_t *flash) {
  if (!flash || flash->sectors < TELEMLOG_FLASH_ERASED_AHEAD + 2) return false;
  FSec *fs = (FSec *)realloc(g_fs, flash->sectors * sizeof(FSec));
  if (!fs) return false;
  g_fs = fs;
  g_fs_n = flash->sectors;
  for (uint32_t s = 0; s < g_fs_n; s++) g_fs[s] = {0, 0, 0, 0, FS_UNKNOWN, 0};
  g_fl = flash;
  g_scan_next = 0;
  g_scan_done = false;
  g_page_seq_next = 1;
  g_wsec = 0;
  g_wpage = FL_PAGES;
  stage_reset(g_stage[0]);
  stage_reset(g_stage[1]);
  g_stage_cur = 0;
  memset(&g_fc, 0, sizeof(g_fc));
  return true;
}

void telemlog_flash_service(uint32_t now_ms, bool allow_erase) {
  (void)now_ms;
  if (!g_fl) return;
  if (!g_scan_done) {  // one sector per call: no boot delay
    scan_sector(g_scan_next++);
    if (g_scan_next >= g_fs_n) scan_finish();
    return;
  }
  if (flash_program()) return;
  if (allow_erase) flash_erase_ahead();
}

void telemlog_flash_counters(telemlog_flash_counters_t *out) {
  *out = g_fc;
  out->used_sectors = out->erased_sectors = 0;
  for (uint32_t s = 0; s < g_fs_n; s++) {
    if (g_fs[s].state == FS_USED) out->used_sectors++;
    if (g_fs[s].state == FS_ERASED) out->erased_sectors++;
  }
}

size_t telemlog_flashinfo_frame(uint8_t *buf, size_t cap) {
  if (cap < 2 || !g_fl || !g_scan_done) return 0;
  // Boots in flash, newest first: for each boot the lowest first_seq and highest last_seq.
  uint32_t n = 0, done_below = 0xFFFFFFFFu;
  buf[0] = TELEMLOG_T_FLASHINFO;
  while (n < TELEMLOG_FLASHINFO_MAX && 2 + 12 * (n + 1) <= cap) {
    uint32_t boot = 0, lo = 0xFFFFFFFFu, hi = 0;
    for (uint32_t s = 0; s < g_fs_n; s++)
      if (g_fs[s].state == FS_USED && g_fs[s].boot_id < done_below && g_fs[s].boot_id > boot) boot = g_fs[s].boot_id;
    if (boot == 0) break;
    for (uint32_t s = 0; s < g_fs_n; s++) {
      if (g_fs[s].state != FS_USED || g_fs[s].boot_id != boot) continue;
      if (g_fs[s].first_seq < lo) lo = g_fs[s].first_seq;
      if (g_fs[s].last_seq > hi) hi = g_fs[s].last_seq;
    }
    put32(buf + 2 + 12 * n, boot);
    put32(buf + 6 + 12 * n, lo);
    put32(buf + 10 + 12 * n, hi);
    n++;
    done_below = boot;
  }
  buf[1] = (uint8_t)n;
  return 2 + 12 * n;
}

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
  if (g_fl) {  // a new boot never shares a sector with an older one
    stage_reset(g_stage[0]);
    stage_reset(g_stage[1]);
    if (g_wpage > 0) g_wpage = FL_PAGES;
  }
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
  if (stream == TELEMLOG_SLOW && g_fl) {
    uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_SLOW_BODY_MAX];
    size_t n = read_frame(stream, r.gidx_next - 1, r.seg[1].boot_id, seq, frame, sizeof(frame));
    if (n) flash_stage(frame, (uint32_t)n, seq);
  }
  return seq;
}

size_t telemlog_get(int stream, uint32_t boot_id, uint32_t seq, uint8_t *buf, size_t cap, uint32_t *ms) {
  uint32_t gidx;
  uint8_t frame[TELEMLOG_LOG_HDR + TELEMLOG_FAST_BODY_MAX];
  if (stream < 0 || stream > 1) return 0;
  size_t n = 0;
  if (locate(stream, boot_id, seq, &gidx)) n = read_frame(stream, gidx, boot_id, seq, frame, sizeof(frame));
  if (!n && stream == TELEMLOG_SLOW) n = flash_fetch(boot_id, seq, frame, sizeof(frame));
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
    g_next_backfill_ms = g_next_info_ms = g_second_start_ms = g_next_flashinfo_ms = now_ms;
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
  if (g_fl && g_scan_done && (int32_t)(now_ms - g_next_flashinfo_ms) >= 0) {
    g_next_flashinfo_ms = now_ms + TELEMLOG_FLASHINFO_PERIOD_MS;
    size_t n = telemlog_flashinfo_frame(frame, sizeof(frame));
    if (n > 2) send_counted(send, BROADCAST, frame, n, &ok);
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
    if (!n && g_job.stream == TELEMLOG_SLOW) n = flash_fetch(g_job.boot_id, seq, frame, sizeof(frame));
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
