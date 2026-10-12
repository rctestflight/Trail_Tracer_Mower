// telemlog: the rover's numbered telemetry log, with live broadcast and backfill on request.
// Protocol: Trail_Tracer_Mower docs/espnow_protocol.md (LOG 0x40, REQ 0x41, INFO 0x42).
//
// - Two streams, each a ring of fixed-size slots: FAST (stream 0) and SLOW (stream 1).
// - Records are numbered per stream per boot: seq 0, 1, 2, ...
// - The rings live in no-init RAM. After a warm reset (panic, watchdog, software restart)
//   the records of the previous boot stay available under the previous boot_id.
// - Threads: call telemlog_on_frame() from the ESP-NOW receive callback; call every other
//   function from loop() only. The two sides share one single-producer single-consumer
//   queue with volatile indexes and no locks.
// The core has no Arduino or ESP-IDF dependency, so it also builds and runs on a PC.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifndef TELEMLOG_FAST_SLOTS
#define TELEMLOG_FAST_SLOTS 300
#endif
#ifndef TELEMLOG_SLOW_SLOTS
#define TELEMLOG_SLOW_SLOTS 600
#endif
// Static RAM = FAST_SLOTS x FAST_SLOT_BYTES + SLOW_SLOTS x SLOW_SLOT_BYTES, all no-init.
// Measured on the ESP32-S2 with Wi-Fi + ESP-NOW and nothing else: 200 x 244 + 400 x 96 links
// (118 KB static in all); 250 x 244 + 500 x 96 overflows dram0_0_seg by 5.8 KB.
#ifndef TELEMLOG_FAST_SLOT_BYTES
#define TELEMLOG_FAST_SLOT_BYTES 244  // body 2 + 7 x 32 = 226 B plus the header
#endif
#ifndef TELEMLOG_SLOW_SLOT_BYTES
#define TELEMLOG_SLOW_SLOT_BYTES 96   // 84 holds the 67 B status and short events
#endif
#define TELEMLOG_SLOT_HDR 16  // u32 boot_id, u32 seq, u32 ms, u16 len, u16 crc
#define TELEMLOG_FAST_BODY_MAX (TELEMLOG_FAST_SLOT_BYTES - TELEMLOG_SLOT_HDR)  // 228
#define TELEMLOG_SLOW_BODY_MAX (TELEMLOG_SLOW_SLOT_BYTES - TELEMLOG_SLOT_HDR)  // 80

#define TELEMLOG_FAST 0
#define TELEMLOG_SLOW 1
#define TELEMLOG_T_LOG 0x40
#define TELEMLOG_T_REQ 0x41
#define TELEMLOG_T_INFO 0x42
#define TELEMLOG_LOG_HDR 14   // u8 type, u8 stream, u32 boot_id, u32 seq, u32 ms
#define TELEMLOG_REQ_LEN 18   // u8 type, u8 stream, u32 boot_id, u32 from_seq, u64 mask
#define TELEMLOG_INFO_LEN 41  // u8 type, u32 boot_id, u32 prev_boot_id, then {oldest, newest} for s0 cur, s1 cur, s0 prev, s1 prev;
                              // no record made: both 0xFFFFFFFF; records made but none held: oldest = newest + 1
#define TELEMLOG_NO_SEQ 0xFFFFFFFFu

#define TELEMLOG_LIVE_FAST_MAX_BACKLOG 3   // live FAST frames further behind are skipped (they stay in the ring)
#define TELEMLOG_BACKFILL_PERIOD_MS 20     // 50 backfill frames per second at most
#define TELEMLOG_INFO_PERIOD_MS 1000
#define TELEMLOG_REQ_QUEUE 8

// Returns true if the frame was queued for transmission.
typedef bool (*telemlog_send_fn)(const uint8_t mac[6], const uint8_t *buf, size_t len);

typedef struct {
  uint32_t oldest, newest;  // TELEMLOG_NO_SEQ for both if the boot holds no record
} telemlog_range_t;

typedef struct {
  uint32_t boot_id;
  uint32_t prev_boot_id;          // 0 if no previous boot is held
  telemlog_range_t r[2][2];       // [stream][0 = current boot, 1 = previous boot]
} telemlog_info_t;

typedef struct {
  uint32_t backfill_sent_last_s;  // backfill frames sent in the last whole second
  uint32_t backfill_sent;         // since boot
  uint32_t live_sent;
  uint32_t live_skipped;          // live frames not sent because the backlog was too long
  uint32_t send_fail;             // send_fn returned false (live, backfill or INFO)
  uint32_t req_received;
  uint32_t req_dropped;           // queue full or malformed
  uint32_t add_refused;           // body too long or bad stream
  uint8_t prev_kept;              // 1 if the rings survived the last reset
} telemlog_counters_t;

// keep_previous: true after a warm reset (not power-on, not brown-out). If false, or if the
// no-init RAM does not hold a valid log, both rings start empty.
void telemlog_init(uint32_t boot_id, bool keep_previous);

// Returns the seq of the new record, or TELEMLOG_NO_SEQ if refused.
// FAST bodies up to TELEMLOG_FAST_BODY_MAX bytes, SLOW up to TELEMLOG_SLOW_BODY_MAX.
uint32_t telemlog_add(int stream, uint32_t ms, const uint8_t *body, size_t len);

// Copies a held record's body to buf. Returns its length, or 0 if not held (or corrupt).
size_t telemlog_get(int stream, uint32_t boot_id, uint32_t seq, uint8_t *buf, size_t cap,
                    uint32_t *ms);

void telemlog_info(telemlog_info_t *out);
size_t telemlog_info_frame(uint8_t *buf, size_t cap);  // the INFO frame bytes
void telemlog_counters(telemlog_counters_t *out);

// From the ESP-NOW receive callback. Queues a REQ frame; ignores every other frame.
void telemlog_on_frame(const uint8_t mac[6], const uint8_t *data, int len);

// From loop(). Per call: at most one live FAST frame, one live SLOW frame, one backfill frame,
// and the INFO frame when it is due. Never waits; a failed send is counted and dropped.
void telemlog_poll(uint32_t now_ms, telemlog_send_fn send);

// ---- SLOW flash ring (optional) -------------------------------------------------------------
// SLOW records are also kept in a flash partition, so they survive a power cycle and reach back
// hours. The caller owns the policy: when an erase is allowed, and how the flash is reached.
// Layout: 4 KB sectors of 16 pages of 256 B. Page = u32 page_seq, u32 ~page_seq, then entries
// {u8 n, n bytes LOG frame, u16 crc16}, never across a page; unused bytes stay 0xFF.
// read() should use a memory-mapped partition (esp_partition_mmap): esp_partition_read() turns
// the cache off, like erase and write, and stalls interrupts whose code is not in IRAM.
#define TELEMLOG_T_FLASHINFO 0x43          // u8 type, u8 n, n x {u32 boot_id, u32 oldest, u32 newest}
#define TELEMLOG_FLASHINFO_MAX 20          // newest boot first
#define TELEMLOG_FLASHINFO_PERIOD_MS 5000
#ifndef TELEMLOG_FLASH_ERASED_AHEAD
#define TELEMLOG_FLASH_ERASED_AHEAD 4      // sectors kept erased ahead of the write position
#endif
#define TELEMLOG_FLASH_SECTOR 4096
#define TELEMLOG_FLASH_PAGE 256

typedef struct {
  bool (*erase_sector)(uint32_t sector);                      // erase sector n of the partition
  bool (*write)(uint32_t offset, const void *data, size_t n);  // program; never crosses a page
  bool (*read)(uint32_t offset, void *data, size_t n);
  uint32_t sectors;                                            // partition size / 4096
} telemlog_flash_t;

typedef struct {
  uint32_t pages_written, sectors_erased;
  uint32_t entries_skipped;  // no erased page was ready; the record is still in RAM
  uint32_t write_fail, erase_fail;
  uint32_t used_sectors, erased_sectors;
} telemlog_flash_counters_t;

// After telemlog_init(). Reads every sector once to rebuild the index (2 KB heap for 128 sectors).
bool telemlog_flash_attach(const telemlog_flash_t *flash);
// From loop(): programs at most one full staged page, and, if allow_erase, erases at most one
// sector to keep TELEMLOG_FLASH_ERASED_AHEAD sectors ready. The oldest sector is erased first.
void telemlog_flash_service(uint32_t now_ms, bool allow_erase);
void telemlog_flash_counters(telemlog_flash_counters_t *out);
size_t telemlog_flashinfo_frame(uint8_t *buf, size_t cap);
