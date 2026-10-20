// Table values of the telemetry log: one line in include/telemetry_table.h (or include/user_telemetry.h) defines a
// value; user code sets it with tlm_set(TLM_<name>, value); tlm::service() packs the values of each rate into one
// record (bodies 0x34, 0x35, 0x36; docs/espnow_protocol.md, "Table values"). No Arduino dependency: builds on a PC.
#pragma once
#include <stddef.h>
#include <stdint.h>

#define TLM_T_U8 0
#define TLM_T_I8 1
#define TLM_T_U16 2
#define TLM_T_I16 3
#define TLM_T_U32 4
#define TLM_T_I32 5
#define TLM_T_F32 6
#define TLM_SIZE_U8 1
#define TLM_SIZE_I8 1
#define TLM_SIZE_U16 2
#define TLM_SIZE_I16 2
#define TLM_SIZE_U32 4
#define TLM_SIZE_I32 4
#define TLM_SIZE_F32 4
#define TLM_R_1HZ 0
#define TLM_R_10HZ 1
#define TLM_R_LOG 2

enum tlm_id : uint16_t {
#define TLM(name, type, scale, unit, decimals, rate, group, label) TLM_##name,
#include "telemetry_table.h"
#undef TLM
  TLM_COUNT
};

// Bytes a sample per rate, and the limits (one record body: 7 B header + samples; SLOW body 84 B, FAST 228 B).
#define TLM(name, type, scale, unit, decimals, rate, group, label) +(TLM_R_##rate == TLM_R_1HZ ? TLM_SIZE_##type : 0)
constexpr int TLM_BYTES_1HZ = 0
#include "telemetry_table.h"
    ;
#undef TLM
#define TLM(name, type, scale, unit, decimals, rate, group, label) +(TLM_R_##rate == TLM_R_10HZ ? TLM_SIZE_##type : 0)
constexpr int TLM_BYTES_10HZ = 0
#include "telemetry_table.h"
    ;
#undef TLM
#define TLM(name, type, scale, unit, decimals, rate, group, label) +(TLM_R_##rate == TLM_R_LOG ? TLM_SIZE_##type : 0)
constexpr int TLM_BYTES_LOG = 0
#include "telemetry_table.h"
    ;
#undef TLM
static_assert(TLM_BYTES_LOG <= 16, "telemetry table: the LOG values take more than 16 B a sample; move a value to 10HZ or 1HZ");
static_assert(TLM_BYTES_10HZ <= 64, "telemetry table: the 10HZ values take more than 64 B a sample; move a value to 1HZ");
static_assert(TLM_BYTES_1HZ <= 77, "telemetry table: the 1HZ values take more than 77 B a sample (the SLOW record limit)");

extern float g_tlm_value[TLM_COUNT + 1];

/**
 * @brief Sets one telemetry value: a store into RAM, nothing else. Call it as often as the value changes (every
 *        update period is fine); each record takes the last value set before its sample time.
 * @param id the value, TLM_<name> from its line in include/telemetry_table.h or include/user_telemetry.h
 * @param v the value (NAN: no value)
 */
inline void tlm_set(tlm_id id, float v) { g_tlm_value[id] = v; }

namespace tlm {

const uint8_t BODY_1HZ = 0x34, BODY_10HZ = 0x35, BODY_LOG = 0x36;
const int HDR = 7;      // u8 body, u32 table hash, u8 sample count n, u8 rate in Hz
const int FORMAT = 2;   // the first line of the hash text: "tlm 2"

// telemlog_add, or a test sink.
typedef uint32_t (*add_fn)(int stream, uint32_t ms, const uint8_t* body, size_t len);

void init(add_fn add);  // every value to "no value", the record buffers (heap, only for a rate in use)
// The table hash: CRC-32 of the text of every table line (fixed records and table values), computed at compile time.
// telemetry_def.json "version"; in every table record, and the answer to "tlm" and "get telemetry_def".
uint32_t hash();
int count();            // TLM_COUNT
// The value that the log gives for v (round(v / scale), clamped to the type, times the scale); NAN for "no value".
// For the page /telemetry: the page shows the log value.
double logged(int id, float v);
// From loop(), outside the frame path. log_rate_hz: an allowed log rate (100, 50, 25, 20, 10, 5, 4, 2 or 1 Hz).
void service(uint32_t now_ms, int log_rate_hz);

}  // namespace tlm
