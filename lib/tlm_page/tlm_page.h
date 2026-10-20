// Telemetry for the page /telemetry (src/web_telemetry.cpp): the field list and the values as JSON, from the
// telemetry table (include/telemetry_table.h) at run time. No hand list of fields, labels or units: a new table line
// shows on the page with no other change.
//
// Fields: the status record (body 0x21), the newest log sample (layout 3, bodies 0x2A to 0x2D, the IMU axes when the
// sample has them) and every table value (TLM lines). A status field whose name the sample also has (ap_state, ap_sub,
// xf_mm) is left out: the sample is newer. TIME fields (the sample time) are left out: the answer gives the sample age.
// Each value is the log value (the same bytes, decoded by the table), shown with the decimals of its table line.
//
// loop() copies the newest status body, sample and table values into a Snapshot (take_status, take_sample,
// take_values); the web server task formats it (values_answer). No Arduino dependency: builds on a PC.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "tlm.h"

namespace tlmpage {

const int STATUS_MAX = 84;  // rec::STATUS_BYTES
const int SAMPLE_MAX = 60;  // rec::IMU_SAMPLE3_BYTES

struct Snapshot {
  uint32_t now_ms;       // millis() at the copy
  uint32_t status_ms;    // record time of the status body
  uint32_t sample_ms;    // time of the sample
  uint8_t status_len;    // bytes after the body type; 0: no status yet
  uint8_t sample_len;    // bytes of the sample; 0: no sample yet
  uint8_t sample_imu;    // 1: the sample has the IMU axes
  uint8_t status[STATUS_MAX];  // the status body after the body type
  uint8_t sample[SAMPLE_MAX];  // one sample
  float tlm[TLM_COUNT + 1];    // the table values as set (g_tlm_value)
};

// loop(): the newest status body (with its body type byte).
void take_status(Snapshot* s, const uint8_t* body, size_t len, uint32_t ms);
// loop(): the newest status record of this boot in the SLOW ring of the log (lib/telemlog). If the ring holds none
// (more than TELEMLOG_SLOW_SLOTS newer records), the copy keeps the status it has; its age shows it.
void take_status_log(Snapshot* s);
// loop(): the newest sample of the open sample record (rec::Block body()). After a full record went out, the record
// buffer still holds it, and its count byte says how many samples it has. block_ms: rec::Block ms().
void take_sample(Snapshot* s, const uint8_t* block_body, uint32_t block_ms);
void take_values(Snapshot* s);  // the table values

typedef bool (*sink_fn)(void* ctx, const char* text, size_t len);  // false: stop (the connection failed)

struct Heap { uint32_t free, min, max_block; };

// GET /api/telemetry?def=1: {"fw","hash","count","fields":[{"n","r","g","l","u","d","k","p"[,"b"|"e"]},...]}
//   n name (the log key; an array element name_i), r record (status, samples, tlm), g group, l label, u unit,
//   d decimals, k kind (v value, bits, enum, text), p place in the short set (1, 2, ...; 0 = under "more"),
//   b bit names from bit 0 ("|"-separated), e enum names (code=name, "|"-separated).
bool def_answer(const char* fw, sink_fn sink, void* ctx);
// GET /api/telemetry[?all=1]: {"ms","st","sa","heap","heap_min","maxblk","v":[...]}
//   ms uptime at the copy; st, sa the age of the status and of the sample in ms (null: none yet); v one value per
//   field in field order (all) or per field with p > 0 (short set); null = no value. bits and enum: the code.
bool values_answer(const Snapshot& s, bool all, const Heap& h, sink_fn sink, void* ctx);

int count();               // fields on the page
int short_count();         // fields in the short set
const char* name(int i);   // field i, for the desk test
int missing_short();       // names of the short list that the table does not have (0 expected)

}  // namespace tlmpage
