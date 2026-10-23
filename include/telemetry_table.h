// Telemetry table: one line per logged value (docs/espnow_protocol.md, "Telemetry definition").
//
// No include guard: this is an X-macro list. A user defines TLR, TLF and TLM (each one it needs), includes this file,
// then undefines them; a macro left undefined here takes no lines. tools/gen_telemetry_def.py reads the same lines and
// writes telemetry_def.json (the build folder; docs/telemetry_def.json for the default build). lib/tlm builds the table
// hash from the same lines at compile time, the value names TLM_<name>, and the records of the table values.
//
// Three kinds of line. One line each; no line breaks inside a line.
//
// TLR(body, name, stream, rate, header, block, count_offset, block_offset, stride, min_bytes, consts)
//   A fixed record: its bytes are packed by hand (lib/rover_telemetry, src/platform/telemetry.inc); these lines describe them.
//   body          the body type byte
//   name          the "body" name the reader gives the record (tools/rover_log.py)
//   stream        0 FAST, 1 SLOW
//   rate          records a second: a number, LOG (telemetry_rate_hz), BINS (telemetry_bins_hz), CHANGE (on a
//                 change), SWEEP (each sweep page)
//   header        "LAYOUT ..." the TLF layouts of the header fields; offsets count from the byte after the body type
//   block         "LAYOUT[@base] ..." the TLF layouts of one repeated block (a sample), base added to their offsets;
//                 "SPECIAL": the rest of the record is described in docs/espnow_protocol.md only
//   count_offset  the u8 block count n (header offset), or - for a record with no blocks
//   block_offset  the first block (header offset), or -
//   stride        bytes a block, or -
//   min_bytes     the shortest body that decodes (bytes after the body type); a field past the end has no value
//   consts        "key=value ..." fixed keys the reader adds to the record (JSON values)
//
// TLF(layout, name, type, offset, scale, unit, decimals, kind, none, group, label, names)
//   A field of a fixed record.
//   layout    the group of fields a TLR line names
//   name      the key in telemetry_def.json, tools/rover_log.py and the telemetry app; name[n] or name[n][m] is an
//             array of contiguous elements, named name_i or name_i_j, listed by tools/rover_log.py as name: [...]
//   type      U8, I8, U16, I16, U24, I24, U32, I32, F32, BF16 (top 16 bits of a float32), TEXT (to the body end),
//             PTEXT (u8 length, then the text), VERSION (3 x u8, major.minor.patch), UNIXMS (u32 s, u16 ms;
//             s * 1000 + ms); U8>>4: the top 4 bits; U8&15: the low 4 bits
//   offset    bytes from the layout start (the header: from the byte after the body type)
//   scale     shown value = wire value x scale; a number or a fraction a/b
//   unit      text for the app; "" for none
//   decimals  decimals on the screen
//   kind      VALUE, BITS (one 0/1 value per bit name; the field name names the set), ENUM (a name per code), TEXT,
//             TIME (ms after the record time), HEX (8 hex digits)
//   none      the wire code for "no value" (compared before a shift or mask), or -
//   group     the plot group in the app
//   label     the name on the screen
//   names     "|"-separated: BITS: the bit names from bit 0 ("" for an unused bit); ENUM: code=name, *=the name of an
//             unknown code (else the code), @field=value: the field exists only when that field has that value;
//             an array: one label part per element of the last index
//
// TLM(name, type, scale, unit, decimals, rate, group, label)
//   A table value: user code sets it with tlm_set(TLM_<name>, value); lib/tlm packs it into the record of its rate.
//   name      the key in telemetry_def.json, the telemetry app and the CSV. Keep a name for good: the app
//             and old logs know the value by it.
//   type      the wire type: U8, I8, U16, I16, U32, I32 or F32
//   scale     a plain number, not 0: shown value = wire value x scale. The firmware sends round(value / scale),
//             clamped to the type; the top code of each type (U8 255, I8 -128, U16 65535, I16 -32768,
//             U32 4294967295, I32 -2147483648; F32 NaN) means "no value" (NaN given, or never set).
//             tlm_set keeps a float: whole numbers are exact up to 16777216.
//   unit      text for the app, for example "mm", "V", "deg/s"; "" for none
//   decimals  decimals on the screen
//   rate      1HZ (body 0x34, stream 1), 10HZ (0x35, stream 0) or LOG (0x36, the log rate, telemetry_rate_hz; stream 0).
//             A record carries the last value set before its sample time: no filter.
//   group     the plot group in the app
//   label     the name on the screen
//   The values of one rate, in table order, make one sample of that rate's record. Bytes a sample at most:
//   LOG 16, 10HZ 64, 1HZ 77 (the build stops past it).
//
// Your lines go in include/user_telemetry.h (TLM lines).

#ifndef TLR
#define TLR(body, name, stream, rate, header, block, count_offset, block_offset, stride, min_bytes, consts)
#define TLM_TABLE_NO_TLR
#endif
#ifndef TLF
#define TLF(layout, name, type, offset, scale, unit, decimals, kind, none, group, label, names)
#define TLM_TABLE_NO_TLF
#endif
#ifndef TLM
#define TLM(name, type, scale, unit, decimals, rate, group, label)
#define TLM_TABLE_NO_TLM
#endif

// Fixed records.
//  body  name       str rate    header       block                    cnt blk stride min consts
TLR(0x21, status,    1,  1,      "STATUS",    "",                      -,  -,  -,     66, "")
TLR(0x2A, samples,   0,  100,    "",          "SAMPLE SAMPLE3",        0,  1,  50,    1,  "layout=3 imu=false rate_hz=100")
TLR(0x2B, samples,   0,  100,    "",          "SAMPLE SAMPLE3 IMU@50", 0,  1,  60,    1,  "layout=3 imu=true rate_hz=100")
TLR(0x2C, samples,   0,  LOG,    "DEC",       "SAMPLE SAMPLE3",        0,  2,  50,    2,  "layout=3 imu=false")
TLR(0x2D, samples,   0,  LOG,    "DEC",       "SAMPLE SAMPLE3 IMU@50", 0,  2,  60,    2,  "layout=3 imu=true")
TLR(0x2E, bins,      0,  BINS,   "BINS",      "",                      -,  -,  -,     115, "")
TLR(0x30, event,     1,  CHANGE, "EVENT",     "",                      -,  -,  -,     10, "")
TLR(0x31, name,      1,  0.1,    "NAME",      "",                      -,  -,  -,     2,  "")
TLR(0x33, sweep_raw, 0,  SWEEP,  "SWEEP_RAW", "SPECIAL",               -,  -,  -,     11, "")
TLR(0x37, polar,     0,  31,     "",          "SPECIAL",               -,  -,  -,     1,  "")
// Records of older firmware, for old logs: sample layout 1 (before 0.5.58), sweep with grid points (0.5.87 to 0.5.102).
TLR(0x20, samples,   0,  100,    "",          "SAMPLE",                0,  1,  32,    1,  "layout=1 imu=false rate_hz=100")
TLR(0x22, samples,   0,  100,    "",          "SAMPLE IMU@32",         0,  1,  42,    1,  "layout=1 imu=true rate_hz=100")
TLR(0x24, samples,   0,  LOG,    "DEC",       "SAMPLE",                0,  2,  32,    2,  "layout=1 imu=false")
TLR(0x25, samples,   0,  LOG,    "DEC",       "SAMPLE IMU@32",         0,  2,  42,    2,  "layout=1 imu=true")
TLR(0x32, sweep,     0,  SWEEP,  "SWEEP",     "SPECIAL",               -,  -,  -,     10, "")

// Status, body 0x21 (offsets from the byte after the body type).
//  layout  name             type     off scale unit     dec kind   none    group        label                                        names
TLF(STATUS, drive_mode,      U8,      0,  1,    "",      0,  VALUE, -,      "Vehicle",   "Drive mode (0 manual, 1 cruise, 2 autonomous)", "")
TLF(STATUS, charge_step,     I8,      1,  1,    "",      0,  VALUE, -,      "Charge",    "Charge state machine step",                 "")
TLF(STATUS, vehicle_status,  U8,      2,  1,    "",      0,  VALUE, -,      "Vehicle",   "Vehicle status (0 stuck, 1 driving, 2 charging)", "")
TLF(STATUS, vbatt_mv,        U16,     3,  1,    "mV",    0,  VALUE, -,      "Power",     "Battery voltage",                           "")
TLF(STATUS, vcharge_mv,      U16,     5,  1,    "mV",    0,  VALUE, -,      "Charge",    "Charge voltage",                            "")
TLF(STATUS, icharge_ma,      I16,     7,  1,    "mA",    0,  VALUE, -,      "Charge",    "Charge current",                            "")
TLF(STATUS, v5v_mv,          U16,     9,  1,    "mV",    0,  VALUE, -,      "Power",     "5 V rail",                                  "")
TLF(STATUS, temperature_c,   I8,      11, 1,    "degC",  0,  VALUE, -,      "Rx STM32",  "Rx STM32 temperature",                      "")
TLF(STATUS, ap_state,        U8,      12, 1,    "",      0,  VALUE, -,      "Autopilot", "Autopilot state (255 off)",                 "")
TLF(STATUS, ap_sub,          U8,      13, 1,    "",      0,  VALUE, -,      "Autopilot", "Autopilot sub-step",                        "")
TLF(STATUS, flags,           U8,      14, 1,    "",      0,  VALUE, -,      "Vehicle",   "Status flags",                              "")
TLF(STATUS, status_flags,    U8,      14, 1,    "",      0,  BITS,  -,      "Vehicle",   "Status flag",                               "go_home|gyro|can|espnow|rover_enabled|locked|imu_fake|moving")
TLF(STATUS, status_bits,     U32,     15, 1,    "",      0,  VALUE,  -,      "Rx STM32",  "Rx STM32 status bits",                      "")
TLF(STATUS, charge_code,     U32,     19, 1,    "",      0,  VALUE, -,      "Charge",    "Rx STM32 charge code",                      "")
TLF(STATUS, crc_errors,      U16,     23, 1,    "count", 0,  VALUE, -,      "Link",      "CRC errors, total (wraps)",                 "")
TLF(STATUS, overruns,        U16,     25, 1,    "count", 0,  VALUE, -,      "Link",      "UART overruns, total (wraps)",              "")
TLF(STATUS, packets_last_s,  U16,     27, 1,    "count", 0,  VALUE, -,      "Link",      "Frames in the last second",                 "")
TLF(STATUS, loop_max_us,     U16,     29, 1,    "us",    0,  VALUE, -,      "ESP32",     "Longest frame handling in the last second", "")
TLF(STATUS, bounces,         U16,     31, 1,    "count", 0,  VALUE, -,      "Autopilot", "Bounces",                                   "")
TLF(STATUS, turnarounds,     U16,     33, 1,    "count", 0,  VALUE, -,      "Autopilot", "Turnarounds",                               "")
TLF(STATUS, charges,         U16,     35, 1,    "count", 0,  VALUE, -,      "Charge",    "Charges",                                   "")
TLF(STATUS, odrive_vbus_mv,  U16,     37, 1,    "mV",    0,  VALUE, -,      "ODrive",    "ODrive node 0 bus voltage",                 "")
TLF(STATUS, odrive_ibus_ma,  I16,     39, 1,    "mA",    0,  VALUE, -,      "ODrive",    "ODrive node 0 bus current",                 "")
TLF(STATUS, axis_state[6],   U8,      41, 1,    "",      0,  VALUE, -,      "ODrive",    "ODrive axis state",                         "node 0|node 1|node 2|node 3|node 4|node 5")
TLF(STATUS, axis_errors_or,  U32,     47, 1,    "",      0,  VALUE, -,      "ODrive",    "ODrive axis errors, OR of all nodes",       "")
TLF(STATUS, error_nodes,     U8,      51, 1,    "",      0,  VALUE, -,      "ODrive",    "Nodes with an axis error (bit n = node n)", "")
TLF(STATUS, heartbeat_nodes, U8,      52, 1,    "",      0,  VALUE, -,      "ODrive",    "Nodes with a heartbeat (bit n = node n)",   "")
TLF(STATUS, free_heap,       U32,     53, 1,    "B",     0,  VALUE, -,      "ESP32",     "Free heap",                                 "")
TLF(STATUS, send_fail,       U16,     57, 1,    "count", 0,  VALUE, -,      "Link",      "ESP-NOW send failures, total (wraps)",      "")
TLF(STATUS, backfill_last_s, U16,     59, 1,    "count", 0,  VALUE, -,      "Link",      "Backfill frames in the last second",        "")
TLF(STATUS, xf_mm,           I16,     61, 1,    "mm",    0,  VALUE, -32768, "Autopilot", "Crossing point, filtered",                  "")
TLF(STATUS, dist_mm,         U16,     63, 1,    "mm",    0,  VALUE, -,      "Autopilot", "Wire distance, filtered",                   "")
TLF(STATUS, angle_deg,       U8,      65, 1,    "deg",   0,  VALUE, -,      "Autopilot", "Heading to the wire at the last decision", "")
TLF(STATUS, monitor_age_s,   U8,      66, 1,    "s",     0,  VALUE, -,      "Link",      "Time since the last MON frame (255 none)",  "")
TLF(STATUS, pc_unix_ms,      UNIXMS,  67, 1,    "ms",    0,  VALUE, 0,      "Time",      "PC time at this record (unix)",             "")
TLF(STATUS, fw_version,      VERSION, 73, 1,    "",      0,  TEXT,  -,      "Vehicle",   "ESP32 firmware version",                    "")
TLF(STATUS, cpu_busy_pct,    U8,      76, 1,    "%",     0,  VALUE, -,      "ESP32",     "ESP32 load, all tasks but idle",            "")
TLF(STATUS, cpu_loop_pct,    U8,      77, 1,    "%",     0,  VALUE, -,      "ESP32",     "ESP32 load, loop()",                        "")
TLF(STATUS, cpu_drive_pct,   U8,      78, 1,    "%",     0,  VALUE, -,      "ESP32",     "ESP32 load, drive parts of loop()",         "")
TLF(STATUS, dsp_period_us,   U32,     79, 1,    "us",    0,  VALUE, 0,      "Rx STM32",  "Update period in effect",                   "")

// Sample header of a log rate below 100 Hz (bodies 0x2C, 0x2D, 0x24, 0x25).
TLF(DEC,    rate_hz,         U8,      1,  1,    "Hz",    0,  VALUE, -,      "Log",       "Sample rate",                               "")

// One sample, bytes 0 to 31 (layouts 1 and 3).
TLF(SAMPLE, ms,              U16,     0,  1,    "ms",    0,  TIME,  -,      "Time",      "Sample time",                               "")
TLF(SAMPLE, x1_mm,           I16,     2,  1,    "mm",    0,  VALUE, -,      "Position",  "Sensor 1 x",                                "")
TLF(SAMPLE, y1_mm,           I16,     4,  1,    "mm",    0,  VALUE, -,      "Position",  "Sensor 1 y",                                "")
TLF(SAMPLE, r1_mm,           U16,     6,  1,    "mm",    0,  VALUE, -,      "Position",  "Sensor 1 wire radius",                      "")
TLF(SAMPLE, sample_flags,    U8,      8,  1,    "",      0,  BITS,  -,      "Flags",     "Sample flag",                               "lost|overload|disabled|weak|entry_btx|exit_btx|charging|wheels_fresh")
TLF(SAMPLE, ap_state,        U8>>4,   9,  1,    "",      0,  ENUM,  255,    "Autopilot", "Autopilot state",                           "0=roam|1=return|2=join|3=follow|4=depart|5=fault|6=dock")
TLF(SAMPLE, ap_sub,          U8&15,   9,  1,    "",      0,  VALUE, 255,    "Autopilot", "Autopilot sub-step",                        "")
TLF(SAMPLE, drive,           I16,     10, 0.0001, "",    4,  VALUE, -,      "Drive",     "Drive command (fraction of full)",          "")
TLF(SAMPLE, steer,           I16,     12, 0.0001, "",    4,  VALUE, -,      "Drive",     "Steer command (fraction of full)",          "")
TLF(SAMPLE, vel0_rps,        I16,     14, 0.001, "rev/s", 3, VALUE, -,      "Wheels",    "Wheel speed, node 0",                       "")
TLF(SAMPLE, vel1_rps,        I16,     16, 0.001, "rev/s", 3, VALUE, -,      "Wheels",    "Wheel speed, node 1",                       "")
TLF(SAMPLE, gyro_dps,        I16,     18, 0.02, "deg/s", 2,  VALUE, -32768, "IMU",       "Gyro z",                                    "")
TLF(SAMPLE, btx1_mm,         U8,      20, 4,    "mm",    0,  VALUE, -,      "Beacons",   "Sensor 1 beacon 1 distance",                "")
TLF(SAMPLE, btx2_mm,         U8,      21, 4,    "mm",    0,  VALUE, -,      "Beacons",   "Sensor 1 beacon 2 distance",                "")
TLF(SAMPLE, coil[4],         BF16,    22, 1,    "",      6,  VALUE, -,      "Coils",     "Wire bin coil value",                       "sensor 1 coil 1|sensor 1 coil 2|sensor 2 coil 1|sensor 2 coil 2")
TLF(SAMPLE, xf_mm,           I16,     30, 1,    "mm",    0,  VALUE, -32768, "Autopilot", "Crossing point, filtered",                  "")

// Layout 3, bytes 32 to 49.
TLF(SAMPLE3, s2_btx1_mm,     U8,      32, 4,    "mm",    0,  VALUE, -,      "Beacons",   "Sensor 2 beacon 1 distance",                "")
TLF(SAMPLE3, s2_btx2_mm,     U8,      33, 4,    "mm",    0,  VALUE, -,      "Beacons",   "Sensor 2 beacon 2 distance",                "")
TLF(SAMPLE3, enc_pos_turn[2], I32,    34, 0.0001, "turn", 4, VALUE, -,      "Wheels",    "Wheel position",                            "node 0|node 1")
TLF(SAMPLE3, enc_age_ms[2],  U8,      42, 1,    "ms",    0,  VALUE, 255,    "Wheels",    "Wheel position age",                        "node 0|node 1")
TLF(SAMPLE3, x2_mm,          I16,     44, 1,    "mm",    0,  VALUE, -32768, "Position",  "Sensor 2 x",                                "")
TLF(SAMPLE3, y2_mm,          I16,     46, 1,    "mm",    0,  VALUE, -32768, "Position",  "Sensor 2 y",                                "")
TLF(SAMPLE3, heading_deg,    U16,     48, 360/65536, "deg", 3, VALUE, -,    "IMU",       "Gyro heading (wraps at 360)",               "")

// The IMU axes (after byte 31 in layout 1, after byte 49 in layout 3).
TLF(IMU,    ax_g,            I16,     0,  0.001, "g",    3,  VALUE, -,      "IMU",       "Accelerometer x",                           "")
TLF(IMU,    ay_g,            I16,     2,  0.001, "g",    3,  VALUE, -,      "IMU",       "Accelerometer y",                           "")
TLF(IMU,    az_g,            I16,     4,  0.001, "g",    3,  VALUE, -,      "IMU",       "Accelerometer z",                           "")
TLF(IMU,    gx_dps,          I16,     6,  0.02, "deg/s", 2,  VALUE, -,      "IMU",       "Gyro x",                                    "")
TLF(IMU,    gy_dps,          I16,     8,  0.02, "deg/s", 2,  VALUE, -,      "IMU",       "Gyro y",                                    "")

// Coil values of all 8 bins, body 0x2E.
TLF(BINS,   filter_n,        U8,      0,  1,    "",      0,  VALUE, -,      "Bins",      "Filter factor (100 Hz / rate)",             "")
TLF(BINS,   period_ms,       U16,     1,  1,    "ms",    0,  VALUE, -,      "Bins",      "Output period",                             "")
TLF(BINS,   hz[8],           U16,     3,  2,    "Hz",    0,  VALUE, 0,      "Bins",      "Bin frequency",                             "")
TLF(BINS,   coils[8][4],     I24,     19, 1,    "LSB",   0,  VALUE, -8388608, "Bins",    "Bin coil value",                            "sensor 1 coil 1|sensor 1 coil 2|sensor 2 coil 1|sensor 2 coil 2")

// Event, body 0x30.
TLF(EVENT,  code,            U16,     0,  1,    "",      0,  VALUE, -,      "Event",     "Event code",                                "")
TLF(EVENT,  event,           U16,     0,  1,    "",      0,  ENUM,  -,      "Event",     "Event",                                     "1=boot|2=mode|3=ap_state|4=stuck|5=charge_step|6=odrive_error|7=halted|8=setting|9=stall|10=bad_key|11=run|12=imu|13=restart_refused|14=prev_boot|15=stm32|16=time|17=monitor|18=stm32_cmd|19=wifi|20=usb|21=settings|22=setting_change|*=?")
TLF(EVENT,  a,               I32,     2,  1,    "",      0,  VALUE, -,      "Event",     "Event value a",                             "")
TLF(EVENT,  b,               I32,     6,  1,    "",      0,  VALUE, -,      "Event",     "Event value b",                             "")
TLF(EVENT,  text,            TEXT,    10, 1,    "",      0,  TEXT,  -,      "Event",     "Event text",                                "")
TLF(EVENT,  kind,            I32,     2,  1,    "",      0,  ENUM,  -,      "Event",     "Rx STM32 command link event",               "@code=18|1=no_answer|2=wrong_answer|3=read_back_differs|4=packet_gap|5=queue_full|6=hello_boot|7=stm_reset|8=refused|9=cal_restore|*=?")

// Vehicle name, body 0x31.
TLF(NAME,   vehicle_id,      U8,      0,  1,    "",      0,  VALUE, -,      "Vehicle",   "Vehicle number",                            "")
TLF(NAME,   name,            PTEXT,   1,  1,    "",      0,  TEXT,  -,      "Vehicle",   "Vehicle name",                              "")

// Sweep headers, bodies 0x33 and 0x32 (the readings after them: docs/espnow_protocol.md).
TLF(SWEEP_RAW, table_id,     U8,      0,  1,    "",      0,  VALUE, -,      "Sweep",     "Sweep table id",                            "")
TLF(SWEEP_RAW, f0_hz,        U24,     1,  0.1,  "Hz",    1,  VALUE, -,      "Sweep",     "Frequency of grid point 0",                 "")
TLF(SWEEP_RAW, step_hz,      U16,     4,  0.1,  "Hz",    1,  VALUE, -,      "Sweep",     "Grid step",                                 "")
TLF(SWEEP_RAW, points,       U16,     6,  1,    "",      0,  VALUE, -,      "Sweep",     "Grid points",                               "")
TLF(SWEEP_RAW, sweep_bins,   U8,      8,  1,    "",      0,  VALUE, -,      "Sweep",     "Sweep bins",                                "")
TLF(SWEEP_RAW, mask,         U8,      9,  1,    "",      0,  VALUE, -,      "Sweep",     "Sweep bin mask",                            "")
TLF(SWEEP,  table_id,        U8,      0,  1,    "",      0,  VALUE, -,      "Sweep",     "Sweep table id",                            "")
TLF(SWEEP,  f0_hz,           U24,     1,  0.1,  "Hz",    1,  VALUE, -,      "Sweep",     "Frequency of grid point 0",                 "")
TLF(SWEEP,  step_hz,         U16,     4,  0.1,  "Hz",    1,  VALUE, -,      "Sweep",     "Grid step",                                 "")
TLF(SWEEP,  points,          U16,     6,  1,    "",      0,  VALUE, -,      "Sweep",     "Grid points",                               "")
TLF(SWEEP,  sweep_bins,      U8,      8,  1,    "",      0,  VALUE, -,      "Sweep",     "Sweep bins",                                "")

#include "user_telemetry.h"

#ifdef TLM_TABLE_NO_TLR
#undef TLR
#undef TLM_TABLE_NO_TLR
#endif
#ifdef TLM_TABLE_NO_TLF
#undef TLF
#undef TLM_TABLE_NO_TLF
#endif
#ifdef TLM_TABLE_NO_TLM
#undef TLM
#undef TLM_TABLE_NO_TLM
#endif
