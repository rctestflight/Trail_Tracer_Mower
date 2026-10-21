// User telemetry lines: the columns are in include/telemetry_table.h. No include guard (X-macro list).
// One line here and one tlm_set(TLM_<name>, value) call in your code: the value then goes into the log, the
// telemetry app and the CSV of tools/rover_log.py, with no other edit.
//
// The example lines build only in env telemetry_newvalue (-D TELEMETRY_NEWVALUE); src/main.cpp sets them.

#ifdef TELEMETRY_NEWVALUE
//  name          type scale   unit     dec rate  group   label
TLM(test_frames,  U16, 1,      "count", 0,  LOG,  "Test", "Rx STM32 frames, mod 65536")
TLM(test_r1_mm,   U16, 1,      "mm",    0,  LOG,  "Test", "Wire radius, sensor 1")
TLM(test_drive,   I16, 0.0001, "",      3,  10HZ, "Test", "Drive command")
TLM(test_heap_kb, U16, 0.1,    "kB",    1,  1HZ,  "Test", "Free heap")
#endif

// Your lines:
// 0.6.11: dual tone polarity detection, Rx STM32 2.1.134+ extension items 9 (sign) and 10 (sign angle spread), per sensor and
// Tx channel; set in src/platform/telemetry.inc (dtTelemetry). No value when the item is older than 1 s (or never came).
//  name                type scale unit  dec rate  group                 label
TLM(dt_decoded,        U8,  1,   "",    0, 1HZ,  "Polarity detection", "Tx channels decoded now (bit 0 = CH1 .. bit 4 = CH5)")
TLM(follow_dir,        I8,  1,   "",    0, 1HZ,  "Drive",              "Direction round the perimeter (+1 clockwise, -1 counterclockwise, 0 not known)")
TLM(dt_sign_s1_ch1,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 1, Tx channel 1 (+1, -1; 0 none yet)")
TLM(dt_sign_s1_ch2,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 1, Tx channel 2 (+1, -1; 0 none yet)")
TLM(dt_sign_s1_ch3,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 1, Tx channel 3 (+1, -1; 0 none yet)")
TLM(dt_sign_s1_ch4,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 1, Tx channel 4 (+1, -1; 0 none yet)")
TLM(dt_sign_s1_ch5,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 1, Tx channel 5 (+1, -1; 0 none yet)")
TLM(dt_sign_s2_ch1,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 2, Tx channel 1 (+1, -1; 0 none yet)")
TLM(dt_sign_s2_ch2,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 2, Tx channel 2 (+1, -1; 0 none yet)")
TLM(dt_sign_s2_ch3,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 2, Tx channel 3 (+1, -1; 0 none yet)")
TLM(dt_sign_s2_ch4,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 2, Tx channel 4 (+1, -1; 0 none yet)")
TLM(dt_sign_s2_ch5,   I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity sign, sensor 2, Tx channel 5 (+1, -1; 0 none yet)")
TLM(dt_spread_s1_ch1, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 1, Tx channel 1 (over 45 deg: no reliable sign)")
TLM(dt_spread_s1_ch2, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 1, Tx channel 2 (over 45 deg: no reliable sign)")
TLM(dt_spread_s1_ch3, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 1, Tx channel 3 (over 45 deg: no reliable sign)")
TLM(dt_spread_s1_ch4, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 1, Tx channel 4 (over 45 deg: no reliable sign)")
TLM(dt_spread_s1_ch5, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 1, Tx channel 5 (over 45 deg: no reliable sign)")
TLM(dt_spread_s2_ch1, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 2, Tx channel 1 (over 45 deg: no reliable sign)")
TLM(dt_spread_s2_ch2, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 2, Tx channel 2 (over 45 deg: no reliable sign)")
TLM(dt_spread_s2_ch3, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 2, Tx channel 3 (over 45 deg: no reliable sign)")
TLM(dt_spread_s2_ch4, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 2, Tx channel 4 (over 45 deg: no reliable sign)")
TLM(dt_spread_s2_ch5, U16, 0.1, "deg", 1, 10HZ, "Polarity detection", "Sign angle spread, sensor 2, Tx channel 5 (over 45 deg: no reliable sign)")
// 0.6.21: polarity per sensor and Tx channel (Rx STM32 2.2.0+, ext id 12, every frame; set in dtTelemetry), and the
// world facts of src/main.cpp worldFacts().
TLM(dt_polarity_s1_ch1,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 1, Tx channel 1 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s1_ch2,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 1, Tx channel 2 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s1_ch3,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 1, Tx channel 3 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s1_ch4,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 1, Tx channel 4 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s1_ch5,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 1, Tx channel 5 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s2_ch1,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 2, Tx channel 1 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s2_ch2,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 2, Tx channel 2 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s2_ch3,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 2, Tx channel 3 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s2_ch4,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 2, Tx channel 4 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(dt_polarity_s2_ch5,    I8,  1,   "",    0, 10HZ, "Polarity detection", "Polarity, sensor 2, Tx channel 5 (+1 along the tracked field, -1 opposite, 0 unknown)")
TLM(wire_inside,       I8,  1,   "",    0, 10HZ, "Polarity detection", "Inside the loop (+1 inside, -1 outside, 0 not known)")
TLM(wire_heading,      I8,  1,   "",    0, 10HZ, "Polarity detection", "Heading round the loop (+1 clockwise, -1 counterclockwise, 0 not known)")
