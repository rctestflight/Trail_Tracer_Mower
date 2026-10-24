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
// 0.6.28: the rover's last decision (lib/rover_autopilot, Decision), set in src/main.cpp; a new decision: a new seq
TLM(rover_dec_seq,     U16, 1,   "",    0, 10HZ, "Rover decisions", "Decision number (mod 65536): changes at each decision")
TLM(rover_dec_code,    U8,  1,   "",    0, 10HZ, "Rover decisions", "Decision: 1 bounce, 3 join, 4 follow, 5 reverse, 6 beacon, 7 dock, 8 depart, 9 return, 10 fault, 11 Tx off, 12 Tx on, 13 start, 14 turn back, 15 probe, 16 follow time end")
TLM(rover_dec_side,    I8,  1,   "",    0, 10HZ, "Rover decisions", "Wire side used (+1 left, -1 right)")
TLM(rover_dec_psi_deg, I16, 0.1, "deg", 1, 10HZ, "Rover decisions", "Fitted wire heading at the decision (0: no fit)")
TLM(rover_dec_turn_deg, I16, 0.1, "deg", 1, 10HZ, "Rover decisions", "Planned turn (+ left)")
TLM(rover_fault,       U8,  1,   "",    0, 10HZ, "Rover decisions", "Fault: 0 none, 1 stuck, 2 lost, 3 no polarity at a join, 4 polarity says outside at the start (set tx_dir the other way)")
TLM(rover_dec_d_mm,    U16, 1,   "mm",  0, 10HZ, "Rover decisions", "Distance to the wire at the decision")
// 0.6.33: sensor 2 wire radius from the Rx STM32 (set in telemetrySample). Sensor 2 sends only with enable_sensor_2 on.
TLM(s2_radius_mm,      U16, 1,   "mm",  0, LOG,  "Position", "Wire radius, sensor 2 (no value: sensor 2 off or no wire)")
// 0.6.36: the inside filter's belief (src/main.cpp insideFilter); wire_inside changes at 95 % and 5 %.
TLM(wire_inside_pct,   U8,  1,   "%",   0, 10HZ, "Polarity detection", "Inside filter: chance that the sensor is inside the loop (wire_inside changes at 95 % and 5 %)")
