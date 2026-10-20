// Settings table: one line per setting.
//
// No include guard: this is an X-macro list. A user defines S and SF, includes this file, then undefines them.
// src/usb_msc_fat.cpp builds the table from it; include/usb_msc_fat.h adds the SF fields to DeviceConfig;
// tools/gen_settings_def.py reads the same lines and writes docs/settings_def.json.
//
// S(name, field, type, default, min, max, opts, nvs, flags, unit, group, label, help)
//   name     config.json key, and the key of set, get and list
//   field    the DeviceConfig field (hand-written in usb_msc_fat.h)
//   type     T_TEXT (char array, NVS string), T_U8 (uint8_t, NVS u8), T_I32 (int, NVS i32),
//            T_IBOOL (int 0/1, NVS i32, JSON true/false), T_BOOL (bool, NVS u8), T_F32 (float, NVS 4-byte blob),
//            T_F64 (double, NVS 8-byte blob), T_LONG (long, NVS i32), T_CHOICE (uint8_t, NVS u8, JSON text)
//   default  the value at first start and after an erase (T_TEXT: in opts; T_CHOICE: the stored number)
//   min, max the range check; NOLIM = no limit on that side (T_TEXT: max = length)
//   opts     T_CHOICE: "name|name|name=value"; a name without "=value" stores its position. T_TEXT: the default text
//   nvs      the NVS key in namespace "wfrx_cfg", 15 characters at most. Never change a key or its type:
//            the value saved by older firmware is then lost.
//   flags    F_ZERO_OFF: 0 = off, else min to max. F_LEGACY: read from an older config.json only (no write, no
//            list, get or set). F_CHOICE_INT: config.json may also give the stored number. F_LIVE: (app) takes
//            effect at once.
//   unit, group, label, help   for the app definition (docs/settings_def.json). Empty: the generator takes the
//            words of the key from the committed settings_def.json (first step).
// SF(name, ctype, type, ...) is the same, but the table declares the DeviceConfig field: `ctype name;`.
// Add a setting: one SF line here, then use cfg.<name> where it acts.
//
// The order is the config.json key order and the order of "list".

S(vehicle_name,          vehicle_name,          T_TEXT,   0,     1,     16,     "Vehicle", "vName",    0,          "", "", "", "")
S(vehicleID,             vehicleID,             T_U8,     1,     0,     255,    "",        "vID",      0,          "", "", "", "")
S(rx_channel,            rx_channel,            T_U8,     1,     1,     5,      "",        "rxCh",     0,          "", "", "", "")
S(drive_output,          drive_output,          T_CHOICE, 0,     NOLIM, NOLIM,  "auto|servo|can", "drvOut", 0,  "", "", "", "")
S(steer_output,          steer_output,          T_U8,     1,     1,     4,      "",        "strOut",   0,          "", "", "", "")
S(throttle_output,       throttle_output,       T_U8,     2,     1,     4,      "",        "thrOut",   0,          "", "", "", "")
S(CRUISE_THROTTLE,       CRUISE_THROTTLE,       T_I32,    1650,  1000,  2000,   "",        "crsThr",   0,          "", "", "", "")
S(CREEP_THROTTLE,        CREEP_THROTTLE,        T_I32,    1620,  1000,  2000,   "",        "crpThr",   0,          "", "", "", "")
S(steer_center_us,       PWM_OUT_1_DEFAULT,     T_I32,    1500,  800,   2200,   "",        "p1Def",    0,          "", "", "", "")
S(steer_min_us,          PWM_OUT_1_MIN,         T_I32,    1000,  800,   2200,   "",        "p1Min",    0,          "", "", "", "")
S(steer_max_us,          PWM_OUT_1_MAX,         T_I32,    2000,  800,   2200,   "",        "p1Max",    0,          "", "", "", "")
S(throttle_center_us,    PWM_OUT_2_DEFAULT,     T_I32,    1500,  800,   2200,   "",        "p2Def",    0,          "", "", "", "")
S(throttle_min_us,       PWM_OUT_2_MIN,         T_I32,    1000,  800,   2200,   "",        "p2Min",    0,          "", "", "", "")
S(throttle_max_us,       PWM_OUT_2_MAX,         T_I32,    2000,  800,   2200,   "",        "p2Max",    0,          "", "", "", "")
S(steer_lpf_hz,          steer_lpf_hz,          T_F64,    5.0,   0.5,   50,     "",        "lpfHz",    F_ZERO_OFF, "", "", "", "")
S(kp,                    kp,                    T_F64,    3.0,   0,     NOLIM,  "",        "kp",       0,          "", "", "", "")
S(ki,                    ki,                    T_F64,    0.0,   0,     NOLIM,  "",        "ki",       0,          "", "", "", "")
S(kd,                    kd,                    T_F64,    0.0,   0,     NOLIM,  "",        "kd",       0,          "", "", "", "")
S(pid_direction,         pid_direction,         T_IBOOL,  0,     0,     1,      "",        "pidDir",   0,          "", "", "", "")
S(steering_rate_kp,      steering_rate_kp,      T_F64,    0.02,  0,     NOLIM,  "",        "rateKp",   0,          "", "", "", "")
S(steering_rate_ki,      steering_rate_ki,      T_F64,    0.01,  0,     NOLIM,  "",        "rateKi",   0,          "", "", "", "")
S(steering_rate_kd,      steering_rate_kd,      T_F64,    0.0,   0,     NOLIM,  "",        "rateKd",   0,          "", "", "", "")
S(steering_rate_pid_direction, steering_rate_pid_direction, T_IBOOL, 0, 0, 1,    "",        "rateDir",  0,          "", "", "", "")
S(steering_rate_max_dps, steering_rate_max_dps, T_F64,    90.0,  0.001, 500,    "",        "rateMax",  0,          "", "", "", "")
S(enable_sensor_2,       enable_sensor_2,       T_BOOL,   0,     NOLIM, NOLIM,  "",        "enS2",     0,          "", "", "", "")
S(stuck_threshold_mm,    stuck_threshold_mm,    T_F32,    1.0,   1,     NOLIM,  "",        "stThresh", 0,          "", "", "", "")
S(stuck_timeout_ms,      stuck_timeout_ms,      T_F32,    20000, 1000,  NOLIM,  "",        "stTout",   0,          "", "", "", "")
S(BTxFollowChannel,      BTxFollowChannel,      T_U8,     0,     0,     2,      "",        "btxCh",    0,          "", "", "", "")
S(BTxThreshold,          BTxThreshold,          T_I32,    170,   11,    2000,   "",        "btxThr",   0,          "", "", "", "")
S(BTxAccelRate,          BTxAccelRate,          T_F64,    0.5,   0.3,   NOLIM,  "",        "btxAcc",   0,          "", "", "", "")
S(BTxCreepDuration,      BTxCreepDuration,      T_I32,    4,     1,     NOLIM,  "",        "btxDur",   0,          "", "", "", "")
S(SineSwerveAmplitude,   SineSwerveAmplitude,   T_U8,     0,     0,     255,    "",        "sineAmp",  0,          "", "", "", "")
S(SineSwerveFrequency,   SineSwerveFrequency,   T_U8,     15,    1,     255,    "",        "sineFrq",  0,          "", "", "", "")
S(CHARGE_VBATT_STOP,     CHARGE_VBATT_STOP,     T_F64,    11.9,  6,     16,     "",        "chgVStp",  0,          "", "", "", "")
S(CHARGE_MAX_CURRENT,    CHARGE_MAX_CURRENT,    T_F64,    6.0,   0.1,   6,      "",        "chgIMax",  0,          "", "", "", "")
S(CHARGE_MAX_SECONDS,    CHARGE_MAX_SECONDS,    T_I32,    180,   10,    3600,   "",        "chgTSec",  0,          "", "", "", "")
S(battery_cells,         battery_cells,         T_U8,     2,     1,     4,      "",        "batCells", 0,          "", "", "", "")
S(espnow_enabled,        espnow_enabled,        T_BOOL,   0,     NOLIM, NOLIM,  "",        "espNow",   0,          "", "", "", "")
S(telemetry_enabled,     telemetry_enabled,     T_BOOL,   1,     NOLIM, NOLIM,  "",        "telEn",    0,          "", "", "", "")
S(telemetry_rate_hz,     telemetry_rate_hz,     T_I32,    20,    1,     100,    "",        "telHz",    0,          "", "", "", "")
S(telemetry_bins_hz,     telemetry_bins_hz,     T_I32,    5,     0,     100,    "",        "binHz",    0,          "", "", "", "")
S(rover_enabled,         rover_enabled,         T_BOOL,   0,     NOLIM, NOLIM,  "",        "rvEn",     0,          "", "", "", "")
S(rover_goal_cw,         rover_goal_cw,         T_BOOL,   1,     NOLIM, NOLIM,  "",        "rvCW",     0,          "", "", "", "")
S(rover_sensor_height_mm, rover_sensor_height_mm, T_F64,  150.0, 30,    500,    "",        "rvH",      0,          "", "", "", "")
S(rover_full_speed_mps,  rover_full_speed_mps,  T_F64,    1.0,   0.1,   5,      "",        "rvVmax",   0,          "", "", "", "")
S(rover_full_yaw_rate_dps, rover_full_yaw_rate_dps, T_F64, 115.0, 10,   720,    "",        "rvWmax",   0,          "", "", "", "")
S(rover_seek_vbatt,      rover_seek_vbatt,      T_F64,    0.0,   6,     26,     "",        "rvSeekV",  F_ZERO_OFF, "", "", "", "")
S(rover_seek_after_s,    rover_seek_after_s,    T_I32,    600,   0,     NOLIM,  "",        "rvSeekS",  0,          "", "", "", "")
S(rover_entry_btx,       rover_entry_btx,       T_U8,     1,     0,     2,      "",        "rvEntBtx", 0,          "", "", "", "")
S(rover_exit_btx,        rover_exit_btx,        T_U8,     2,     0,     2,      "",        "rvExtBtx", 0,          "", "", "", "")
S(rover_lost_timeout_s,  rover_lost_timeout_s,  T_I32,    30,    0,     NOLIM,  "",        "rvLostS",  0,          "", "", "", "")
S(rover_speed_limit_mps, rover_speed_limit_mps, T_F64,    0.5,   0,     5,      "",        "rvVlim",   0,          "", "", "", "")
S(espnow_tx_power_dbm,   espnow_tx_power_dbm,   T_F64,    19.5,  2,     19.5,   "",        "txDbm",    0,          "", "", "", "")
S(cmd_key,               cmd_key,               T_LONG,   0,     NOLIM, NOLIM,  "",        "cmdKey",   0,          "", "", "", "")
S(imu_required,          imu_required,          T_BOOL,   1,     NOLIM, NOLIM,  "",        "imuReq",   0,          "", "", "", "")
S(dsp_frame_period_us,   dsp_frame_period_us,   T_I32,    10000, DSP_FRAME_PERIOD_MIN_US, 655350, "", "dspPer", 0,    "", "", "", "")
S(dsp_sensors,           dsp_sensors,           T_I32,    2,     1,     2,      "",        "dspSens",  0,          "", "", "", "")
S(dsp_bin_hz_0,          dsp_bin_hz[0],         T_F64,    53333.3, 42500, 120000, "",      "dspBin0",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_1,          dsp_bin_hz[1],         T_F64,    55315.5, 42500, 120000, "",      "dspBin1",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_2,          dsp_bin_hz[2],         T_F64,    51323.2, 42500, 120000, "",      "dspBin2",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_3,          dsp_bin_hz[3],         T_F64,    57347.7, 42500, 120000, "",      "dspBin3",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_4,          dsp_bin_hz[4],         T_F64,    49344.6, 42500, 120000, "",      "dspBin4",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_5,          dsp_bin_hz[5],         T_F64,    62500.0, 42500, 120000, "",      "dspBin5",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_6,          dsp_bin_hz[6],         T_F64,    46875.0, 42500, 120000, "",      "dspBin6",  F_ZERO_OFF, "", "", "", "")
S(dsp_bin_hz_7,          dsp_bin_hz[7],         T_F64,    0.0,     42500, 120000, "",      "dspBin7",  F_ZERO_OFF, "", "", "", "")
S(dsp_xy_mask,           dsp_xy_mask,           T_I32,    0,     0,     255,    "",        "dspXy",    0,          "", "", "", "")
S(dsp_slow_adc,          dsp_slow_adc,          T_I32,    1,     0,     1,      "",        "dspSlow",  0,          "", "", "", "")
S(dsp_pga,               dsp_pga,               T_I32,    -1,    -1,    2,      "",        "dspPga",   0,          "", "", "", "")
S(dsp_lanes,             dsp_lanes,             T_I32,    2,     0,     3,      "",        "dspLanes", 0,          "", "", "", "")
S(dsp_sweep_bins,        dsp_sweep_bins,        T_CHOICE, 255,   NOLIM, NOLIM,  "off|1|2|3|4|5|6|7|8|auto=255", "dspSweep", F_CHOICE_INT, "", "", "", "")
SF(dt_channels,          uint8_t,               T_U8,     255,   0,     255,    "",        "dtCh",     F_LIVE,     "", "Polarity detection", "Decoded Tx channels", "255 = auto: every Tx channel with a position bin within 30 Hz of its carrier. Else a mask: 1 = CH1, 2 = CH2, 4 = CH3, 8 = CH4, 16 = CH5 (add them; 31 = all five); 0 = polarity detection off. Rx STM32 2.1.134+")
SF(dt_side_a_ch1,        uint8_t,               T_U8,     0,     0,     2,      "",        "dtSideA1",  F_LIVE,     "", "Polarity detection", "Side A of Tx channel 1", "0 = not set, 1 = inside the perimeter, 2 = outside")
SF(dt_side_a_ch2,        uint8_t,               T_U8,     0,     0,     2,      "",        "dtSideA2",  F_LIVE,     "", "Polarity detection", "Side A of Tx channel 2", "0 = not set, 1 = inside the perimeter, 2 = outside")
SF(dt_side_a_ch3,        uint8_t,               T_U8,     0,     0,     2,      "",        "dtSideA3",  F_LIVE,     "", "Polarity detection", "Side A of Tx channel 3", "0 = not set, 1 = inside the perimeter, 2 = outside")
SF(dt_side_a_ch4,        uint8_t,               T_U8,     0,     0,     2,      "",        "dtSideA4",  F_LIVE,     "", "Polarity detection", "Side A of Tx channel 4", "0 = not set, 1 = inside the perimeter, 2 = outside")
SF(dt_side_a_ch5,        uint8_t,               T_U8,     0,     0,     2,      "",        "dtSideA5",  F_LIVE,     "", "Polarity detection", "Side A of Tx channel 5", "0 = not set, 1 = inside the perimeter, 2 = outside")
S(usb_drive,             usb_drive,             T_BOOL,   1,     NOLIM, NOLIM,  "",        "usbDrive", 0,          "", "", "", "")
S(radio_mode,            radio_mode,            T_CHOICE, 0,     NOLIM, NOLIM,  "espnow|wifi", "radioMode", 0,       "", "", "", "")
// Outputs 3 and 4: an older config.json and NVS only (no list, get or set; an output with no function holds 1500 us)
S(PWM_OUT_3_DEFAULT,     PWM_OUT_3_DEFAULT,     T_I32,    1500,  800,   2200,   "",        "p3Def",    F_LEGACY,   "", "", "", "")
S(PWM_OUT_3_MIN,         PWM_OUT_3_MIN,         T_I32,    1000,  800,   2200,   "",        "p3Min",    F_LEGACY,   "", "", "", "")
S(PWM_OUT_3_MAX,         PWM_OUT_3_MAX,         T_I32,    2000,  800,   2200,   "",        "p3Max",    F_LEGACY,   "", "", "", "")
S(PWM_OUT_4_DEFAULT,     PWM_OUT_4_DEFAULT,     T_I32,    1500,  800,   2200,   "",        "p4Def",    F_LEGACY,   "", "", "", "")
S(PWM_OUT_4_MIN,         PWM_OUT_4_MIN,         T_I32,    1000,  800,   2200,   "",        "p4Min",    F_LEGACY,   "", "", "", "")
S(PWM_OUT_4_MAX,         PWM_OUT_4_MAX,         T_I32,    2000,  800,   2200,   "",        "p4Max",    F_LEGACY,   "", "", "", "")
#ifdef SETTINGS_NEWKEY
SF(test_new_setting,     int,                   T_I32,    7,     0,     100,    "",        "testNew",  F_LIVE,     "", "Vehicle", "Test setting", "Test of one new table line: no effect on the vehicle")
#endif
