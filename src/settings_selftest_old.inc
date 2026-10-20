// Self-check build only (SETTINGS_SELFTEST), included at the end of src/usb_msc_fat.cpp.
// Part 1: the settings code of 0.5.111 (ecd5bc6), cut out by a script and renamed old_*. NVS goes through a
// parameter (RAM store) in place of Preferences; nothing else is changed.

static void old_populate_defaults(DeviceConfig& c,
                               const DeviceConfig* seed = nullptr)
{
    // If a seed is supplied (from NVS) use it; otherwise use compiled defaults.
    if (seed) {
        c = *seed;
        return;
    }
    strcpy(c.vehicle_name, "Vehicle");
    c.vehicleID             = 1;
    c.rx_channel            = 1;
    c.drive_output          = 0;   // auto
    c.steer_output          = 1;
    c.throttle_output       = 2;
    c.steer_lpf_hz          = 5.0;
    c.CRUISE_THROTTLE       = 1650;
    c.CREEP_THROTTLE        = 1620;
    c.PWM_OUT_1_DEFAULT     = 1500;
    c.PWM_OUT_1_MIN         = 1000;
    c.PWM_OUT_1_MAX         = 2000;
    c.PWM_OUT_2_DEFAULT     = 1500;
    c.PWM_OUT_2_MIN         = 1000;
    c.PWM_OUT_2_MAX         = 2000;
    c.PWM_OUT_3_DEFAULT     = 1500;
    c.PWM_OUT_3_MIN         = 1000;
    c.PWM_OUT_3_MAX         = 2000;
    c.PWM_OUT_4_DEFAULT     = 1500;
    c.PWM_OUT_4_MIN         = 1000;
    c.PWM_OUT_4_MAX         = 2000;
    c.kp                    = 3.0;
    c.ki                    = 0.0;
    c.kd                    = 0.0;
    c.pid_direction         = 0;   
    c.steering_rate_kp = 0.02;
    c.steering_rate_ki = 0.01;
    c.steering_rate_kd = 0.0;
    c.steering_rate_pid_direction = 0;
    c.steering_rate_max_dps = 90.0;
    c.enable_sensor_2       = false;
    c.stuck_threshold_mm    = 0.1f;
    c.stuck_timeout_ms      = 20000.0f;
    c.BTxFollowChannel      = 0;
    c.BTxThreshold          = 170;
    c.BTxAccelRate          = 0.5;
    c.BTxCreepDuration      = 4;
    c.SineSwerveAmplitude   = 0;
    c.SineSwerveFrequency   = 15;
    c.CHARGE_VBATT_STOP     = 11.9;
    c.CHARGE_MAX_CURRENT    = 6.0;
    c.CHARGE_MAX_SECONDS    = 180;
    c.battery_cells         = 2;
    c.espnow_enabled        = false;
    c.telemetry_enabled     = true;
    c.telemetry_rate_hz     = 20;
    c.telemetry_bins_hz     = 5;
    c.rover_enabled            = false;
    c.rover_goal_cw            = true;
    c.rover_sensor_height_mm   = 150.0;
    c.rover_full_speed_mps     = 1.0;
    c.rover_full_yaw_rate_dps  = 115.0;
    c.rover_seek_vbatt         = 0.0;
    c.rover_seek_after_s       = 600;
    c.rover_entry_btx          = 1;
    c.rover_exit_btx           = 2;
    c.rover_lost_timeout_s     = 30;
    c.rover_speed_limit_mps    = 0.5;
    c.espnow_tx_power_dbm      = 19.5;
    c.cmd_key                  = 0;
    c.imu_required             = true;
    // DSP: the STM32 v2 app defaults (bins 0-4 = Tx CH 1-5, then 62.5 kHz, 46.875 kHz, off)
    static const double BIN_HZ[8] = {53333.3, 55315.5, 51323.2, 57347.7, 49344.6, 62500.0, 46875.0, 0.0};
    c.dsp_frame_period_us      = 10000;
    c.dsp_sensors              = 2;
    for (int b = 0; b < 8; b++) c.dsp_bin_hz[b] = BIN_HZ[b];
    c.dsp_xy_mask              = 0;
    c.dsp_slow_adc             = 1;
    c.dsp_pga                  = -1;
    c.dsp_lanes                = 2;
    c.dsp_sweep_bins           = 255;  // auto: all free bins (the Rx STM32 default)
    c.usb_drive                = true;
    c.radio_mode               = RADIO_MODE_ESPNOW;
}

static void old_validate_config(DeviceConfig& c)
{
    // Channel
    if (c.rx_channel < 1 || c.rx_channel > 5) c.rx_channel = 1;
    c.telemetry_rate_hz = rec::rateNearest(c.telemetry_rate_hz);  // 100/N Hz only
    c.telemetry_bins_hz = c.telemetry_bins_hz <= 0 ? 0 : rec::rateNearest(c.telemetry_bins_hz);  // 0 = off
    if (c.battery_cells < 1) c.battery_cells = 2;
    if (c.battery_cells > 4) c.battery_cells = 4;  // 4S at most
    // Vehicle name: 16 characters at most from A-Z a-z 0-9 space _ -; others removed; empty: "Vehicle"
    {
        char clean[17]; int n = 0;
        for (int i = 0; c.vehicle_name[i] && i < 16; i++) {
            const char ch = c.vehicle_name[i];
            if (isalnum((unsigned char)ch) || ch == ' ' || ch == '_' || ch == '-') clean[n++] = ch;
        }
        clean[n] = 0;
        strcpy(c.vehicle_name, n ? clean : "Vehicle");
    }
    if (c.drive_output > 2) c.drive_output = 0;
    if (c.steer_output < 1 || c.steer_output > 4) c.steer_output = 1;
    if (c.throttle_output < 1 || c.throttle_output > 4) c.throttle_output = 2;
    if (c.throttle_output == c.steer_output) c.throttle_output = c.steer_output == 2 ? 1 : 2;
    if (!isfinite(c.steer_lpf_hz) || c.steer_lpf_hz < 0.0) c.steer_lpf_hz = 0.0;
    if (c.steer_lpf_hz > 0.0 && c.steer_lpf_hz < 0.5) c.steer_lpf_hz = 0.5;
    if (c.steer_lpf_hz > 50.0) c.steer_lpf_hz = 50.0;

    // Throttle range sanity
    c.CRUISE_THROTTLE   = constrain(c.CRUISE_THROTTLE, 1000, 2000);
    c.CREEP_THROTTLE    = constrain(c.CREEP_THROTTLE,  1000, 2000);

    // PWM output limits – must satisfy MIN > PWM_ABS_MIN, MAX < PWM_ABS_MAX
    // and DEFAULT must be within [MIN, MAX].
    auto fix_out = [](int& d, int& lo, int& hi) {
        lo = constrain(lo, PWM_ABS_MIN, PWM_ABS_MAX);
        hi = constrain(hi, PWM_ABS_MIN, PWM_ABS_MAX);
        if (lo > hi) {
            lo = constrain(hi - 100, PWM_ABS_MIN, PWM_ABS_MAX);
            hi = constrain(lo + 100, PWM_ABS_MIN, PWM_ABS_MAX);
        }
        d  = constrain(d, lo, hi);
    };
    fix_out(c.PWM_OUT_1_DEFAULT, c.PWM_OUT_1_MIN, c.PWM_OUT_1_MAX);
    fix_out(c.PWM_OUT_2_DEFAULT, c.PWM_OUT_2_MIN, c.PWM_OUT_2_MAX);
    fix_out(c.PWM_OUT_3_DEFAULT, c.PWM_OUT_3_MIN, c.PWM_OUT_3_MAX);
    fix_out(c.PWM_OUT_4_DEFAULT, c.PWM_OUT_4_MIN, c.PWM_OUT_4_MAX);

    // PID
    if (c.kp < 0.0)  c.kp = 0.0;
    if (c.ki < 0.0)  c.ki = 0.0;
    if (c.kd < 0.0)  c.kd = 0.0;
    if (c.pid_direction < 0 || c.pid_direction > 1) c.pid_direction = 0;

    if (!isfinite(c.steering_rate_kp) || c.steering_rate_kp < 0.0) c.steering_rate_kp = 0.02;
    if (!isfinite(c.steering_rate_ki) || c.steering_rate_ki < 0.0) c.steering_rate_ki = 0.01;
    if (!isfinite(c.steering_rate_kd) || c.steering_rate_kd < 0.0) c.steering_rate_kd = 0.0;
    if (c.steering_rate_pid_direction < 0 || c.steering_rate_pid_direction > 1) c.steering_rate_pid_direction = 0;
    if (!isfinite(c.steering_rate_max_dps) || c.steering_rate_max_dps <= 0.0 || c.steering_rate_max_dps > 500.0) c.steering_rate_max_dps = 90.0;

    // Stuck
    if (c.stuck_threshold_mm < 1.0f)   c.stuck_threshold_mm = 1.0f;
    if (c.stuck_timeout_ms   < 1000.0f) c.stuck_timeout_ms  = 1000.0f;

    // BTx
    if (c.BTxFollowChannel > 2)   c.BTxFollowChannel = 0;
    c.BTxThreshold = constrain(c.BTxThreshold, 11, 2000);
    if (c.BTxAccelRate     < 0.3) c.BTxAccelRate     = 0.3;
    if (c.BTxCreepDuration < 1)   c.BTxCreepDuration = 1;

    // Sine
    if (c.SineSwerveFrequency < 1) c.SineSwerveFrequency = 1;

    // Charging: the STM32 application limits (TrailTracer-Rx-STM32 rctf_wfrx_dsp.h): stop voltage
    // 16.0 V, current 6.0 A, charge time 3600 s (command 8 is in seconds in app 0.1.0 and 2.x). From
    // 0.5.69 CHARGE_MAX_SECONDS is the charge time and goes as it is; up to 0.5.68 the ESP32 sent 2 x it.
    if (c.CHARGE_VBATT_STOP  < 6.0)  c.CHARGE_VBATT_STOP  = 6.0;
    if (c.CHARGE_VBATT_STOP  > 16.0) c.CHARGE_VBATT_STOP  = 16.0;
    if (c.CHARGE_MAX_CURRENT < 0.1)  c.CHARGE_MAX_CURRENT = 0.1;
    if (c.CHARGE_MAX_CURRENT > 6.0) c.CHARGE_MAX_CURRENT = 6.0;
    if (c.CHARGE_MAX_SECONDS < 10)   c.CHARGE_MAX_SECONDS = 10;
    if (c.CHARGE_MAX_SECONDS > 3600) c.CHARGE_MAX_SECONDS = 3600;

    // Rover autopilot
    c.rover_sensor_height_mm  = constrain(c.rover_sensor_height_mm, 30.0, 500.0);
    c.rover_full_speed_mps    = constrain(c.rover_full_speed_mps, 0.1, 5.0);
    c.rover_full_yaw_rate_dps = constrain(c.rover_full_yaw_rate_dps, 10.0, 720.0);
    if (c.rover_seek_vbatt != 0.0) c.rover_seek_vbatt = constrain(c.rover_seek_vbatt, 6.0, 26.0);
    if (c.rover_seek_after_s < 0)   c.rover_seek_after_s = 0;
    if (c.rover_entry_btx > 2)      c.rover_entry_btx = 0;
    if (c.rover_exit_btx > 2)       c.rover_exit_btx = 0;
    if (c.rover_lost_timeout_s < 0) c.rover_lost_timeout_s = 0;
    c.rover_speed_limit_mps   = constrain(c.rover_speed_limit_mps, 0.0, 5.0);
    c.espnow_tx_power_dbm     = round(constrain(c.espnow_tx_power_dbm, 2.0, 19.5) * 4.0) / 4.0;

    // DSP: the widest ranges of the STM32 v2 apps; the STM32 clamps to its own and answers the value in effect
    c.dsp_frame_period_us = constrain(c.dsp_frame_period_us, DSP_FRAME_PERIOD_MIN_US, 655350);
    c.dsp_sensors         = constrain(c.dsp_sensors, 1, 2);
    for (int b = 0; b < 8; b++)
        if (!isfinite(c.dsp_bin_hz[b]) || c.dsp_bin_hz[b] < 0.0) c.dsp_bin_hz[b] = 0.0;
        else if (c.dsp_bin_hz[b] != 0.0) c.dsp_bin_hz[b] = constrain(c.dsp_bin_hz[b], 42500.0, 120000.0);
    c.dsp_xy_mask         = constrain(c.dsp_xy_mask, 0, 255);
    c.dsp_slow_adc        = constrain(c.dsp_slow_adc, 0, 1);
    c.dsp_pga             = constrain(c.dsp_pga, -1, 2);
    c.dsp_lanes           = constrain(c.dsp_lanes, 0, 3);
    if (c.dsp_sweep_bins > 8 && c.dsp_sweep_bins != 255) c.dsp_sweep_bins = 0;
    if (c.radio_mode > RADIO_MODE_WIFI) c.radio_mode = RADIO_MODE_ESPNOW;
}

template <class P>
static void old_prefs_save(P& p, const DeviceConfig& c)
{
    p.putUChar("vID",      c.vehicleID);
    p.putString("vName",   c.vehicle_name);
    p.putUChar("drvOut",   c.drive_output);
    p.putUChar("strOut",   c.steer_output);
    p.putUChar("thrOut",   c.throttle_output);
    p.putDouble("lpfHz",   c.steer_lpf_hz);
    p.putBool("chgReal",   true);  // CHARGE_MAX_SECONDS holds the real time (0.5.69)
    p.putUChar("rxCh",     c.rx_channel);
    p.putInt("crsThr",     c.CRUISE_THROTTLE);
    p.putInt("crpThr",     c.CREEP_THROTTLE);
    p.putInt("p1Def",      c.PWM_OUT_1_DEFAULT);
    p.putInt("p1Min",      c.PWM_OUT_1_MIN);
    p.putInt("p1Max",      c.PWM_OUT_1_MAX);
    p.putInt("p2Def",      c.PWM_OUT_2_DEFAULT);
    p.putInt("p2Min",      c.PWM_OUT_2_MIN);
    p.putInt("p2Max",      c.PWM_OUT_2_MAX);
    p.putInt("p3Def",      c.PWM_OUT_3_DEFAULT);
    p.putInt("p3Min",      c.PWM_OUT_3_MIN);
    p.putInt("p3Max",      c.PWM_OUT_3_MAX);
    p.putInt("p4Def",      c.PWM_OUT_4_DEFAULT);
    p.putInt("p4Min",      c.PWM_OUT_4_MIN);
    p.putInt("p4Max",      c.PWM_OUT_4_MAX);
    p.putDouble("kp",      c.kp);
    p.putDouble("ki",      c.ki);
    p.putDouble("kd",      c.kd);
    p.putInt("pidDir",     c.pid_direction);
    p.putDouble("rateKp", c.steering_rate_kp);
    p.putDouble("rateKi", c.steering_rate_ki);
    p.putDouble("rateKd", c.steering_rate_kd);
    p.putInt("rateDir", c.steering_rate_pid_direction);
    p.putDouble("rateMax", c.steering_rate_max_dps);
    p.putBool("enS2",      c.enable_sensor_2);
    p.putFloat("stThresh", c.stuck_threshold_mm);
    p.putFloat("stTout",   c.stuck_timeout_ms);
    p.putUChar("btxCh",    c.BTxFollowChannel);
    p.putInt("btxThr",     c.BTxThreshold);
    p.putDouble("btxAcc",  c.BTxAccelRate);
    p.putInt("btxDur",     c.BTxCreepDuration);
    p.putUChar("sineAmp",  c.SineSwerveAmplitude);
    p.putUChar("sineFrq",  c.SineSwerveFrequency);
    p.putDouble("chgVStp", c.CHARGE_VBATT_STOP);
    p.putDouble("chgIMax", c.CHARGE_MAX_CURRENT);
    p.putInt("chgTSec",    c.CHARGE_MAX_SECONDS);
    p.putUChar("batCells", c.battery_cells);
    p.putBool("espNow",    c.espnow_enabled);
    p.putBool("telEn",     c.telemetry_enabled);
    p.putInt("telHz",      c.telemetry_rate_hz);
    p.putInt("binHz",      c.telemetry_bins_hz);
    p.putBool("rvEn",      c.rover_enabled);
    p.putBool("rvCW",      c.rover_goal_cw);
    p.putDouble("rvH",     c.rover_sensor_height_mm);
    p.putDouble("rvVmax",  c.rover_full_speed_mps);
    p.putDouble("rvWmax",  c.rover_full_yaw_rate_dps);
    p.putDouble("rvSeekV", c.rover_seek_vbatt);
    p.putInt("rvSeekS",    c.rover_seek_after_s);
    p.putUChar("rvEntBtx", c.rover_entry_btx);
    p.putUChar("rvExtBtx", c.rover_exit_btx);
    p.putInt("rvLostS",    c.rover_lost_timeout_s);
    p.putDouble("rvVlim",  c.rover_speed_limit_mps);
    p.putDouble("txDbm",   c.espnow_tx_power_dbm);
    p.putLong("cmdKey",    c.cmd_key);
    p.putBool("imuReq",    c.imu_required);
    p.putInt("dspPer",     c.dsp_frame_period_us);
    p.putInt("dspSens",    c.dsp_sensors);
    for (int b = 0; b < 8; b++) { char k[8]; snprintf(k, sizeof(k), "dspBin%d", b); p.putDouble(k, c.dsp_bin_hz[b]); }
    p.putInt("dspXy",      c.dsp_xy_mask);
    p.putInt("dspSlow",    c.dsp_slow_adc);
    p.putInt("dspPga",     c.dsp_pga);
    p.putInt("dspLanes",   c.dsp_lanes);
    p.putUChar("dspSweep", c.dsp_sweep_bins);
    p.putBool("usbDrive",  c.usb_drive);
    p.putUChar("radioMode", c.radio_mode);
}

template <class P>
static bool old_prefs_load(P& p, DeviceConfig& c)
{
    if (!p.isKey("vID")) { return false; } // namespace is empty

    c.vehicleID             = p.getUChar("vID",      c.vehicleID);
    {
        const String name = p.getString("vName", c.vehicle_name);
        strncpy(c.vehicle_name, name.c_str(), 16); c.vehicle_name[16] = 0;
    }
    // A board updated from 0.5.68 or older drove its motor controllers over CAN: keep that, not auto.
    c.drive_output          = p.isKey("drvOut") ? p.getUChar("drvOut", c.drive_output) : 2;
    c.steer_output          = p.getUChar("strOut",   c.steer_output);
    c.throttle_output       = p.getUChar("thrOut",   c.throttle_output);
    c.steer_lpf_hz          = p.getDouble("lpfHz",   c.steer_lpf_hz);
    c.rx_channel            = p.getUChar("rxCh",     c.rx_channel);
    c.CRUISE_THROTTLE       = p.getInt("crsThr",     c.CRUISE_THROTTLE);
    c.CREEP_THROTTLE        = p.getInt("crpThr",     c.CREEP_THROTTLE);
    c.PWM_OUT_1_DEFAULT     = p.getInt("p1Def",      c.PWM_OUT_1_DEFAULT);
    c.PWM_OUT_1_MIN         = p.getInt("p1Min",      c.PWM_OUT_1_MIN);
    c.PWM_OUT_1_MAX         = p.getInt("p1Max",      c.PWM_OUT_1_MAX);
    c.PWM_OUT_2_DEFAULT     = p.getInt("p2Def",      c.PWM_OUT_2_DEFAULT);
    c.PWM_OUT_2_MIN         = p.getInt("p2Min",      c.PWM_OUT_2_MIN);
    c.PWM_OUT_2_MAX         = p.getInt("p2Max",      c.PWM_OUT_2_MAX);
    c.PWM_OUT_3_DEFAULT     = p.getInt("p3Def",      c.PWM_OUT_3_DEFAULT);
    c.PWM_OUT_3_MIN         = p.getInt("p3Min",      c.PWM_OUT_3_MIN);
    c.PWM_OUT_3_MAX         = p.getInt("p3Max",      c.PWM_OUT_3_MAX);
    c.PWM_OUT_4_DEFAULT     = p.getInt("p4Def",      c.PWM_OUT_4_DEFAULT);
    c.PWM_OUT_4_MIN         = p.getInt("p4Min",      c.PWM_OUT_4_MIN);
    c.PWM_OUT_4_MAX         = p.getInt("p4Max",      c.PWM_OUT_4_MAX);
    c.kp                    = p.getDouble("kp",      c.kp);
    c.ki                    = p.getDouble("ki",      c.ki);
    c.kd                    = p.getDouble("kd",      c.kd);
    c.pid_direction         = p.getInt("pidDir",     c.pid_direction);
    c.steering_rate_kp = p.getDouble("rateKp", c.steering_rate_kp);
    c.steering_rate_ki = p.getDouble("rateKi", c.steering_rate_ki);
    c.steering_rate_kd = p.getDouble("rateKd", c.steering_rate_kd);
    c.steering_rate_pid_direction = p.getInt("rateDir", c.steering_rate_pid_direction);
    c.steering_rate_max_dps = p.getDouble("rateMax", c.steering_rate_max_dps);
    c.enable_sensor_2       = p.getBool("enS2",      c.enable_sensor_2);
    c.stuck_threshold_mm    = p.getFloat("stThresh", c.stuck_threshold_mm);
    c.stuck_timeout_ms      = p.getFloat("stTout",   c.stuck_timeout_ms);
    c.BTxFollowChannel      = p.getUChar("btxCh",    c.BTxFollowChannel);
    c.BTxThreshold          = p.getInt("btxThr", c.BTxThreshold);
    c.BTxAccelRate          = p.getDouble("btxAcc",  c.BTxAccelRate);
    c.BTxCreepDuration      = p.getInt("btxDur",     c.BTxCreepDuration);
    c.SineSwerveAmplitude   = p.getUChar("sineAmp",  c.SineSwerveAmplitude);
    c.SineSwerveFrequency   = p.getUChar("sineFrq",  c.SineSwerveFrequency);
    c.CHARGE_VBATT_STOP     = p.getDouble("chgVStp", c.CHARGE_VBATT_STOP);
    c.CHARGE_MAX_CURRENT    = p.getDouble("chgIMax", c.CHARGE_MAX_CURRENT);
    c.CHARGE_MAX_SECONDS    = p.getInt("chgTSec",    c.CHARGE_MAX_SECONDS);
    if (!p.getBool("chgReal", false)) c.CHARGE_MAX_SECONDS *= 2;  // up to 0.5.68 the value was half the charge time
    c.battery_cells         = p.getUChar("batCells", c.battery_cells);
    c.espnow_enabled        = p.getBool("espNow",    c.espnow_enabled);
    c.telemetry_enabled     = p.getBool("telEn",     c.telemetry_enabled);
    c.telemetry_rate_hz     = p.getInt("telHz",      c.telemetry_rate_hz);
    c.telemetry_bins_hz     = p.getInt("binHz",      c.telemetry_bins_hz);
    c.rover_enabled            = p.getBool("rvEn",      c.rover_enabled);
    c.rover_goal_cw            = p.getBool("rvCW",      c.rover_goal_cw);
    c.rover_sensor_height_mm   = p.getDouble("rvH",     c.rover_sensor_height_mm);
    c.rover_full_speed_mps     = p.getDouble("rvVmax",  c.rover_full_speed_mps);
    c.rover_full_yaw_rate_dps  = p.getDouble("rvWmax",  c.rover_full_yaw_rate_dps);
    c.rover_seek_vbatt         = p.getDouble("rvSeekV", c.rover_seek_vbatt);
    c.rover_seek_after_s       = p.getInt("rvSeekS",    c.rover_seek_after_s);
    c.rover_entry_btx          = p.getUChar("rvEntBtx", c.rover_entry_btx);
    c.rover_exit_btx           = p.getUChar("rvExtBtx", c.rover_exit_btx);
    c.rover_lost_timeout_s     = p.getInt("rvLostS",    c.rover_lost_timeout_s);
    c.rover_speed_limit_mps    = p.getDouble("rvVlim",  c.rover_speed_limit_mps);
    c.espnow_tx_power_dbm      = p.getDouble("txDbm",   c.espnow_tx_power_dbm);
    c.cmd_key                  = p.getLong("cmdKey",    c.cmd_key);
    c.imu_required             = p.getBool("imuReq",    c.imu_required);
    c.dsp_frame_period_us      = p.getInt("dspPer",     c.dsp_frame_period_us);
    c.dsp_sensors              = p.getInt("dspSens",    c.dsp_sensors);
    for (int b = 0; b < 8; b++) { char k[8]; snprintf(k, sizeof(k), "dspBin%d", b); c.dsp_bin_hz[b] = p.getDouble(k, c.dsp_bin_hz[b]); }
    c.dsp_xy_mask              = p.getInt("dspXy",      c.dsp_xy_mask);
    c.dsp_slow_adc             = p.getInt("dspSlow",    c.dsp_slow_adc);
    c.dsp_pga                  = p.getInt("dspPga",     c.dsp_pga);
    c.dsp_lanes                = p.getInt("dspLanes",   c.dsp_lanes);
    c.dsp_sweep_bins           = p.getUChar("dspSweep", c.dsp_sweep_bins);
    c.usb_drive                = p.getBool("usbDrive",  c.usb_drive);
    c.radio_mode               = p.getUChar("radioMode", c.radio_mode);
    return true;
}

static void old_config_to_doc(const DeviceConfig& c, JsonDocument& doc, bool with_eff = false)
{
    doc["vehicle_name"]          = c.vehicle_name;
    doc["vehicleID"]             = c.vehicleID;
    doc["rx_channel"]            = c.rx_channel;
    doc["drive_output"]          = c.drive_output == 1 ? "servo" : c.drive_output == 2 ? "can" : "auto";
    doc["steer_output"]          = c.steer_output;
    doc["throttle_output"]       = c.throttle_output;
    doc["CRUISE_THROTTLE"]       = c.CRUISE_THROTTLE;
    doc["CREEP_THROTTLE"]        = c.CREEP_THROTTLE;
    doc["steer_center_us"]       = c.PWM_OUT_1_DEFAULT;
    doc["steer_min_us"]          = c.PWM_OUT_1_MIN;
    doc["steer_max_us"]          = c.PWM_OUT_1_MAX;
    doc["throttle_center_us"]    = c.PWM_OUT_2_DEFAULT;
    doc["throttle_min_us"]       = c.PWM_OUT_2_MIN;
    doc["throttle_max_us"]       = c.PWM_OUT_2_MAX;
    doc["steer_lpf_hz"]          = c.steer_lpf_hz;
    doc["kp"]                    = c.kp;
    doc["ki"]                    = c.ki;
    doc["kd"]                    = c.kd;
    doc["pid_direction"]         = (bool)c.pid_direction;   // false=NORMAL, true=REVERSE
    doc["steering_rate_kp"] = c.steering_rate_kp;
    doc["steering_rate_ki"] = c.steering_rate_ki;
    doc["steering_rate_kd"] = c.steering_rate_kd;
    doc["steering_rate_pid_direction"] = (bool)c.steering_rate_pid_direction;
    doc["steering_rate_max_dps"] = c.steering_rate_max_dps;
    doc["enable_sensor_2"]       = c.enable_sensor_2;
    doc["stuck_threshold_mm"]    = c.stuck_threshold_mm;
    doc["stuck_timeout_ms"]      = c.stuck_timeout_ms;
    doc["BTxFollowChannel"]      = c.BTxFollowChannel;
    doc["BTxThreshold"]          = c.BTxThreshold;
    doc["BTxAccelRate"]          = c.BTxAccelRate;
    doc["BTxCreepDuration"]      = c.BTxCreepDuration;
    doc["SineSwerveAmplitude"]   = c.SineSwerveAmplitude;
    doc["SineSwerveFrequency"]   = c.SineSwerveFrequency;
    doc["CHARGE_VBATT_STOP"]     = c.CHARGE_VBATT_STOP;
    doc["CHARGE_MAX_CURRENT"]    = c.CHARGE_MAX_CURRENT;
    doc["CHARGE_MAX_SECONDS"]    = c.CHARGE_MAX_SECONDS;
    doc["battery_cells"]         = c.battery_cells;
    doc["espnow_enabled"]        = c.espnow_enabled;
    doc["telemetry_enabled"]     = c.telemetry_enabled;
    doc["telemetry_rate_hz"]     = c.telemetry_rate_hz;
    doc["telemetry_bins_hz"]     = c.telemetry_bins_hz;
    doc["rover_enabled"]            = c.rover_enabled;
    doc["rover_goal_cw"]            = c.rover_goal_cw;
    doc["rover_sensor_height_mm"]   = c.rover_sensor_height_mm;
    doc["rover_full_speed_mps"]     = c.rover_full_speed_mps;
    doc["rover_full_yaw_rate_dps"]  = c.rover_full_yaw_rate_dps;
    doc["rover_seek_vbatt"]         = c.rover_seek_vbatt;
    doc["rover_seek_after_s"]       = c.rover_seek_after_s;
    doc["rover_entry_btx"]          = c.rover_entry_btx;
    doc["rover_exit_btx"]           = c.rover_exit_btx;
    doc["rover_lost_timeout_s"]     = c.rover_lost_timeout_s;
    doc["rover_speed_limit_mps"]    = c.rover_speed_limit_mps;
    doc["espnow_tx_power_dbm"]      = c.espnow_tx_power_dbm;
    doc["cmd_key"]                  = c.cmd_key;
    doc["imu_required"]             = c.imu_required;
    doc["dsp_frame_period_us"]      = c.dsp_frame_period_us;
    doc["dsp_sensors"]              = c.dsp_sensors;
    for (int b = 0; b < 8; b++) doc[dsp_bin_key(b)] = c.dsp_bin_hz[b];
    doc["dsp_xy_mask"]              = c.dsp_xy_mask;
    doc["dsp_slow_adc"]             = c.dsp_slow_adc;
    doc["dsp_pga"]                  = c.dsp_pga;
    doc["dsp_lanes"]                = c.dsp_lanes;
    if (c.dsp_sweep_bins == 0) doc["dsp_sweep_bins"] = "off";
    else if (c.dsp_sweep_bins == 255) doc["dsp_sweep_bins"] = "auto";
    else doc["dsp_sweep_bins"] = String((int)c.dsp_sweep_bins);
    doc["usb_drive"]                = c.usb_drive;
    doc["radio_mode"]               = c.radio_mode == RADIO_MODE_WIFI ? "wifi" : "espnow";
    if (!with_eff) return;
    auto eff = [&](const String& key, int i, bool is_int) {
        if (isnan(g_dsp_eff[i])) doc[key] = nullptr;
        else if (is_int) doc[key] = (long)g_dsp_eff[i];
        else doc[key] = g_dsp_eff[i];
    };
    eff("dsp_frame_period_us_eff", DSP_EFF_PERIOD, true);
    eff("dsp_sensors_eff", DSP_EFF_SENSORS, true);
    for (int b = 0; b < 8; b++) eff(String(dsp_bin_key(b)) + "_eff", DSP_EFF_BIN0 + b, false);
    eff("dsp_xy_mask_eff", DSP_EFF_XY, true);
    eff("dsp_slow_adc_eff", DSP_EFF_SLOW, true);
    eff("dsp_pga_eff", DSP_EFF_PGA, true);
    eff("dsp_lanes_eff", DSP_EFF_LANES, true);
    // Wi-Fi network (NVS only, not in config.json): the SSID, and whether a password is stored. Never the password.
    doc["wifi_ssid"]                = msc_fat_wifi_ssid();
    doc["wifi_password"]            = msc_fat_wifi_has_password() ? "set" : "not set";
}

static bool old_config_from_json_text(DeviceConfig& c, const String& json_no_comments)
{
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json_no_comments);
    if (err) {
        ESP_LOGE(TAG, "JSON parse error: %s", err.c_str());
        return false;
    }

    // Helper macro: update only if key present and value is the expected type.
#define JSON_GET_INT(key, field)    if (doc[key].is<int>())    c.field = doc[key].as<int>()
#define JSON_GET_UINT8(key, field)  if (doc[key].is<int>())    c.field = (uint8_t)constrain(doc[key].as<int>(), 0, 255)
#define JSON_GET_DBL(key, field)    if (doc[key].is<double>()) c.field = doc[key].as<double>()
#define JSON_GET_FLT(key, field)    if (doc[key].is<double>()) c.field = (float)doc[key].as<double>()
#define JSON_GET_BOOL(key, field)   if (doc[key].is<bool>())   c.field = doc[key].as<bool>()

    if (doc["vehicle_name"].is<const char*>()) {
        strncpy(c.vehicle_name, doc["vehicle_name"].as<const char*>(), 16); c.vehicle_name[16] = 0;
    }
    if (doc["drive_output"].is<const char*>()) {
        const char* d = doc["drive_output"].as<const char*>();
        c.drive_output = !strcmp(d, "servo") ? 1 : !strcmp(d, "can") ? 2 : 0;
    }
    JSON_GET_UINT8("steer_output",          steer_output);
    JSON_GET_UINT8("throttle_output",       throttle_output);
    JSON_GET_DBL("steer_lpf_hz",            steer_lpf_hz);
    JSON_GET_UINT8("vehicleID",             vehicleID);
    JSON_GET_UINT8("rx_channel",            rx_channel);
    JSON_GET_INT("CRUISE_THROTTLE",         CRUISE_THROTTLE);
    JSON_GET_INT("CREEP_THROTTLE",          CREEP_THROTTLE);
    JSON_GET_INT("PWM_OUT_1_DEFAULT",       PWM_OUT_1_DEFAULT);
    JSON_GET_INT("PWM_OUT_1_MIN",           PWM_OUT_1_MIN);
    JSON_GET_INT("PWM_OUT_1_MAX",           PWM_OUT_1_MAX);
    JSON_GET_INT("PWM_OUT_2_DEFAULT",       PWM_OUT_2_DEFAULT);
    JSON_GET_INT("PWM_OUT_2_MIN",           PWM_OUT_2_MIN);
    JSON_GET_INT("PWM_OUT_2_MAX",           PWM_OUT_2_MAX);
    JSON_GET_INT("PWM_OUT_3_DEFAULT",       PWM_OUT_3_DEFAULT);
    JSON_GET_INT("PWM_OUT_3_MIN",           PWM_OUT_3_MIN);
    JSON_GET_INT("PWM_OUT_3_MAX",           PWM_OUT_3_MAX);
    JSON_GET_INT("PWM_OUT_4_DEFAULT",       PWM_OUT_4_DEFAULT);
    JSON_GET_INT("PWM_OUT_4_MIN",           PWM_OUT_4_MIN);
    JSON_GET_INT("PWM_OUT_4_MAX",           PWM_OUT_4_MAX);
    // 0.5.69 names by function; an older config.json gives PWM_OUT_1_* (steering) and PWM_OUT_2_* (throttle) above
    JSON_GET_INT("steer_center_us",         PWM_OUT_1_DEFAULT);
    JSON_GET_INT("steer_min_us",            PWM_OUT_1_MIN);
    JSON_GET_INT("steer_max_us",            PWM_OUT_1_MAX);
    JSON_GET_INT("throttle_center_us",      PWM_OUT_2_DEFAULT);
    JSON_GET_INT("throttle_min_us",         PWM_OUT_2_MIN);
    JSON_GET_INT("throttle_max_us",         PWM_OUT_2_MAX);
    // A config.json from 0.5.68 or older (PWM_OUT_1_DEFAULT, no steer_center_us): CHARGE_MAX_SECONDS was half the charge
    // time; the file is written again in the 0.5.69 form after the read (s_rate_config_missing).
    const bool legacy_file = doc["PWM_OUT_1_DEFAULT"].is<int>() && doc["steer_center_us"].isNull();
    s_rate_config_missing = doc["steering_rate_kp"].isNull() || doc["steering_rate_ki"].isNull() || doc["steering_rate_kd"].isNull() || doc["steering_rate_pid_direction"].isNull() || doc["steering_rate_max_dps"].isNull() || legacy_file;

    JSON_GET_DBL("kp",                      kp);
    JSON_GET_DBL("ki",                      ki);
    JSON_GET_DBL("kd",                      kd);
    JSON_GET_BOOL("pid_direction",          pid_direction);
    JSON_GET_DBL("steering_rate_kp", steering_rate_kp);
    JSON_GET_DBL("steering_rate_ki", steering_rate_ki);
    JSON_GET_DBL("steering_rate_kd", steering_rate_kd);
    JSON_GET_BOOL("steering_rate_pid_direction", steering_rate_pid_direction);
    JSON_GET_DBL("steering_rate_max_dps", steering_rate_max_dps);
    JSON_GET_BOOL("enable_sensor_2",        enable_sensor_2);
    JSON_GET_FLT("stuck_threshold_mm",      stuck_threshold_mm);
    JSON_GET_FLT("stuck_timeout_ms",        stuck_timeout_ms);
    JSON_GET_UINT8("BTxFollowChannel",      BTxFollowChannel);
    JSON_GET_INT("BTxThreshold",            BTxThreshold);
    JSON_GET_INT("BTxFollowThreshold",      BTxThreshold);
    JSON_GET_DBL("BTxAccelRate",            BTxAccelRate);
    JSON_GET_INT("BTxCreepDuration",        BTxCreepDuration);
    JSON_GET_UINT8("SineSwerveAmplitude",   SineSwerveAmplitude);
    JSON_GET_UINT8("SineSwerveFrequency",   SineSwerveFrequency);
    JSON_GET_DBL("CHARGE_VBATT_STOP",       CHARGE_VBATT_STOP);
    JSON_GET_DBL("CHARGE_MAX_CURRENT",      CHARGE_MAX_CURRENT);
    JSON_GET_INT("CHARGE_MAX_SECONDS",      CHARGE_MAX_SECONDS);
    JSON_GET_UINT8("battery_cells",         battery_cells);
    JSON_GET_BOOL("espnow_enabled",         espnow_enabled);
    JSON_GET_BOOL("telemetry_enabled",      telemetry_enabled);
    JSON_GET_INT("telemetry_rate_hz",       telemetry_rate_hz);
    JSON_GET_INT("telemetry_bins_hz",       telemetry_bins_hz);
    JSON_GET_BOOL("rover_enabled",           rover_enabled);
    JSON_GET_BOOL("rover_goal_cw",           rover_goal_cw);
    JSON_GET_DBL("rover_sensor_height_mm",   rover_sensor_height_mm);
    JSON_GET_DBL("rover_full_speed_mps",     rover_full_speed_mps);
    JSON_GET_DBL("rover_full_yaw_rate_dps",  rover_full_yaw_rate_dps);
    JSON_GET_DBL("rover_seek_vbatt",         rover_seek_vbatt);
    JSON_GET_INT("rover_seek_after_s",       rover_seek_after_s);
    JSON_GET_UINT8("rover_entry_btx",        rover_entry_btx);
    JSON_GET_UINT8("rover_exit_btx",         rover_exit_btx);
    JSON_GET_INT("rover_lost_timeout_s",     rover_lost_timeout_s);
    JSON_GET_DBL("rover_speed_limit_mps",    rover_speed_limit_mps);
    JSON_GET_DBL("espnow_tx_power_dbm",      espnow_tx_power_dbm);
    if (doc["cmd_key"].is<long>()) c.cmd_key = doc["cmd_key"].as<long>();
    JSON_GET_BOOL("imu_required",           imu_required);
    JSON_GET_INT("dsp_frame_period_us",     dsp_frame_period_us);
    JSON_GET_INT("dsp_sensors",             dsp_sensors);
    for (int b = 0; b < 8; b++) JSON_GET_DBL(dsp_bin_key(b), dsp_bin_hz[b]);
    JSON_GET_INT("dsp_xy_mask",             dsp_xy_mask);
    JSON_GET_INT("dsp_slow_adc",            dsp_slow_adc);
    JSON_GET_INT("dsp_pga",                 dsp_pga);
    JSON_GET_INT("dsp_lanes",               dsp_lanes);
    if (doc["dsp_sweep_bins"].is<const char*>()) {  // "off", "auto" or "1".."8"; anything else: off
        const char* d = doc["dsp_sweep_bins"].as<const char*>();
        const int n = atoi(d);
        c.dsp_sweep_bins = !strcmp(d, "auto") ? 255 : (n >= 1 && n <= 8) ? n : 0;
    } else if (doc["dsp_sweep_bins"].is<int>()) {
        const int n = doc["dsp_sweep_bins"].as<int>();
        c.dsp_sweep_bins = (n >= 0 && n <= 8) ? n : n == 255 ? 255 : 0;
    }
    JSON_GET_BOOL("usb_drive",              usb_drive);
    if (doc["radio_mode"].is<const char*>()) {  // "espnow" or "wifi"; any other text keeps the value
        const char* m = doc["radio_mode"].as<const char*>();
        if (!strcmp(m, "espnow")) c.radio_mode = RADIO_MODE_ESPNOW;
        else if (!strcmp(m, "wifi")) c.radio_mode = RADIO_MODE_WIFI;
    }
    if (legacy_file && doc["CHARGE_MAX_SECONDS"].is<int>()) c.CHARGE_MAX_SECONDS = doc["CHARGE_MAX_SECONDS"].as<int>() * 2;  // see legacy_file

#undef JSON_GET_INT
#undef JSON_GET_UINT8
#undef JSON_GET_DBL
#undef JSON_GET_FLT
#undef JSON_GET_BOOL

    return true;
}

static int old_set_key(DeviceConfig* cfg, const char* key, const char* value_json)
{
    const size_t n = strlen(key);
    if (n > 4 && !strcmp(key + n - 4, "_eff")) return 4;  // read-only: the value in effect, from the STM32
    if (!strncmp(key, "wifi_", 5)) return 4;              // read-only: the USB command "wifi set" writes them
    JsonDocument cur;
    old_config_to_doc(*cfg, cur);
    JsonVariant old = cur[key];
    if (old.isNull()) return 1;                       // unknown key
    // A text key takes a JSON string, or a bare word (letters, digits, '_'), as "set radio_mode wifi".
    String quoted;
    if (old.is<const char*>() && value_json[0] != '"') {
        for (const char* q = value_json; *q; q++)
            if (!isalnum((unsigned char)*q) && *q != '_') return 2;
        quoted = String("\"") + value_json + "\"";
        value_json = quoted.c_str();
    }
    JsonDocument val;
    if (deserializeJson(val, value_json)) return 2;   // not a JSON literal
    JsonVariant nv = val.as<JsonVariant>();
    // The value must have the key's type, or the parse below would ignore it silently.
    bool type_ok;
    if (old.is<bool>())      type_ok = nv.is<bool>();
    else if (old.is<const char*>()) type_ok = nv.is<const char*>();
    else if (old.is<long>()) type_ok = nv.is<long>();
    else                     type_ok = nv.is<double>() && !nv.is<bool>();
    if (!type_ok) return 2;
    String text = "{\"";
    text += key;
    text += "\":";
    text += value_json;
    text += "}";
    DeviceConfig tmp = *cfg;
    if (!old_config_from_json_text(tmp, text)) return 2;
    old_validate_config(tmp);                             // the same clamps as a config.json edit
    if (old.is<const char*>()) {                      // a text key: the value must survive the clamps unchanged
        JsonDocument chk;
        old_config_to_doc(tmp, chk);
        if (strcmp(chk[key].as<const char*>(), nv.as<const char*>())) return 2;
    }
    *cfg = tmp;
    return 0;
}
