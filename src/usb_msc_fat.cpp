// ─────────────────────────────────────────────────────────────────────────────
// usb_msc_fat.cpp
//
// USB Mass Storage Class device backed by a FAT filesystem in internal flash.
//
// Architecture:
//  - A dedicated flash partition labelled "storage" (1 MB, type=data/fat)
//    provides the raw block storage exposed over USB MSC with 512-byte sectors.
//  - A write-back erase-block cache absorbs the impedance mismatch between the
//    512-byte FAT/USB sector size and the 4096-byte flash erase granularity.
//  - FatFS (ff.h, ESP-IDF component) is mounted over a custom diskio driver
//    that directs all I/O through the same cache/flash functions used by MSC.
//  - USB MSC callbacks and FatFS access are kept mutually exclusive: FatFS is
//    only used when the USB host has been idle for ≥ MSC_WRITE_STABLE_MS ms.
//  - Parameters are persisted to NVS via the Preferences library and are also
//    stored in config.json on the FAT volume so the user can edit them.
// ─────────────────────────────────────────────────────────────────────────────

#include "decimator.h"  // rec::rateNearest
#include "usb_msc_fat.h"

#include "USB.h"
#include "USBMSC.h"

#include <Preferences.h>
#include <ArduinoJson.h>
#include <inttypes.h>
#include <math.h>

#include "esp_partition.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern "C" {
#include "ff.h"
#include "diskio_impl.h"
}

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────
static const char* TAG = "msc_fat";

#define STORAGE_PARTITION_LABEL   "storage"
#define STORAGE_SECTOR_SIZE       512u
#define FLASH_ERASE_BLOCK         4096u
#define SECTORS_PER_ERASE_BLOCK   (FLASH_ERASE_BLOCK / STORAGE_SECTOR_SIZE)   // 8
#define FAT_DRIVE                 "0:"
#define CONFIG_PATH_SUFFIX        "-config.json"
#define DEFAULT_CONFIG_PATH       "0:/config.json"
#define NVS_NAMESPACE             "wfrx_cfg"       // ≤ 15 chars
#define MSC_WRITE_STABLE_MS       3000u             // idle window before parse

// PWM safety limits enforced during validation
#define PWM_ABS_MIN  800
#define PWM_ABS_MAX  2200

// ─────────────────────────────────────────────────────────────────────────────
// Static state
// ─────────────────────────────────────────────────────────────────────────────

// Flash partition handle
static const esp_partition_t* s_partition = nullptr;
static uint32_t               s_sector_count = 0;

// 4-KB write-back erase-block cache.
// Protected by s_flash_mutex; accessed from USB task and main task.
static SemaphoreHandle_t s_flash_mutex  = nullptr;
static uint8_t           s_erase_cache[FLASH_ERASE_BLOCK];
static uint32_t          s_cached_eb    = UINT32_MAX; // current erase-block index
static bool              s_cache_dirty  = false;

// FatFS state
static FATFS s_fatfs;
static BYTE  s_pdrv = FF_DRV_NOT_USED;

// USB MSC
static USBMSC s_msc;
static uint32_t s_media_back_ms = 0;  // msc_fat_save_all: the drive shows "no medium" until this time
static bool     s_ejected = false;    // msc_fat_eject: "no medium" until the restart
static volatile uint32_t s_last_write_ms  = 0;
static volatile bool     s_ever_written   = false; // at least one write since boot

// Config file path (discovered at runtime, may have a prefix)
static char s_config_path[64] = "0:/config.json";

// Config.json change tracking (size + FatFS date/time stamp)
static uint32_t s_cfg_size = UINT32_MAX;
static uint16_t s_cfg_fdate = 0;
static uint16_t s_cfg_ftime = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Config file discovery
// ─────────────────────────────────────────────────────────────────────────────

// Search FAT volume for ANY config file matching *-config.json pattern.
// Returns the first matching file found in the root directory.
// If found, updates s_config_path and returns true.
// If not found, s_config_path remains at default "0:/config.json".
static bool find_config_file()
{
    FF_DIR dir;
    FILINFO info;
    if (f_opendir(&dir, FAT_DRIVE "/") != FR_OK) {
        return false;
    }

    const char* no_ext_suffix = "-config";
    const size_t suffix_len = strlen(CONFIG_PATH_SUFFIX);
    const size_t no_ext_len = strlen(no_ext_suffix);

    char best_name[64] = {0};
    bool best_is_json = false;

    while (f_readdir(&dir, &info) == FR_OK && info.fname[0] != 0) {
        if (info.fattrib & AM_DIR) continue;

        const char* name = info.fname;
        const size_t n = strlen(name);
        bool match_json = (n >= suffix_len) &&
            (strcmp(name + (n - suffix_len), CONFIG_PATH_SUFFIX) == 0);
        bool match_no_ext = (n >= no_ext_len) &&
            (strcmp(name + (n - no_ext_len), no_ext_suffix) == 0);

        if (match_json || match_no_ext) {
            bool cand_is_json = match_json;
            if (best_name[0] == 0) {
                strncpy(best_name, name, sizeof(best_name) - 1);
                best_name[sizeof(best_name) - 1] = 0;
                best_is_json = cand_is_json;
                continue;
            }

            // Deterministic priority:
            // 1) prefer *-config.json over *-config
            // 2) for ties, lexicographically smaller name
            bool take = false;
            if (cand_is_json && !best_is_json) {
                take = true;
            } else if (cand_is_json == best_is_json && strcmp(name, best_name) < 0) {
                take = true;
            }

            if (take) {
                strncpy(best_name, name, sizeof(best_name) - 1);
                best_name[sizeof(best_name) - 1] = 0;
                best_is_json = cand_is_json;
            }
        }
    }

    f_closedir(&dir);
    if (best_name[0] == 0) {
        return false;
    }

    int written = snprintf(s_config_path, sizeof(s_config_path), "0:/%s", best_name);
    if (written <= 0 || written >= (int)sizeof(s_config_path)) {
        return false;
    }

    ESP_LOGI(TAG, "Found config file: %s", s_config_path);
    return true;
}

// Remove temporary editor sidecar files created next to the active config file.
// Example: config.json.sb-1ebf95c7-ihAnmH on macOS.
static void cleanup_config_sidecar_files()
{
    const char* slash = strrchr(s_config_path, '/');
    const char* base = slash ? (slash + 1) : s_config_path;

    char sidecar_prefix[64];
    snprintf(sidecar_prefix, sizeof(sidecar_prefix), "%s.sb-", base);
    const size_t prefix_len = strlen(sidecar_prefix);

    FF_DIR dir;
    FILINFO info;
    if (f_opendir(&dir, FAT_DRIVE "/") != FR_OK) {
        return;
    }

    while (f_readdir(&dir, &info) == FR_OK && info.fname[0] != 0) {
        if (info.fattrib & AM_DIR) continue;

        if (strncmp(info.fname, sidecar_prefix, prefix_len) == 0) {
            char tmp_path[64];
            snprintf(tmp_path, sizeof(tmp_path), "0:/%s", info.fname);
            FRESULT res = f_unlink(tmp_path);
            if (res == FR_OK) {
                ESP_LOGI(TAG, "Removed temp sidecar: %s", tmp_path);
            } else {
                ESP_LOGW(TAG, "Failed to remove sidecar %s (%d)", tmp_path, res);
            }
        }
    }

    f_closedir(&dir);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Defaults
// ─────────────────────────────────────────────────────────────────────────────
static void populate_defaults(DeviceConfig& c,
                               const DeviceConfig* seed = nullptr)
{
    // If a seed is supplied (from NVS) use it; otherwise use compiled defaults.
    if (seed) {
        c = *seed;
        return;
    }
    c.vehicleID             = 1;
    c.rx_channel            = 1;
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
    c.usb_drive                = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Validation / clamping
// ─────────────────────────────────────────────────────────────────────────────
static inline int clamp_pwm(int v) {
    if (v < PWM_ABS_MIN) v = PWM_ABS_MIN;
    if (v > PWM_ABS_MAX) v = PWM_ABS_MAX;
    return v;
}

static void validate_config(DeviceConfig& c)
{
    // Channel
    if (c.rx_channel < 1 || c.rx_channel > 5) c.rx_channel = 1;
    c.telemetry_rate_hz = rec::rateNearest(c.telemetry_rate_hz);  // 100/N Hz only
    c.telemetry_bins_hz = c.telemetry_bins_hz <= 0 ? 0 : rec::rateNearest(c.telemetry_bins_hz);  // 0 = off
    if (c.battery_cells < 1 || c.battery_cells > 12) c.battery_cells = 2;

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
    // 16.0 V, current 6.0 A, charge time 3600 s. The ESP32 sends 2 x CHARGE_MAX_SECONDS (as shipped
    // units do), so 1800 is the highest setting with an effect. Above a limit the STM32 used the limit.
    if (c.CHARGE_VBATT_STOP  < 6.0)  c.CHARGE_VBATT_STOP  = 6.0;
    if (c.CHARGE_VBATT_STOP  > 16.0) c.CHARGE_VBATT_STOP  = 16.0;
    if (c.CHARGE_MAX_CURRENT < 0.1)  c.CHARGE_MAX_CURRENT = 0.1;
    if (c.CHARGE_MAX_CURRENT > 6.0) c.CHARGE_MAX_CURRENT = 6.0;
    if (c.CHARGE_MAX_SECONDS < 5)   c.CHARGE_MAX_SECONDS = 5;
    if (c.CHARGE_MAX_SECONDS > 1800) c.CHARGE_MAX_SECONDS = 1800;

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
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: NVS (Preferences)
// ─────────────────────────────────────────────────────────────────────────────
static void prefs_save(const DeviceConfig& c)
{
    Preferences p;
    p.begin(NVS_NAMESPACE, false);
    p.putUChar("vID",      c.vehicleID);
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
    p.putBool("usbDrive",  c.usb_drive);
    p.end();
}

static bool prefs_load(DeviceConfig& c)
{
    Preferences p;
    if (!p.begin(NVS_NAMESPACE, true)) return false;
    if (!p.isKey("vID")) { p.end(); return false; } // namespace is empty

    c.vehicleID             = p.getUChar("vID",      c.vehicleID);
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
    c.usb_drive                = p.getBool("usbDrive",  c.usb_drive);
    p.end();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Flash block device – erase-block write-back cache
// ─────────────────────────────────────────────────────────────────────────────

// Must hold s_flash_mutex before calling any of the functions below.

static esp_err_t flash_flush_cache_locked()
{
    if (!s_cache_dirty || s_cached_eb == UINT32_MAX) return ESP_OK;

    uint32_t eb_offset = s_cached_eb * FLASH_ERASE_BLOCK;
    esp_err_t err = esp_partition_erase_range(s_partition, eb_offset,
                                               FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    err = esp_partition_write(s_partition, eb_offset,
                               s_erase_cache, FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    s_cache_dirty = false;
    return ESP_OK;
}

// Load erase-block 'eb' into the cache (flushing any dirty previous block first).
static esp_err_t flash_load_erase_block_locked(uint32_t eb)
{
    if (s_cached_eb == eb) return ESP_OK;  // already loaded

    esp_err_t err = flash_flush_cache_locked();
    if (err != ESP_OK) return err;

    uint32_t eb_offset = eb * FLASH_ERASE_BLOCK;
    err = esp_partition_read(s_partition, eb_offset,
                              s_erase_cache, FLASH_ERASE_BLOCK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read erase block failed @ 0x%08" PRIx32 ": %s",
                 eb_offset, esp_err_to_name(err));
        return err;
    }
    s_cached_eb   = eb;
    s_cache_dirty = false;
    return ESP_OK;
}

// Public (within this file) read / write / flush helpers.
// These acquire the mutex internally.

static bool flash_read_sector(uint32_t lba, uint8_t* buf)
{
    if (!s_partition || lba >= s_sector_count) return false;

    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    uint32_t eb = lba / SECTORS_PER_ERASE_BLOCK;
    uint32_t offset_in_eb = (lba % SECTORS_PER_ERASE_BLOCK) * STORAGE_SECTOR_SIZE;

    esp_err_t err;
    if (s_cached_eb == eb) {
        // Serve from cache (which may have unsaved writes)
        memcpy(buf, s_erase_cache + offset_in_eb, STORAGE_SECTOR_SIZE);
        err = ESP_OK;
    } else {
        // Read directly from flash (cache holds a different block)
        err = esp_partition_read(s_partition,
                                  lba * STORAGE_SECTOR_SIZE,
                                  buf, STORAGE_SECTOR_SIZE);
    }
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

static bool flash_write_sector(uint32_t lba, const uint8_t* buf)
{
    if (!s_partition || lba >= s_sector_count) return false;

    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    uint32_t eb = lba / SECTORS_PER_ERASE_BLOCK;
    uint32_t offset_in_eb = (lba % SECTORS_PER_ERASE_BLOCK) * STORAGE_SECTOR_SIZE;

    esp_err_t err = flash_load_erase_block_locked(eb);
    if (err == ESP_OK) {
        memcpy(s_erase_cache + offset_in_eb, buf, STORAGE_SECTOR_SIZE);
        s_cache_dirty = true;
    }
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

static bool flash_flush()
{
    xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    esp_err_t err = flash_flush_cache_locked();
    xSemaphoreGive(s_flash_mutex);
    return (err == ESP_OK);
}

// ─────────────────────────────────────────────────────────────────────────────
// FatFS diskio driver
// ─────────────────────────────────────────────────────────────────────────────

static DSTATUS diskio_init_cb(unsigned char pdrv)
{
    (void) pdrv;
    return s_partition ? 0 : STA_NOINIT;
}

static DSTATUS diskio_status_cb(unsigned char pdrv)
{
    (void) pdrv;
    return s_partition ? 0 : STA_NOINIT;
}

static DRESULT diskio_read_cb(unsigned char pdrv,
                               unsigned char* buff,
                               uint32_t sector,
                               unsigned count)
{
    (void) pdrv;
    for (unsigned i = 0; i < count; ++i) {
        if (!flash_read_sector(sector + i, buff + i * STORAGE_SECTOR_SIZE))
            return RES_ERROR;
    }
    return RES_OK;
}

static DRESULT diskio_write_cb(unsigned char pdrv,
                                const unsigned char* buff,
                                uint32_t sector,
                                unsigned count)
{
    (void) pdrv;
    for (unsigned i = 0; i < count; ++i) {
        if (!flash_write_sector(sector + i, buff + i * STORAGE_SECTOR_SIZE))
            return RES_ERROR;
    }
    // Flush after FatFS write sequences so the data persists before the next
    // power cycle.
    return flash_flush() ? RES_OK : RES_ERROR;
}

static DRESULT diskio_ioctl_cb(unsigned char pdrv, unsigned char cmd, void* buff)
{
    (void) pdrv;
    switch (cmd) {
        case CTRL_SYNC:
            return flash_flush() ? RES_OK : RES_ERROR;
        case GET_SECTOR_COUNT:
            *(DWORD*)buff = (DWORD)s_sector_count;
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = (WORD)STORAGE_SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = (DWORD)SECTORS_PER_ERASE_BLOCK;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FatFS mount / unmount
// ─────────────────────────────────────────────────────────────────────────────

static bool fatfs_mount()
{
    FRESULT res = f_mount(&s_fatfs, FAT_DRIVE, 1 /* force mount */);
    if (res == FR_OK) return true;

    ESP_LOGW(TAG, "f_mount failed (%d); formatting...", res);
    // Format the volume (FM_FAT = FAT12/16, au=0 = default cluster size)
    BYTE work[FF_MAX_SS];
    res = f_mkfs(FAT_DRIVE, FM_FAT, 0, work, sizeof(work));
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_mkfs failed (%d)", res);
        return false;
    }
    res = f_mount(&s_fatfs, FAT_DRIVE, 1);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_mount after mkfs failed (%d)", res);
        return false;
    }
    // Set volume label only if this FatFS build enables label APIs.
#if FF_USE_LABEL
    f_setlabel("WFRX CONFIG");
#endif
    return true;
}

static void fatfs_unmount()
{
    f_mount(nullptr, FAT_DRIVE, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON serialise / parse
// ─────────────────────────────────────────────────────────────────────────────

// Build a pretty-printed JSON document from cfg and write it to s_config_path.
double g_dsp_eff[DSP_EFF_N] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

static const char* dsp_bin_key(int b)
{
    static const char* const K[8] = {"dsp_bin_hz_0", "dsp_bin_hz_1", "dsp_bin_hz_2", "dsp_bin_hz_3",
                                     "dsp_bin_hz_4", "dsp_bin_hz_5", "dsp_bin_hz_6", "dsp_bin_hz_7"};
    return K[b];
}

// Every config.json key with its value, as one JSON object (the file, and CMD get/list over ESP-NOW).
// with_eff: also the read-only DSP values in effect (<key>_eff; null = no STM32 answer yet), for get and list.
static void config_to_doc(const DeviceConfig& c, JsonDocument& doc, bool with_eff = false)
{
    doc["vehicleID"]             = c.vehicleID;
    doc["rx_channel"]            = c.rx_channel;
    doc["CRUISE_THROTTLE"]       = c.CRUISE_THROTTLE;
    doc["CREEP_THROTTLE"]        = c.CREEP_THROTTLE;
    doc["PWM_OUT_1_DEFAULT"]     = c.PWM_OUT_1_DEFAULT;
    doc["PWM_OUT_1_MIN"]         = c.PWM_OUT_1_MIN;
    doc["PWM_OUT_1_MAX"]         = c.PWM_OUT_1_MAX;
    doc["PWM_OUT_2_DEFAULT"]     = c.PWM_OUT_2_DEFAULT;
    doc["PWM_OUT_2_MIN"]         = c.PWM_OUT_2_MIN;
    doc["PWM_OUT_2_MAX"]         = c.PWM_OUT_2_MAX;
    doc["PWM_OUT_3_DEFAULT"]     = c.PWM_OUT_3_DEFAULT;
    doc["PWM_OUT_3_MIN"]         = c.PWM_OUT_3_MIN;
    doc["PWM_OUT_3_MAX"]         = c.PWM_OUT_3_MAX;
    doc["PWM_OUT_4_DEFAULT"]     = c.PWM_OUT_4_DEFAULT;
    doc["PWM_OUT_4_MIN"]         = c.PWM_OUT_4_MIN;
    doc["PWM_OUT_4_MAX"]         = c.PWM_OUT_4_MAX;
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
    doc["usb_drive"]                = c.usb_drive;
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
}

static bool config_write_json(const DeviceConfig& c)
{
    // The document holds approximately 50 key-value pairs.
    JsonDocument doc;
    config_to_doc(c, doc);

    // Measure serialised size first
    size_t json_len = measureJsonPretty(doc);

    // Build output in memory first; only open/truncate the file after we know
    // serialization completed successfully.
    String json_str;
    if (!json_str.reserve((unsigned)json_len + 4)) {
        ESP_LOGE(TAG, "json_str reserve failed");
        return false;
    }
    size_t written_json = serializeJsonPretty(doc, json_str);
    if (written_json != json_len) {
        ESP_LOGE(TAG, "serializeJsonPretty truncated (%u/%u)",
                 (unsigned)written_json, (unsigned)json_len);
        return false;
    }

    String out_str;
    if (!out_str.reserve(json_str.length() + 4)) {
        ESP_LOGE(TAG, "out_str reserve failed");
        return false;
    }
    out_str += json_str;
    out_str += "\n";

    FIL fil;
    FRESULT res = f_open(&fil, s_config_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_open write failed (%d)", res);
        return false;
    }

    UINT bw;
    res = f_write(&fil, out_str.c_str(), (UINT)out_str.length(), &bw);
    f_close(&fil);

    if (res != FR_OK || bw != out_str.length()) {
        ESP_LOGE(TAG, "f_write failed (%d)", res);
        return false;
    }
    return true;
}

static bool config_from_json_text(DeviceConfig& c, const String& json_no_comments);

// Parse s_config_path on the mounted FAT volume into cfg.
// Unknown / invalid fields are silently ignored; previously valid fields keep
// their current (NVS-loaded) value.
static bool s_rate_config_missing = false;

static bool config_read_json(DeviceConfig& c)
{
    FIL fil;
    FRESULT res = f_open(&fil, s_config_path, FA_READ);
    if (res != FR_OK) {
        ESP_LOGW(TAG, "config.json not found (%d)", res);
        return false;
    }

    FSIZE_t fsize = f_size(&fil);
    if (fsize == 0 || fsize > 8192) {
        f_close(&fil);
        ESP_LOGW(TAG, "config.json invalid size (%u)", (unsigned)fsize);
        return false;
    }

    String json_str;
    json_str.reserve((unsigned)fsize + 1);

    char rd_buf[256];
    UINT br;
    while (f_read(&fil, rd_buf, sizeof(rd_buf), &br) == FR_OK && br > 0) {
        json_str.concat(rd_buf, br);
    }
    f_close(&fil);

    // Allow human-readable comments in config.json by stripping C/C++ comment
    // syntax before handing content to ArduinoJson.
    String json_no_comments;
    json_no_comments.reserve(json_str.length());
    bool in_string = false;
    bool escaped = false;
    bool in_line_comment = false;
    bool in_block_comment = false;

    for (size_t i = 0; i < json_str.length(); ++i) {
        char ch = json_str[i];
        char next = (i + 1 < json_str.length()) ? json_str[i + 1] : '\0';

        if (in_line_comment) {
            if (ch == '\n') {
                in_line_comment = false;
                json_no_comments += ch;
            }
            continue;
        }

        if (in_block_comment) {
            if (ch == '*' && next == '/') {
                in_block_comment = false;
                ++i;
            }
            continue;
        }

        if (in_string) {
            json_no_comments += ch;
            if (escaped) {
                escaped = false;
            } else if (ch == '\\') {
                escaped = true;
            } else if (ch == '"') {
                in_string = false;
            }
            continue;
        }

        if (ch == '"') {
            in_string = true;
            json_no_comments += ch;
            continue;
        }

        if (ch == '/' && next == '/') {
            in_line_comment = true;
            ++i;
            continue;
        }

        if (ch == '/' && next == '*') {
            in_block_comment = true;
            ++i;
            continue;
        }

        json_no_comments += ch;
    }

    return config_from_json_text(c, json_no_comments);
}

// Apply the keys present in a JSON object to c (config.json content, or one CMD set over ESP-NOW).
static bool config_from_json_text(DeviceConfig& c, const String& json_no_comments)
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
    s_rate_config_missing = doc["steering_rate_kp"].isNull() || doc["steering_rate_ki"].isNull() || doc["steering_rate_kd"].isNull() || doc["steering_rate_pid_direction"].isNull() || doc["steering_rate_max_dps"].isNull();

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
    JSON_GET_BOOL("usb_drive",              usb_drive);

#undef JSON_GET_INT
#undef JSON_GET_UINT8
#undef JSON_GET_DBL
#undef JSON_GET_FLT
#undef JSON_GET_BOOL

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Config file change detection helpers
// ─────────────────────────────────────────────────────────────────────────────

// Record the current size/timestamp of config.json (must be called with FAT
// Record the current size/timestamp of the config file (must be called with FAT
// mounted).
static void record_config_stamp()
{
    FILINFO info;
    if (f_stat(s_config_path, &info) == FR_OK) {
        s_cfg_size  = (uint32_t)info.fsize;
        s_cfg_fdate = info.fdate;
        s_cfg_ftime = info.ftime;
    } else {
        s_cfg_size  = UINT32_MAX;
        s_cfg_fdate = 0;
        s_cfg_ftime = 0;
    }
}

// Returns true if config file differs from what we last recorded.
static bool config_has_changed()
{
    FILINFO info;
    if (f_stat(s_config_path, &info) != FR_OK) return false;
    return (info.fsize  != s_cfg_size  ||
            info.fdate  != s_cfg_fdate ||
            info.ftime  != s_cfg_ftime);
}

// ─────────────────────────────────────────────────────────────────────────────
// USB MSC callbacks
// ─────────────────────────────────────────────────────────────────────────────

static int32_t on_msc_read(uint32_t lba, uint32_t offset,
                             void* buffer, uint32_t bufsize)
{
    uint8_t* dst = static_cast<uint8_t*>(buffer) + offset;
    uint32_t sectors = bufsize / STORAGE_SECTOR_SIZE;
    for (uint32_t i = 0; i < sectors; ++i) {
        if (!flash_read_sector(lba + i, dst + i * STORAGE_SECTOR_SIZE))
            return -1;
    }
    return (int32_t)bufsize;
}

static int32_t on_msc_write(uint32_t lba, uint32_t offset,
                              uint8_t* buffer, uint32_t bufsize)
{
    const uint8_t* src = buffer + offset;
    uint32_t sectors = bufsize / STORAGE_SECTOR_SIZE;
    for (uint32_t i = 0; i < sectors; ++i) {
        if (!flash_write_sector(lba + i, src + i * STORAGE_SECTOR_SIZE))
            return -1;
    }
    // Record that a write happened; the quiet timer is checked in msc_fat_loop().
    s_last_write_ms = (uint32_t)millis();
    s_ever_written  = true;
    return (int32_t)bufsize;
}

static bool on_msc_start_stop(uint8_t power_condition,
                                bool start, bool load_eject)
{
    (void)power_condition;
    if (load_eject && !start) {
        // Host requested eject: flush write cache immediately to ensure data
        // is committed before the user unplugs the cable.
        ESP_LOGI(TAG, "USB eject: flushing flash cache");
        flash_flush();
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// msc_fat_init
// ─────────────────────────────────────────────────────────────────────────────
void msc_fat_init(DeviceConfig* cfg)
{
    // 1. Create the flash access mutex.
    s_flash_mutex = xSemaphoreCreateMutex();
    configASSERT(s_flash_mutex);

    // 2. Locate the "storage" partition.
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                            ESP_PARTITION_SUBTYPE_ANY,
                                            STORAGE_PARTITION_LABEL);
    if (!s_partition) {
        ESP_LOGE(TAG, "Partition '%s' not found – check partitions.csv",
                 STORAGE_PARTITION_LABEL);
        // We cannot continue without the partition, but we still populate cfg
        // from defaults + NVS so the firmware can run.
        populate_defaults(*cfg);
        prefs_load(*cfg);
        validate_config(*cfg);
        return;
    }

    s_sector_count = s_partition->size / STORAGE_SECTOR_SIZE;
    ESP_LOGI(TAG, "Storage partition: %u KB, %u sectors",
             (unsigned)(s_partition->size / 1024), (unsigned)s_sector_count);

    // 3. Register the FatFS diskio driver.
    // Keep FatFS drive index fixed to 0 because paths are intentionally fixed
    // as "0:" and "0:/config.json" for compatibility with existing code.
    s_pdrv = 0;

    static const ff_diskio_impl_t diskio_impl = {
        .init   = diskio_init_cb,
        .status = diskio_status_cb,
        .read   = diskio_read_cb,
        .write  = diskio_write_cb,
        .ioctl  = diskio_ioctl_cb,
    };
    ff_diskio_register(s_pdrv, &diskio_impl);

    // 4. Start with compiled defaults, then override from NVS.
    populate_defaults(*cfg);
    prefs_load(*cfg);  // no-op if NVS is empty (first boot)
    validate_config(*cfg);

    // 5. Mount FAT, create config file on first boot, read it, unmount.
    if (fatfs_mount()) {
        // Search for any file matching *-config.json pattern; if found, use it.
        // Otherwise, s_config_path will remain at default "0:/config.json".
        find_config_file();
        cleanup_config_sidecar_files();

        FILINFO info;
        bool file_exists = (f_stat(s_config_path, &info) == FR_OK);

        if (!file_exists) {
            ESP_LOGI(TAG, "First boot: creating %s", s_config_path);
            config_write_json(*cfg);
            // Re-stat after write
            file_exists = (f_stat(s_config_path, &info) == FR_OK);
        } else {
            // File exists – parse it and override NVS values.
            ESP_LOGI(TAG, "config.json found; parsing");
            DeviceConfig file_cfg = *cfg;    // start from NVS base
            if (config_read_json(file_cfg)) {
                validate_config(file_cfg);
                *cfg = file_cfg;
                prefs_save(*cfg);
                if (s_rate_config_missing) config_write_json(*cfg);
            }
        }
        record_config_stamp();
        fatfs_unmount();
    } else {
        ESP_LOGE(TAG, "FAT mount/format failed");
    }

    // 6. Configure and start USB MSC (setting usb_drive; false: no drive on USB).
    if (!cfg->usb_drive) { ESP_LOGI(TAG, "usb_drive false: no USB drive"); return; }
    s_msc.vendorID("WFRX");
    s_msc.productID("CONFIG_DRIVE");
    s_msc.productRevision("1.0");
    s_msc.onStartStop(on_msc_start_stop);
    s_msc.onRead(on_msc_read);
    s_msc.onWrite(on_msc_write);
    s_msc.mediaPresent(true);
    s_msc.begin((uint32_t)s_sector_count, STORAGE_SECTOR_SIZE);
    // USB.begin() is called by Serial.begin() / the Arduino core; no need to
    // call it explicitly here.
}

// ─────────────────────────────────────────────────────────────────────────────
// msc_fat_loop
// ─────────────────────────────────────────────────────────────────────────────
void msc_fat_eject()
{
    s_ejected = true;
    s_media_back_ms = 0;
    s_msc.mediaPresent(false);
}

void msc_fat_media_service()
{
    if (s_media_back_ms && (int32_t)((uint32_t)millis() - s_media_back_ms) >= 0) {
        s_media_back_ms = 0;
        s_msc.mediaPresent(true);  // a PC that holds the drive reads the new config.json
    }
}

bool msc_fat_loop(DeviceConfig* cfg)
{
    msc_fat_media_service();
    if (!s_ever_written) return false;

    uint32_t now = (uint32_t)millis();
    uint32_t idle_ms = now - s_last_write_ms;

    if (idle_ms < MSC_WRITE_STABLE_MS) return false;  // still settling

    // Host has been idle for ≥ 3 s.  Reset the trigger so we only act once per
    // idle window.
    s_ever_written = false;

    // Flush any remaining write-back cache so flash is consistent.
    if (!flash_flush()) {
        ESP_LOGE(TAG, "Cache flush failed after USB idle");
        return false;
    }

    // Mount FAT and check whether config.json changed.
    if (!fatfs_mount()) {
        ESP_LOGE(TAG, "FAT remount failed after USB idle");
        return false;
    }

    // Some editors create temporary sidecar files (for example on macOS).
    // Remove them so they do not accumulate on the config volume.
    cleanup_config_sidecar_files();

    bool changed = config_has_changed();
    if (!changed) {
        ESP_LOGI(TAG, "config.json unchanged");
        fatfs_unmount();
        return false;
    }

    ESP_LOGI(TAG, "config.json changed; parsing");
    DeviceConfig new_cfg = *cfg;   // start from current values
    bool ok = config_read_json(new_cfg);

    if (ok) {
        validate_config(new_cfg);
        record_config_stamp();
        fatfs_unmount();
        *cfg = new_cfg;
        prefs_save(new_cfg);
        return true;
    } else {
        ESP_LOGW(TAG, "config.json parse failed; keeping existing config");
        record_config_stamp();
        fatfs_unmount();
        return false;
    }
}

void msc_fat_save_nvs(const DeviceConfig* cfg)
{
    if (!cfg) return;
    DeviceConfig tmp = *cfg;
    validate_config(tmp);
    prefs_save(tmp);
}

bool msc_fat_save_all(const DeviceConfig* cfg)
{
    if (!cfg) return false;

    DeviceConfig tmp = *cfg;
    validate_config(tmp);

    // Always persist to NVS first so settings survive reboot even if FAT write fails.
    prefs_save(tmp);

    // A PC that has the drive mounted keeps its own copy of the FAT. The drive shows "no medium"
    // during the write and for 2 s after it, so the PC drops that copy and reads the drive again.
    // After msc_fat_eject the medium stays away.
    s_msc.mediaPresent(false);
    s_media_back_ms = s_ejected ? 0 : (uint32_t)millis() + 2000;
    if (!fatfs_mount()) {
        ESP_LOGE(TAG, "FAT mount failed while saving config.json");
        return false;
    }

    bool ok = config_write_json(tmp);
    if (ok) {
        record_config_stamp();
    } else {
        ESP_LOGE(TAG, "Failed to write config file: %s", s_config_path);
    }
    fatfs_unmount();
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Single keys, for config commands over ESP-NOW (docs/espnow_protocol.md, CMD 0x10)
// ─────────────────────────────────────────────────────────────────────────────
bool msc_fat_get_key(const DeviceConfig* cfg, const char* key, String* value_json)
{
    JsonDocument doc;
    config_to_doc(*cfg, doc, true);
    JsonObject o = doc.as<JsonObject>();
    bool known = false;  // a _eff key with no STM32 answer yet holds null: it exists all the same
    for (JsonPair kv : o) if (!strcmp(kv.key().c_str(), key)) { known = true; break; }
    if (!known) return false;
    JsonVariant v = doc[key];
    value_json->clear();
    serializeJson(v, *value_json);
    return true;
}

int msc_fat_set_key(DeviceConfig* cfg, const char* key, const char* value_json)
{
    const size_t n = strlen(key);
    if (n > 4 && !strcmp(key + n - 4, "_eff")) return 4;  // read-only: the value in effect, from the STM32
    JsonDocument cur;
    config_to_doc(*cfg, cur);
    JsonVariant old = cur[key];
    if (old.isNull()) return 1;                       // unknown key
    JsonDocument val;
    if (deserializeJson(val, value_json)) return 2;   // not a JSON literal
    JsonVariant nv = val.as<JsonVariant>();
    // The value must have the key's type, or the parse below would ignore it silently.
    bool type_ok;
    if (old.is<bool>())      type_ok = nv.is<bool>();
    else if (old.is<long>()) type_ok = nv.is<long>();
    else                     type_ok = nv.is<double>() && !nv.is<bool>();
    if (!type_ok) return 2;
    String text = "{\"";
    text += key;
    text += "\":";
    text += value_json;
    text += "}";
    DeviceConfig tmp = *cfg;
    if (!config_from_json_text(tmp, text)) return 2;
    validate_config(tmp);                             // the same clamps as a config.json edit
    *cfg = tmp;
    return 0;
}

bool msc_fat_key_at(const DeviceConfig* cfg, int index, String* line)
{
    JsonDocument doc;
    config_to_doc(*cfg, doc, true);
    int i = 0;
    for (JsonPair kv : doc.as<JsonObject>()) {
        if (i++ != index) continue;
        String value;
        serializeJson(kv.value(), value);  // ArduinoJson 7 replaces a String's content, so not into *line
        *line = kv.key().c_str();
        *line += "=";
        *line += value;
        return true;
    }
    return false;
}
