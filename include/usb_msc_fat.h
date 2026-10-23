#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// DeviceConfig
//
// Holds all runtime-configurable device parameters.  On boot the values are
// loaded from NVS (Preferences) then overridden by config.json if the file has
// changed since the last reading.  When the host edits config.json over USB MSC
// and safely ejects the drive, the new values are parsed and this struct is
// updated.
// ─────────────────────────────────────────────────────────────────────────────
struct DeviceConfig {
    // Identification
    char    vehicle_name[17];  // shown by the telemetry app: 16 at most; letters, digits, space, _ and -
    uint8_t vehicleID;
    uint8_t rx_channel;

    // Drive parameters
    int CRUISE_THROTTLE;
    int CREEP_THROTTLE;

    // Drive (0.5.69): 0 auto (motor controllers once one answers on CAN, else servos), 1 servos (V1.0), 2 CAN
    uint8_t drive_output;
    uint8_t steer_output;     // servo output channel of the steering, 1..4
    uint8_t throttle_output;  // servo output channel of the throttle, 1..4

    // Steering function: centre and end points (config.json steer_center_us, steer_min_us, steer_max_us)
    int PWM_OUT_1_DEFAULT;
    int PWM_OUT_1_MIN;
    int PWM_OUT_1_MAX;

    // Throttle function (config.json throttle_center_us, throttle_min_us, throttle_max_us)
    int PWM_OUT_2_DEFAULT;
    int PWM_OUT_2_MIN;
    int PWM_OUT_2_MAX;

    // Outputs 3 and 4: read from an older config.json only; an output with no function holds 1500 us
    int PWM_OUT_3_DEFAULT;
    int PWM_OUT_3_MIN;
    int PWM_OUT_3_MAX;

    // Output 4 – auxiliary
    int PWM_OUT_4_DEFAULT;
    int PWM_OUT_4_MIN;
    int PWM_OUT_4_MAX;

    // Front PID
    double kp;
    double ki;
    double kd;
    int    pid_direction;   // PID_v1: DIRECT=0, REVERSE=1
    double steer_lpf_hz;    // 2nd-order low-pass on the steering PID output, Hz (0 = off; V1.0: 5)
    // Yaw-rate PID: input/setpoint in deg/s, output normalized to -1..1.
    double steering_rate_kp;
    double steering_rate_ki;
    double steering_rate_kd;
    int    steering_rate_pid_direction; // DIRECT=0, REVERSE=1
    double steering_rate_max_dps;       // Full-scale steering request
    bool   enable_sensor_2;

    // Stuck-vehicle detector
    float stuck_threshold_mm;
    float stuck_timeout_ms;   // milliseconds

    // BTx proximity
    uint8_t BTxFollowChannel;
    int     BTxThreshold;
    double  BTxAccelRate;
    int     BTxCreepDuration; // seconds

    // Sine swerve
    uint8_t SineSwerveAmplitude; // mm (0 = disabled)
    uint8_t SineSwerveFrequency; // seconds per cycle

    // Charging
    double CHARGE_VBATT_STOP;    // V
    double CHARGE_MAX_CURRENT;   // A
    int    CHARGE_MAX_SECONDS;  // the charge time limit, s (0.5.69; up to 0.5.68 half of it)
    uint8_t battery_cells;       // series cells (2 = 2S); the station scales its voltage graphs from it

    // Features
    bool   espnow_enabled;
    bool   telemetry_enabled;  // binary telemetry log over ESP-NOW (starts ESP-NOW by itself)
    int    telemetry_bins_hz;  // 8-bin coil record rate: 0 (off), 100, 50, 25, 20, 10, 5, 4, 2 or 1 (same filter)
    int    telemetry_rate_hz;  // log sample rate: 100, 50, 25, 20, 10, 5, 4, 2 or 1 (anti-alias filter below 100)

    // Rover autopilot (lib/rover_autopilot): replaces wire following in mode 2 when enabled
    bool    rover_goal_cw;           // dock direction along the wire, seen from above
    double  rover_sensor_height_mm;  // sensor centre above the wire
    double  rover_seek_vbatt;        // V: go home below this battery voltage (0 = off)
    uint8_t rover_entry_btx;         // BTx channel before the dock: creep (0 = none)
    uint8_t rover_exit_btx;          // BTx channel after the dock: wrong-way guard (0 = none)
    int     rover_lost_timeout_s;    // stop for good after this long without the wire (0 = never)
    double  espnow_tx_power_dbm;     // radio transmit power, 2..19.5 dBm in 0.25 dB steps (19.5 = the radio's maximum)
    long    cmd_key;                 // fleet key carried by CMD 0x10, STOP 0x12 and RESTART 0x7F; must match; 0 (default) = blank, accepts key 0.
                                     // Sent in clear: it guards against mistakes and other monitors, not attacks.
    bool    imu_required;            // true (default, as the main-branch baseline): a failed MPU6050 stops the drive at start-up.
                                     // false: a board with no IMU (bench) runs without one.
    // DSP of the STM32 v2 app. The ESP32 holds these and writes them at each STM32 start and after a change
    // (the STM32 keeps none of them over a restart). A v1 STM32 app ignores them.
    int     dsp_frame_period_us;     // CID 18: frame period request, us (DSP_FRAME_PERIOD_MIN_US..655350); the STM32 answers the period in effect
    int     dsp_sensors;             // CID 38: 1 or 2 sensors
    double  dsp_bin_hz[8];           // CID 20: bin frequency, Hz; 0 = bin off
    int     dsp_xy_mask;             // CID 25: bit n = bin n in xy mode
    int     dsp_slow_adc;            // CID 39: 1 on, 0 off (off: no charging, no MEAS page)
    int     dsp_pga;                 // CID 29: -1 auto, 0..2 a fixed step for both sensors
    int     dsp_lanes;               // CID 13: lane map 0..3 (with dsp_sensors 1, bit 0 selects the sensor)
    uint8_t dsp_sweep_bins;          // CID 43 (Rx STM32 2.0.49+): bins off (dsp_bin_hz 0) that sweep 46.8-62.8 kHz;
                                     // 0 off, 1..8 that many, 255 auto (all free bins; the default). "off", "auto", "1".."8"
    bool    usb_drive;               // true (default): config.json as a USB drive; false: no drive, settings by command only
                                     // (from the next start). A PC that read the drive when the board left USB lost ports.
    uint8_t radio_mode;              // RADIO_MODE_ESPNOW (default, "espnow" in config.json) or RADIO_MODE_WIFI ("wifi"):
                                     // telemetry over ESP-NOW or over a Wi-Fi network, never both (from the next start)

    // Fields that the settings table declares (SF lines in include/settings_table.h)
#define S(...)
#define SF(name, ctype, ...) ctype name;
#include "settings_table.h"
#undef S
#undef SF
};

#define RADIO_MODE_ESPNOW 0
#define RADIO_MODE_WIFI   1

// The shortest frame period setting: the fastest STM32 period (2336 us, set-up S2N) plus 50 %, rounded up.
// The ESP32 then has margin to serve the radio, the log and USB between frames.
#define DSP_FRAME_PERIOD_MIN_US 2938  // 0.5.103: the shortest with no link; each link mode has its own (src/platform/radio.inc)

// DSP values in effect, from the STM32 answers (NaN = no answer yet). "list" and "get" show them as the
// read-only keys <key>_eff; config.json does not hold them.
enum { DSP_EFF_PERIOD, DSP_EFF_SENSORS, DSP_EFF_BIN0, DSP_EFF_XY = DSP_EFF_BIN0 + 8, DSP_EFF_SLOW, DSP_EFF_PGA,
       DSP_EFF_LANES, DSP_EFF_N };
extern double g_dsp_eff[DSP_EFF_N];

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Initialise the USB MSC device and FAT filesystem layer.
 *
 * Must be called from setup() BEFORE Serial.begin() (or at least before the
 * USB stack is started) so that the MSC descriptor is registered in time.
 *
 * On return, *cfg is populated from NVS (and config.json if it agrees with
 * the stored "last seen" state), ready to be applied to the firmware globals.
 *
 * @param cfg  Pointer to a DeviceConfig struct that will be filled with the
 *             boot-time configuration.
 */
void msc_fat_init(DeviceConfig* cfg);

/**
 * Maintenance function – call once per main loop iteration (or at least
 * frequently, e.g. every few ms).
 *
 * Detects when the USB host has stopped writing (3-second quiet window) and
 * then flushes the flash write-back cache, mounts the FAT filesystem, checks
 * whether config.json has changed and, if so, parses the file and updates *cfg.
 *
 * @param cfg  Pointer to the same DeviceConfig passed to msc_fat_init().
 * @return     true if *cfg was updated and the caller should re-apply it.
 */
bool msc_fat_loop(DeviceConfig* cfg);

/**
 * Persist the provided configuration to NVS immediately.
 *
 * This is intended for firmware-initiated setting changes (for example button
 * actions) so they survive reboot even when config.json is not edited by host.
 */
void msc_fat_save_nvs(const DeviceConfig* cfg);

void msc_fat_media_service();  // the medium back after msc_fat_save_all (also in the monitor role)
// Before a planned USB detach (restart, ROM download mode, update, the power-off after "stm iaphold"):
// the drive reports "no medium" and stays so, and the PC stops its reads. A PC that still read the
// drive when the board left the bus lost USB ports until its own restart (Windows, 2026-10-12).
// Wait about 500 ms after it before the detach.
void msc_fat_eject();
/**
 * @brief Saves the settings to NVS and to config.json on the USB drive (NVS first). A flash write stops the CPU for
 *        its time: save only while the vehicle stands still.
 * @param cfg the settings to save (normally &g_deviceConfig)
 * @return true when both saves passed, false when the config.json save failed
 */
bool msc_fat_save_all(const DeviceConfig* cfg);

/**
 * Single config keys by their config.json name, for config commands over ESP-NOW.
 * get: the value as a JSON literal; false for an unknown key.
 * set: 0 ok (checked as for a config.json edit), 1 unknown key, 4 read-only (a _eff or wifi_ key),
 *      2 bad value, refused with no change: not a JSON literal, the wrong type, not a choice, a text the vehicle
 *      name rules would change, or out of range. Out of range: *why = "Range <min> to <max>" (", or 0 = off" for a
 *      key where 0 = off). A set never takes a default; config.json and NVS do.
 *      The caller applies the result with applyDeviceConfig().
 * key_at: "key=value" for the index-th key; false past the last key.
 */
bool msc_fat_get_key(const DeviceConfig* cfg, const char* key, String* value_json);
int  msc_fat_set_key(DeviceConfig* cfg, const char* key, const char* value_json, String* why = nullptr);
bool msc_fat_key_at(const DeviceConfig* cfg, int index, String* line);
// Settings in the log: fn once per "list" line, in "list" order (cmd_key: 0 or "set" only). Returns the line count.
int msc_fat_each_line(const DeviceConfig* cfg, void (*fn)(void* ctx, int index, const char* key, const char* value),
                      void* ctx);
void msc_fat_saved_config(DeviceConfig* out);  // 0.6.15: the settings of the next start (defaults, then NVS)
bool msc_fat_key_pair_at(const DeviceConfig* active, const DeviceConfig* saved, int index, String* line);  // "list saved"

/**
 * One table line as one JSON object, for the settings page (GET /api/settings), from the table at run time:
 * {"k": key, "t": "string"|"int"|"float"|"bool"|"choice", "v": value, "d": default, "min", "max" (absent = no limit),
 *  "z": 1 (0 = off is also valid), "n": text length, "c": [choice names], "live": 1, "u", "g", "l", "h" (unit, group,
 *  label, help of the table line; absent when empty)}. Values as "get" gives them.
 * index: a table line, 0 up. 1 written; 0 a line with no key (F_LEGACY); -1 past the last line; -2 cap too small.
 */
int msc_fat_setting_json(const DeviceConfig* cfg, int index, char* out, size_t cap);

/**
 * Erase the settings (USB and CMD "config erase [wifi]"): the NVS settings namespace is erased, *cfg takes the
 * defaults and config.json is written with them. wifi: the Wi-Fi details too. keep_link: radio_mode,
 * espnow_enabled, telemetry_enabled and cmd_key keep their values (and go to NVS), so a command over a radio
 * link keeps that link. Writes flash once per part; the caller refuses it while the vehicle drives.
 * false: the config.json write failed (NVS is erased all the same).
 */
bool msc_fat_erase(DeviceConfig* cfg, bool wifi, bool keep_link);

/**
 * A value that took its default (config.json or NVS; a set is refused instead): one line of text per value,
 * oldest first, for the USB port and the log. Source: 0 config.json, 1 NVS, 3 save. false: none waiting.
 */
bool msc_fat_bad_value_pop(char* line, size_t cap, int* source);

/**
 * A setting that config.json changed (a USB drive edit, or at the start a file that differs from NVS): one line of
 * text per key, "config.json: <key> <old> -> <new>", oldest first, for the USB port and the log. false: none waiting.
 */
bool msc_fat_change_pop(char* line, size_t cap);

#ifdef SETTINGS_SELFTEST
/** Self-check build: the old settings code against the table code (USB "cfgselftest"). Returns the difference count. */
int msc_fat_selftest(void (*print)(const char* line));
#endif

/**
 * Wi-Fi network for radio_mode "wifi" (USB "wifi set" / "wifi clear"). NVS only, in its own
 * namespace: never in config.json, never in DeviceConfig. "list" and "get" show wifi_ssid and
 * wifi_password = "set" or "not set"; this file never reads the password except in
 * msc_fat_wifi_password(), for the Wi-Fi start.
 * set: ssid 1..32 bytes; pass "" = an open network (no password key in NVS). false: NVS write failed.
 */
bool msc_fat_wifi_set(const char* ssid, const char* pass);
void msc_fat_wifi_clear();
const char* msc_fat_wifi_ssid();          // "" = none stored
bool msc_fat_wifi_has_password();
void msc_fat_wifi_password(char* out, size_t cap);  // "" = none
