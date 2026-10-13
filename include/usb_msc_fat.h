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
    uint8_t vehicleID;
    uint8_t rx_channel;

    // Drive parameters
    int CRUISE_THROTTLE;
    int CREEP_THROTTLE;

    // Output 1 – front steering
    int PWM_OUT_1_DEFAULT;
    int PWM_OUT_1_MIN;
    int PWM_OUT_1_MAX;

    // Output 2 – throttle
    int PWM_OUT_2_DEFAULT;
    int PWM_OUT_2_MIN;
    int PWM_OUT_2_MAX;

    // Output 3 – auxiliary
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
    int    CHARGE_MAX_SECONDS;
    uint8_t battery_cells;       // series cells (2 = 2S); the station scales its voltage graphs from it

    // Features
    bool   espnow_enabled;
    bool   telemetry_enabled;  // binary telemetry log over ESP-NOW (starts ESP-NOW by itself)
    int    telemetry_bins_hz;  // 8-bin coil record rate: 0 (off), 100, 50, 25, 20, 10, 5, 4, 2 or 1 (same filter)
    int    telemetry_rate_hz;  // log sample rate: 100, 50, 25, 20, 10, 5, 4, 2 or 1 (anti-alias filter below 100)

    // Rover autopilot (lib/rover_autopilot): replaces wire following in mode 2 when enabled
    bool    rover_enabled;
    bool    rover_goal_cw;           // dock direction along the wire, seen from above
    double  rover_sensor_height_mm;  // sensor centre above the wire
    double  rover_full_speed_mps;    // ground speed at full throttle command
    double  rover_full_yaw_rate_dps; // pivot rate at full steering command
    double  rover_seek_vbatt;        // V: go home below this battery voltage (0 = off)
    int     rover_seek_after_s;      // s since the last charge before going home (0 = off)
    uint8_t rover_entry_btx;         // BTx channel before the dock: creep (0 = none)
    uint8_t rover_exit_btx;          // BTx channel after the dock: wrong-way guard (0 = none)
    int     rover_lost_timeout_s;    // stop for good after this long without the wire (0 = never)
    double  rover_speed_limit_mps;   // m/s, cap on every autopilot speed (0 = the autopilot's own speeds)
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
    bool    usb_drive;               // true (default): config.json as a USB drive; false: no drive, settings by command only
                                     // (from the next start). A PC that read the drive when the board left USB lost ports.
};

// The shortest frame period setting: the fastest STM32 period (2336 us, set-up S2N) plus 50 %, rounded up.
// The ESP32 then has margin to serve the radio, the log and USB between frames.
#define DSP_FRAME_PERIOD_MIN_US 3600

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

/**
 * Persist the provided configuration to both NVS and config.json.
 *
 * Returns true when both saves succeed, false if JSON save fails.
 * NVS save is always attempted first.
 */
void msc_fat_media_service();  // the medium back after msc_fat_save_all (also in the monitor role)
// Before a planned USB detach (restart, ROM download mode, update, the power-off after "stm iaphold"):
// the drive reports "no medium" and stays so, and the PC stops its reads. A PC that still read the
// drive when the board left the bus lost USB ports until its own restart (Windows, 2026-10-12).
// Wait about 500 ms after it before the detach.
void msc_fat_eject();
bool msc_fat_save_all(const DeviceConfig* cfg);

/**
 * Single config keys by their config.json name, for config commands over ESP-NOW.
 * get: the value as a JSON literal; false for an unknown key.
 * set: 0 ok (validated and clamped as for a config.json edit), 1 unknown key,
 *      2 bad value (not a JSON literal, or the wrong type for the key), 4 read-only (a _eff key).
 *      The caller applies the result with applyDeviceConfig().
 * key_at: "key=value" for the index-th key; false past the last key.
 */
bool msc_fat_get_key(const DeviceConfig* cfg, const char* key, String* value_json);
int  msc_fat_set_key(DeviceConfig* cfg, const char* key, const char* value_json);
bool msc_fat_key_at(const DeviceConfig* cfg, int index, String* line);
