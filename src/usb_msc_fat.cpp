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
#include <stddef.h>
#include <type_traits>
#include <utility>

#include "nvs.h"

#include "esp_partition.h"
#include "esp_mac.h"
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
#define NVS_WIFI_NAMESPACE        "wfrx_wifi"      // Wi-Fi network: "ssid", "pass" (not in config.json)
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
// The one file object for config.json (read and write; loop() only, never two at once). With
// CONFIG_FATFS_PER_FILE_CACHE a FIL holds a 4 KB sector buffer. On the stack it took config_write_json
// to 4288 B; under "save" from the USB command path (loop, benchSerial, configCommand, msc_fat_save_all)
// that overflowed the 8 KB loop() stack: a panic restart on 4 of 8 USB saves, 36 B free after a save that passed.
static FIL s_fil;
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
// Settings table (include/settings_table.h): one line per setting drives the defaults, the range check,
// NVS save and load, config.json write and read, and set, get and list.
// ─────────────────────────────────────────────────────────────────────────────
enum : uint8_t { T_TEXT, T_U8, T_I32, T_IBOOL, T_BOOL, T_F32, T_F64, T_LONG, T_CHOICE };
enum : uint8_t { F_ZERO_OFF = 1, F_LEGACY = 2, F_CHOICE_INT = 4, F_LIVE = 8 };
#define NOLIM NAN  // no limit on that side

struct SettingDef {
    const char* name;   // config.json key
    const char* opts;   // T_CHOICE: "name|name=value"; T_TEXT: the default text
    const char* nvs;    // NVS key
    double   def, lo, hi;
    uint16_t off;       // offset of the DeviceConfig field
    uint8_t  type, flags;
};

static const SettingDef SETTINGS[] = {
#define S(name, field, type, def, lo, hi, opts, nvs, flags, ...) \
    { #name, opts, nvs, (double)(def), (double)(lo), (double)(hi), (uint16_t)offsetof(DeviceConfig, field), type, flags },
#define SF(name, ctype, type, def, lo, hi, opts, nvs, flags, ...) \
    { #name, opts, nvs, (double)(def), (double)(lo), (double)(hi), (uint16_t)offsetof(DeviceConfig, name), type, flags },
#include "settings_table.h"
#undef S
#undef SF
};
static constexpr int N_SETTINGS = sizeof(SETTINGS) / sizeof(SETTINGS[0]);

// The words of each line (unit, group, label, help), for the settings page. Empty words: the page takes them from its
// copy of docs/settings_def.json.
struct SettingWords { const char *unit, *group, *label, *help; };
static const SettingWords SETTING_WORDS[] = {
#define S(name, field, type, def, lo, hi, opts, nvs, flags, unit, group, label, help) { unit, group, label, help },
#define SF(name, ctype, type, def, lo, hi, opts, nvs, flags, unit, group, label, help) { unit, group, label, help },
#include "settings_table.h"
#undef S
#undef SF
};
static_assert(sizeof(SETTING_WORDS) / sizeof(SETTING_WORDS[0]) == N_SETTINGS, "one word entry per table line");

// Compile-time checks: each line's type matches its DeviceConfig field, and each NVS key fits (15 characters).
template <uint8_t T> struct SettingCType;
template <> struct SettingCType<T_TEXT>   { typedef char type[17]; };
template <> struct SettingCType<T_U8>     { typedef uint8_t type; };
template <> struct SettingCType<T_I32>    { typedef int type; };
template <> struct SettingCType<T_IBOOL>  { typedef int type; };
template <> struct SettingCType<T_BOOL>   { typedef bool type; };
template <> struct SettingCType<T_F32>    { typedef float type; };
template <> struct SettingCType<T_F64>    { typedef double type; };
template <> struct SettingCType<T_LONG>   { typedef long type; };
template <> struct SettingCType<T_CHOICE> { typedef uint8_t type; };
#define SETTING_CHECKS(name, fieldexpr, ty, nvs)                                                              \
    static_assert(std::is_same<std::remove_reference<decltype(std::declval<DeviceConfig&>().fieldexpr)>::type, \
                               SettingCType<ty>::type>::value, #name ": the table type differs from the field"); \
    static_assert(sizeof(nvs) - 1 <= 15, #name ": NVS key longer than 15 characters");
#define S(name, field, ty, def, lo, hi, opts, nvs, ...) SETTING_CHECKS(name, field, ty, nvs)
#define SF(name, ctype, ty, def, lo, hi, opts, nvs, ...) SETTING_CHECKS(name, name, ty, nvs)
#include "settings_table.h"
#undef S
#undef SF
#undef SETTING_CHECKS

enum : uint8_t { SRC_JSON, SRC_NVS, SRC_SET, SRC_SAVE };

static inline void* fptr(DeviceConfig& c, const SettingDef& e) { return (uint8_t*)&c + e.off; }
static inline const void* fptr(const DeviceConfig& c, const SettingDef& e) { return (const uint8_t*)&c + e.off; }

static int find_setting(const char* name)
{
    for (int i = 0; i < N_SETTINGS; i++) if (!strcmp(SETTINGS[i].name, name)) return i;
    return -1;
}

// T_CHOICE: the name of a stored value (not 0-terminated: *len), or nullptr; the stored value of a name, or -1.
static const char* choice_name(const char* opts, int v, size_t* len)
{
    int pos = 0;
    for (const char* s = opts; *s; pos++) {
        const char* end = strchr(s, '|');
        if (!end) end = s + strlen(s);
        const char* eq = (const char*)memchr(s, '=', end - s);
        if ((eq ? atoi(eq + 1) : pos) == v) { *len = (eq ? eq : end) - s; return s; }
        s = *end ? end + 1 : end;
    }
    return nullptr;
}
static int choice_value(const char* opts, const char* name)
{
    int pos = 0;
    const size_t n = strlen(name);
    for (const char* s = opts; *s; pos++) {
        const char* end = strchr(s, '|');
        if (!end) end = s + strlen(s);
        const char* eq = (const char*)memchr(s, '=', end - s);
        const size_t len = (eq ? eq : end) - s;
        if (len == n && !strncmp(s, name, n)) return eq ? atoi(eq + 1) : pos;
        s = *end ? end + 1 : end;
    }
    return -1;
}

static double num_get(const DeviceConfig& c, const SettingDef& e)
{
    const void* p = fptr(c, e);
    switch (e.type) {
        case T_U8: case T_CHOICE: return *(const uint8_t*)p;
        case T_I32: case T_IBOOL: return *(const int*)p;
        case T_BOOL:              return *(const bool*)p;
        case T_F32:               return *(const float*)p;
        case T_F64:               return *(const double*)p;
        case T_LONG:              return *(const long*)p;
        default:                  return 0;
    }
}
static void num_set(DeviceConfig& c, const SettingDef& e, double v)
{
    void* p = fptr(c, e);
    switch (e.type) {
        case T_U8: case T_CHOICE: *(uint8_t*)p = (uint8_t)v; break;
        case T_I32: case T_IBOOL: *(int*)p = (int)v; break;
        case T_BOOL:              *(bool*)p = v != 0; break;
        case T_F32:               *(float*)p = (float)v; break;
        case T_F64:               *(double*)p = v; break;
        case T_LONG:              *(long*)p = (long)v; break;
        default: break;
    }
}
static void set_default(DeviceConfig& c, const SettingDef& e)
{
    if (e.type == T_TEXT) strcpy((char*)fptr(c, e), e.opts);
    else num_set(c, e, e.def);
}
static bool in_range(const SettingDef& e, double v)
{
    if (!isfinite(v)) return false;
    if (e.type == T_CHOICE) { size_t n; return v >= 0 && v <= 255 && choice_name(e.opts, (int)v, &n); }
    if ((e.flags & F_ZERO_OFF) && v == 0.0) return true;
    if (!isnan(e.lo) && v < e.lo) return false;
    if (!isnan(e.hi) && v > e.hi) return false;
    return true;
}

// The value of one key into a JSON variant, with the same C type as before the table (the same JSON text).
static void put_value(JsonVariant v, const SettingDef& e, const DeviceConfig& c)
{
    const void* p = fptr(c, e);
    switch (e.type) {
        case T_TEXT:  v.set((const char*)p); break;
        case T_U8:    v.set(*(const uint8_t*)p); break;
        case T_I32:   v.set(*(const int*)p); break;
        case T_IBOOL: v.set((bool)*(const int*)p); break;  // false = NORMAL, true = REVERSE
        case T_BOOL:  v.set(*(const bool*)p); break;
        case T_F32:   v.set(*(const float*)p); break;
        case T_F64:   v.set(*(const double*)p); break;
        case T_LONG:  v.set(*(const long*)p); break;
        case T_CHOICE: {
            size_t n;
            const char* s = choice_name(e.opts, *(const uint8_t*)p, &n);
            if (s) v.set(JsonString(s, n));  // a copy: a static JsonString is read up to its 0, not its size
            else v.set(*(const uint8_t*)p);
            break;
        }
    }
}

// Values that took their default, for the USB port and the log (msc_fat_bad_value_pop). Oldest dropped when full.
struct BadNote { uint8_t idx, src; bool clamped; double lim; char val[26]; };  // clamped: set to limit lim, not the default
static BadNote s_bad[8];
static uint8_t s_bad_head = 0, s_bad_n = 0;
static bool    s_bad_quiet = false;  // a trial set on a copy: no note
static int     s_bad_count = 0;      // every default taken since the start (the self-check reads it)

static void note_bad(DeviceConfig& c, int idx, const char* val, uint8_t src)
{
    set_default(c, SETTINGS[idx]);
    s_bad_count++;
    if (s_bad_quiet) return;
    if (s_bad_n == 8) { s_bad_head = (s_bad_head + 1) % 8; s_bad_n--; }
    BadNote& b = s_bad[(s_bad_head + s_bad_n) % 8];
    s_bad_n++;
    b.idx = (uint8_t)idx;
    b.src = src;
    b.clamped = false;
    snprintf(b.val, sizeof(b.val), "%s", val);
}

// 0.6.26: a number out of range takes the nearest limit (a user who asks for more gets the most the setting allows),
// with a note; a value of the wrong type still takes the default.
static void note_clamped(DeviceConfig& c, int idx, const char* val, uint8_t src, double lim)
{
    num_set(c, SETTINGS[idx], lim);
    if (s_bad_quiet) return;
    if (s_bad_n == 8) { s_bad_head = (s_bad_head + 1) % 8; s_bad_n--; }
    BadNote& b = s_bad[(s_bad_head + s_bad_n) % 8];
    s_bad_n++;
    b.idx = (uint8_t)idx;
    b.src = src;
    b.clamped = true;
    b.lim = lim;
    snprintf(b.val, sizeof(b.val), "%s", val);
}

bool msc_fat_bad_value_pop(char* line, size_t cap, int* source)
{
    if (!s_bad_n) return false;
    const BadNote b = s_bad[s_bad_head];
    s_bad_head = (s_bad_head + 1) % 8;
    s_bad_n--;
    const SettingDef& e = SETTINGS[b.idx];
    JsonDocument d;  // the default in the form of "get"
    DeviceConfig* tmp = (DeviceConfig*)calloc(1, sizeof(DeviceConfig));
    char def[40] = "?";
    if (tmp) {
        if (b.clamped) num_set(*tmp, e, b.lim);  // the limit now in force
        else set_default(*tmp, e);
        put_value(d.to<JsonVariant>(), e, *tmp);
        serializeJson(d, def, sizeof(def));
        free(tmp);
    }
    static const char* const SRC[] = {"config.json", "NVS", "set", "save"};
    snprintf(line, cap, "%s=%s from %s: %s %s", e.name, b.val, SRC[b.src & 3], b.clamped ? "limit" : "default", def);
    *source = b.src;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: Defaults
// ─────────────────────────────────────────────────────────────────────────────
static void populate_defaults(DeviceConfig& c)
{
    for (const SettingDef& e : SETTINGS) set_default(c, e);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: range check, then the special rules
// ─────────────────────────────────────────────────────────────────────────────
static inline int clamp_pwm(int v) {
    if (v < PWM_ABS_MIN) v = PWM_ABS_MIN;
    if (v > PWM_ABS_MAX) v = PWM_ABS_MAX;
    return v;
}

// The rules a min and max cannot hold (as up to 0.5.111). They move a value; they never take a default.
static void settings_special(DeviceConfig& c)
{
    c.telemetry_rate_hz = rec::rateNearest(c.telemetry_rate_hz);  // 100/N Hz only
    c.telemetry_bins_hz = c.telemetry_bins_hz <= 0 ? 0 : rec::rateNearest(c.telemetry_bins_hz);  // 0 = off
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
    if (c.throttle_output == c.steer_output) c.throttle_output = c.steer_output == 2 ? 1 : 2;
    // Servo outputs: MIN not above MAX, and the centre between them
    auto fix_out = [](int& d, int& lo, int& hi) {
        lo = clamp_pwm(lo);
        hi = clamp_pwm(hi);
        if (lo > hi) {
            lo = clamp_pwm(hi - 100);
            hi = clamp_pwm(lo + 100);
        }
        d = constrain(d, lo, hi);
    };
    fix_out(c.PWM_OUT_1_DEFAULT, c.PWM_OUT_1_MIN, c.PWM_OUT_1_MAX);
    fix_out(c.PWM_OUT_2_DEFAULT, c.PWM_OUT_2_MIN, c.PWM_OUT_2_MAX);
    fix_out(c.PWM_OUT_3_DEFAULT, c.PWM_OUT_3_MIN, c.PWM_OUT_3_MAX);
    fix_out(c.PWM_OUT_4_DEFAULT, c.PWM_OUT_4_MIN, c.PWM_OUT_4_MAX);
    c.espnow_tx_power_dbm = round(c.espnow_tx_power_dbm * 4.0) / 4.0;  // 0.25 dB steps
}

// The field of one key as text, for a note.
static void field_text(const DeviceConfig& c, const SettingDef& e, char* out, size_t cap)
{
    if (e.type == T_CHOICE) { snprintf(out, cap, "%u", (unsigned)*(const uint8_t*)fptr(c, e)); return; }
    JsonDocument d;
    put_value(d.to<JsonVariant>(), e, c);
    serializeJson(d, out, cap);
}

// Each key in its range (else its default, with a note), then the special rules.
static void settings_check(DeviceConfig& c, uint8_t src)
{
    for (int i = 0; i < N_SETTINGS; i++) {
        const SettingDef& e = SETTINGS[i];
        if (e.type == T_TEXT || e.type == T_BOOL) continue;
        if (in_range(e, num_get(c, e))) continue;
        char txt[26];
        field_text(c, e, txt, sizeof(txt));
        note_bad(c, i, txt, src);
    }
    settings_special(c);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: NVS. The same key and the same NVS type per key as up to 0.5.111 (Preferences: putBool = u8,
// putInt and putLong = i32, putFloat and putDouble = blob, putString = string).
// ─────────────────────────────────────────────────────────────────────────────
enum : uint8_t { K_U8, K_I32, K_STR, K_B4, K_B8 };
static uint8_t nvs_kind(uint8_t type)
{
    switch (type) {
        case T_TEXT: return K_STR;
        case T_U8: case T_BOOL: case T_CHOICE: return K_U8;
        case T_I32: case T_IBOOL: case T_LONG: return K_I32;
        case T_F32: return K_B4;
        default: return K_B8;
    }
}

struct NvsStore {
    nvs_handle_t h = 0;
    bool ok = false, rw = false;
    bool open(bool write)
    {
        rw = write;
        ok = nvs_open(NVS_NAMESPACE, write ? NVS_READWRITE : NVS_READONLY, &h) == ESP_OK;
        return ok;
    }
    void close() { if (ok) { if (rw) nvs_commit(h); nvs_close(h); } ok = false; }
    bool exists(const char* key)
    {
        uint64_t t;
        size_t len = 0;
        return nvs_get_i8(h, key, (int8_t*)&t) == ESP_OK || nvs_get_u8(h, key, (uint8_t*)&t) == ESP_OK
            || nvs_get_i16(h, key, (int16_t*)&t) == ESP_OK || nvs_get_u16(h, key, (uint16_t*)&t) == ESP_OK
            || nvs_get_i32(h, key, (int32_t*)&t) == ESP_OK || nvs_get_u32(h, key, (uint32_t*)&t) == ESP_OK
            || nvs_get_i64(h, key, (int64_t*)&t) == ESP_OK || nvs_get_u64(h, key, &t) == ESP_OK
            || nvs_get_str(h, key, nullptr, &len) == ESP_OK || nvs_get_blob(h, key, nullptr, &len) == ESP_OK;
    }
    // 0 read, 1 not stored, 2 stored with another type or size
    int get(const char* key, uint8_t kind, void* out, size_t cap)
    {
        esp_err_t r;
        size_t n = cap;
        switch (kind) {
            case K_U8:  r = nvs_get_u8(h, key, (uint8_t*)out); break;
            case K_I32: r = nvs_get_i32(h, key, (int32_t*)out); break;
            case K_STR: r = nvs_get_str(h, key, (char*)out, &n); break;
            default:
                r = nvs_get_blob(h, key, out, &n);
                if (r == ESP_OK && n != cap) return 2;
                break;
        }
        if (r == ESP_OK) return 0;
        return exists(key) ? 2 : 1;
    }
    void put(const char* key, uint8_t kind, const void* v, size_t n)
    {
        switch (kind) {
            case K_U8:  nvs_set_u8(h, key, *(const uint8_t*)v); break;
            case K_I32: nvs_set_i32(h, key, *(const int32_t*)v); break;
            case K_STR: nvs_set_str(h, key, (const char*)v); break;
            default:    nvs_set_blob(h, key, v, n); break;
        }
    }
};

template <class Store>
static void prefs_save_to(Store& s, const DeviceConfig& c)
{
    for (const SettingDef& e : SETTINGS) {
        const void* p = fptr(c, e);
        const uint8_t k = nvs_kind(e.type);
        if (e.type == T_BOOL) { const uint8_t b = *(const bool*)p ? 1 : 0; s.put(e.nvs, K_U8, &b, 1); }
        else s.put(e.nvs, k, p, k == K_B4 ? 4 : k == K_B8 ? 8 : 0);
    }
    const uint8_t one = 1;
    s.put("chgReal", K_U8, &one, 1);  // CHARGE_MAX_SECONDS holds the real time (0.5.69)
}

template <class Store>
static bool prefs_load_from(Store& s, DeviceConfig& c)
{
    uint8_t b = 0;
    if (s.get("vID", K_U8, &b, 1) == 1) return false;  // the namespace is empty
    for (int i = 0; i < N_SETTINGS; i++) {
        const SettingDef& e = SETTINGS[i];
        union { uint8_t u8; int32_t i32; float f; double d; char s[33]; } v;
        const uint8_t k = nvs_kind(e.type);
        const int r = s.get(e.nvs, k, &v, k == K_STR ? sizeof(v.s) : k == K_B4 ? 4 : k == K_B8 ? 8 : k == K_U8 ? 1 : 4);
        if (r == 1) continue;  // not stored (a new setting): keeps its value
        if (r == 2) { note_bad(c, i, "(another NVS type)", SRC_NVS); continue; }
        void* p = fptr(c, e);
        switch (e.type) {
            case T_TEXT:  strncpy((char*)p, v.s, 16); ((char*)p)[16] = 0; break;
            case T_BOOL:  *(bool*)p = v.u8 == 1; break;  // as Preferences::getBool
            case T_U8: case T_CHOICE: *(uint8_t*)p = v.u8; break;
            case T_F32:   *(float*)p = v.f; break;
            case T_F64:   *(double*)p = v.d; break;
            default:      memcpy(p, &v.i32, 4); break;  // T_I32, T_IBOOL, T_LONG
        }
    }
    // A board updated from 0.5.68 or older drove its motor controllers over CAN: keep that, not auto.
    if (s.get("drvOut", K_U8, &b, 1) == 1) c.drive_output = 2;
    // Up to 0.5.68 the value was half the charge time (the clamp as up to 0.5.111)
    if (s.get("chgReal", K_U8, &b, 1) != 0 || b != 1) c.CHARGE_MAX_SECONDS = constrain(c.CHARGE_MAX_SECONDS * 2, 10, 3600);
    return true;
}

static void prefs_save(const DeviceConfig& c)
{
    NvsStore s;
    if (!s.open(true)) return;
    prefs_save_to(s, c);
    s.close();
}

static bool prefs_load(DeviceConfig& c)
{
    NvsStore s;
    if (!s.open(false)) return false;
    const bool r = prefs_load_from(s, c);
    s.close();
    return r;
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
// 0.6.9: one disk identity per board. FatFs f_mkfs leaves the MBR disk signature 0 and takes the volume serial from
// the clock (not set at the first boot), so every board formatted after an erase showed the PC the same disk.
// Windows logged "Disk N has the same disk identifiers" (System, disk event 158) with two boards on one PC, then a
// paging error on that drive and USB losses at the next detach (2026-10-19). Both values now come from
// the chip MAC. Written only when they differ: once per board (a one-off flash write), never again.
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t s_id_sec[STORAGE_SECTOR_SIZE];

static uint32_t le32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void put_le32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

// The volume serial at its boot sector offset (FAT12/16: 39, FAT32: 67); false: not a FAT boot sector.
static bool vbr_serial_offset(const uint8_t* b, unsigned* off) {
    if (b[510] != 0x55 || b[511] != 0xAA) return false;
    if (!memcmp(b + 82, "FAT32", 5)) { *off = 67; return true; }
    if (!memcmp(b + 54, "FAT", 3)) { *off = 39; return true; }
    return false;
}

static void drive_unique_ids() {
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return;
    uint32_t id = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    id ^= 0x54540000u;  // "TT": never 0 for a real MAC tail
    if (!id) id = 0x54540001u;
    if (diskio_read_cb(s_pdrv, s_id_sec, 0, 1) != RES_OK) return;
    unsigned off = 0;
    uint32_t vbr_lba = 0;
    if (!vbr_serial_offset(s_id_sec, &off)) {  // not a boot sector: an MBR with one partition
        if (s_id_sec[510] != 0x55 || s_id_sec[511] != 0xAA) return;
        vbr_lba = le32(s_id_sec + 0x1BE + 8);
        if (le32(s_id_sec + 440) != id) {
            put_le32(s_id_sec + 440, id);
            if (diskio_write_cb(s_pdrv, s_id_sec, 0, 1) != RES_OK) return;
            ESP_LOGI(TAG, "drive: disk signature %08lx", (unsigned long)id);
        }
        if (!vbr_lba || vbr_lba >= s_sector_count) return;
        if (diskio_read_cb(s_pdrv, s_id_sec, vbr_lba, 1) != RES_OK) return;
        if (!vbr_serial_offset(s_id_sec, &off)) return;
    }
    if (le32(s_id_sec + off) != id) {
        put_le32(s_id_sec + off, id);
        if (diskio_write_cb(s_pdrv, s_id_sec, vbr_lba, 1) != RES_OK) return;
        ESP_LOGI(TAG, "drive: volume serial %08lx", (unsigned long)id);
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
    BYTE* work = (BYTE*)malloc(FF_MAX_SS);  // 4 KB: not on the loop() stack
    if (!work) {
        ESP_LOGE(TAG, "f_mkfs work buffer: no memory");
        return false;
    }
    res = f_mkfs(FAT_DRIVE, FM_FAT, 0, work, FF_MAX_SS);
    free(work);
    if (res != FR_OK) {
        ESP_LOGE(TAG, "f_mkfs failed (%d)", res);
        return false;
    }
    drive_unique_ids();  // 0.6.9: before the mount, so FatFs reads the final boot sector
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
    for (const SettingDef& e : SETTINGS)
        if (!(e.flags & F_LEGACY)) put_value(doc[JsonString(e.name, true)].to<JsonVariant>(), e, c);
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

    FIL& fil = s_fil;  // not on the stack: a FIL holds a 4 KB sector buffer (see s_fil)
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
// Unknown keys are ignored; a key that is not there keeps its current (NVS-loaded) value; a value of the
// wrong type or out of range takes its default (with a note).
static bool s_rate_config_missing = false;

static bool config_read_json(DeviceConfig& c)
{
    FIL& fil = s_fil;  // not on the stack (see s_fil)
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

// One value for one table key (config.json, or a set). A number out of range takes the nearest limit, a value of the
// wrong type the key's default, each with a note. false: a bad value (default taken).
static bool json_read_one(DeviceConfig& c, int i, JsonVariantConst v, uint8_t src)
{
    const SettingDef& e = SETTINGS[i];
    void* p = fptr(c, e);
    bool num_ok = false;
    double num = 0;
    switch (e.type) {
        case T_TEXT:
            if (v.is<const char*>()) { strncpy((char*)p, v.as<const char*>(), 16); ((char*)p)[16] = 0; return true; }
            break;
        case T_CHOICE:
            if (v.is<const char*>()) {
                const int n = choice_value(e.opts, v.as<const char*>());
                if (n >= 0) { *(uint8_t*)p = (uint8_t)n; return true; }
            } else if ((e.flags & F_CHOICE_INT) && v.is<int>() && in_range(e, v.as<int>())) {
                *(uint8_t*)p = (uint8_t)v.as<int>();
                return true;
            }
            break;
        case T_BOOL: case T_IBOOL:
            if (v.is<bool>()) { num_set(c, e, v.as<bool>()); return true; }
            break;
        case T_U8: case T_I32:
            if (v.is<int>()) { num = v.as<int>(); num_ok = true; }
            break;
        case T_LONG:
            if (v.is<long>()) { num = v.as<long>(); num_ok = true; }
            break;
        default:  // T_F32, T_F64
            if (v.is<double>() && !v.is<bool>()) { num = v.as<double>(); num_ok = true; }
            break;
    }
    if (num_ok && in_range(e, num)) { num_set(c, e, num); return true; }
    char txt[26];
    serializeJson(v, txt, sizeof(txt));
    if (num_ok && isfinite(num) && e.type != T_CHOICE) {
        const double lim = (!isnan(e.lo) && num < e.lo) ? e.lo : e.hi;
        if (!isnan(lim)) { note_clamped(c, i, txt, src, lim); return true; }
    }
    note_bad(c, i, txt, src);
    return false;
}

// Apply the keys present in a JSON object to c (config.json content). A key that is not there keeps its value.
static bool config_from_json_text(DeviceConfig& c, const String& json_no_comments)
{
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json_no_comments);
    if (err) {
        ESP_LOGE(TAG, "JSON parse error: %s", err.c_str());
        return false;
    }
    // 0.5.69 names by function; an older config.json gives PWM_OUT_1_* (steering) and PWM_OUT_2_* (throttle).
    // The new names win. The six keys follow each other in the table from steer_center_us.
    static const char* const OLD_PWM[6] = {"PWM_OUT_1_DEFAULT", "PWM_OUT_1_MIN", "PWM_OUT_1_MAX",
                                           "PWM_OUT_2_DEFAULT", "PWM_OUT_2_MIN", "PWM_OUT_2_MAX"};
    const int steer_i = find_setting("steer_center_us");
    for (int k = 0; k < 6; k++) {
        JsonVariantConst v = doc[OLD_PWM[k]];
        if (!v.isNull()) json_read_one(c, steer_i + k, v, SRC_JSON);
    }
    for (int i = 0; i < N_SETTINGS; i++) {
        JsonVariantConst v = doc[SETTINGS[i].name];
        if (!v.isNull()) json_read_one(c, i, v, SRC_JSON);
    }
    {   // the old name of BTxThreshold wins over the new one (as up to 0.5.111)
        JsonVariantConst v = doc["BTxFollowThreshold"];
        if (!v.isNull()) json_read_one(c, find_setting("BTxThreshold"), v, SRC_JSON);
    }
    // A config.json from 0.5.68 or older (PWM_OUT_1_DEFAULT, no steer_center_us): CHARGE_MAX_SECONDS was half the charge
    // time; the file is written again in the 0.5.69 form after the read (s_rate_config_missing).
    const bool legacy_file = doc["PWM_OUT_1_DEFAULT"].is<int>() && doc["steer_center_us"].isNull();
    s_rate_config_missing = doc["steering_rate_kp"].isNull() || doc["steering_rate_ki"].isNull() || doc["steering_rate_kd"].isNull() || doc["steering_rate_pid_direction"].isNull() || doc["steering_rate_max_dps"].isNull() || legacy_file;
    if (legacy_file && doc["CHARGE_MAX_SECONDS"].is<int>())
        c.CHARGE_MAX_SECONDS = constrain(doc["CHARGE_MAX_SECONDS"].as<int>() * 2, 10, 3600);  // the clamp as up to 0.5.111
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
        settings_check(*cfg, SRC_NVS);
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
    settings_check(*cfg, SRC_NVS);

    // 5. Mount FAT, create config file on first boot, read it, unmount.
    drive_unique_ids();  // 0.6.9: an existing drive gets its board's disk identity (once)
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
                settings_check(file_cfg, SRC_JSON);
                // NVS only when config.json differs from it (a user edit): no NVS write at every boot.
                const bool changed = memcmp(&file_cfg, cfg, sizeof(file_cfg)) != 0;
                *cfg = file_cfg;
                if (changed) prefs_save(*cfg);
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
        settings_check(new_cfg, SRC_JSON);
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
    settings_check(tmp, SRC_SAVE);
    prefs_save(tmp);
}

// config.json from c, with the drive away from the PC during the write
static bool json_save(const DeviceConfig& c)
{
    // A PC that has the drive mounted keeps its own copy of the FAT. The drive shows "no medium"
    // during the write and for 2 s after it, so the PC drops that copy and reads the drive again.
    // After msc_fat_eject the medium stays away.
    s_msc.mediaPresent(false);
    s_media_back_ms = s_ejected ? 0 : (uint32_t)millis() + 2000;
    if (!fatfs_mount()) {
        ESP_LOGE(TAG, "FAT mount failed while saving config.json");
        return false;
    }

    bool ok = config_write_json(c);
    if (ok) {
        record_config_stamp();
    } else {
        ESP_LOGE(TAG, "Failed to write config file: %s", s_config_path);
    }
    fatfs_unmount();
    return ok;
}

bool msc_fat_save_all(const DeviceConfig* cfg)
{
    if (!cfg) return false;

    DeviceConfig tmp = *cfg;
    settings_check(tmp, SRC_SAVE);

    // Always persist to NVS first so settings survive reboot even if FAT write fails.
    prefs_save(tmp);
    return json_save(tmp);
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

// The range of a key as the answer to a set out of range
static String range_text(const SettingDef& e)
{
    auto num = [](double v) { char b[24]; snprintf(b, sizeof(b), "%.10g", v); return String(b); };
    String s = "Range ";
    if (isnan(e.hi)) s += num(e.lo) + " or more";
    else if (isnan(e.lo)) s += "up to " + num(e.hi);
    else s += num(e.lo) + " to " + num(e.hi);
    if (e.flags & F_ZERO_OFF) s += ", or 0 = off";
    return s;
}

int msc_fat_set_key(DeviceConfig* cfg, const char* key, const char* value_json, String* why)
{
    const size_t n = strlen(key);
    if (n > 4 && !strcmp(key + n - 4, "_eff")) return 4;  // read-only: the value in effect, from the STM32
    if (!strncmp(key, "wifi_", 5)) return 4;              // read-only: the USB command "wifi set" writes them
    const int i = find_setting(key);
    if (i < 0 || (SETTINGS[i].flags & F_LEGACY)) return 1;  // unknown key
    const SettingDef& e = SETTINGS[i];
    // A text key takes a JSON string, or a bare word (letters, digits, '_'), as "set radio_mode wifi".
    String quoted;
    if ((e.type == T_TEXT || e.type == T_CHOICE) && value_json[0] != '"') {
        for (const char* q = value_json; *q; q++)
            if (!isalnum((unsigned char)*q) && *q != '_') return 2;
        quoted = String("\"") + value_json + "\"";
        value_json = quoted.c_str();
    }
    JsonDocument val;
    if (deserializeJson(val, value_json)) return 2;  // not a JSON literal
    JsonVariantConst nv = val.as<JsonVariantConst>();
    DeviceConfig tmp = *cfg;
    const bool quiet0 = s_bad_quiet;
    s_bad_quiet = true;  // a set is refused, never takes a default: no note
    const bool good = json_read_one(tmp, i, nv, SRC_SET);
    if (good) settings_check(tmp, SRC_SET);
    s_bad_quiet = quiet0;
    if (!good) {  // refused, no change; a number of the right type is out of range
        const bool int_type = e.type == T_U8 || e.type == T_I32;
        const bool type_ok = (int_type && nv.is<int>()) || (e.type == T_LONG && nv.is<long>())
                          || ((e.type == T_F32 || e.type == T_F64) && nv.is<double>() && !nv.is<bool>());
        if (type_ok && why) *why = range_text(e);
        return 2;
    }
    // The vehicle name rules must leave the text as it is (as up to 0.5.111: refused, no change)
    if (e.type == T_TEXT && strcmp((const char*)fptr(tmp, e), nv.as<const char*>())) return 2;
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

// 0.6.15: the settings the board takes at its next start: the defaults, then the values stored in NVS.
void msc_fat_saved_config(DeviceConfig* out)
{
    populate_defaults(*out);
    prefs_load(*out);
}

// 0.6.15: "list saved": one line per key, "<key>=<active> saved=<saved>", both values as "list" writes them.
bool msc_fat_key_pair_at(const DeviceConfig* active, const DeviceConfig* saved, int index, String* line)
{
    String a, s;
    if (!msc_fat_key_at(active, index, &a) || !msc_fat_key_at(saved, index, &s)) return false;
    const int eq = s.indexOf('=');
    *line = a + " saved=" + (eq >= 0 ? s.substring(eq + 1) : s);
    return true;
}

int msc_fat_setting_json(const DeviceConfig* cfg, int index, char* out, size_t cap)
{
    if (index < 0 || index >= N_SETTINGS) return -1;
    const SettingDef& e = SETTINGS[index];
    if (e.flags & F_LEGACY) return 0;
    static const char* const TYPE_NAME[] = {"string", "int", "int", "bool", "bool", "float", "float", "int", "choice"};
    JsonDocument d;
    d["k"] = JsonString(e.name, true);  // static text: no copy
    d["t"] = JsonString(TYPE_NAME[e.type], true);
    put_value(d["v"].to<JsonVariant>(), e, *cfg);
    switch (e.type) {
        case T_TEXT:  d["d"] = JsonString(e.opts, true); d["n"] = (int)e.hi; break;
        case T_BOOL: case T_IBOOL: d["d"] = e.def != 0; break;
        case T_U8: case T_I32: case T_LONG: d["d"] = (long)e.def; break;
        case T_F32: d["d"] = (float)e.def; break;
        case T_F64: d["d"] = e.def; break;
        case T_CHOICE: {
            size_t n;
            const char* s = choice_name(e.opts, (int)e.def, &n);
            if (s) d["d"] = JsonString(s, n);
            JsonArray c = d["c"].to<JsonArray>();
            for (const char* p = e.opts; *p;) {
                const char* end = strchr(p, '|');
                if (!end) end = p + strlen(p);
                const char* eq = (const char*)memchr(p, '=', end - p);
                c.add(JsonString(p, (eq ? eq : end) - p));
                p = *end ? end + 1 : end;
            }
            break;
        }
    }
    if (e.type != T_TEXT && e.type != T_CHOICE && e.type != T_BOOL && e.type != T_IBOOL) {
        if (!isnan(e.lo)) d["min"] = e.lo;
        if (!isnan(e.hi)) d["max"] = e.hi;
        if (e.flags & F_ZERO_OFF) d["z"] = 1;
    }
    if (e.flags & F_LIVE) d["live"] = 1;
    const SettingWords& w = SETTING_WORDS[index];
    if (*w.unit) d["u"] = JsonString(w.unit, true);
    if (*w.group) d["g"] = JsonString(w.group, true);
    if (*w.label) d["l"] = JsonString(w.label, true);
    if (*w.help) d["h"] = JsonString(w.help, true);
    const size_t len = serializeJson(d, out, cap);
    return len + 1 < cap ? 1 : -2;
}

// Erase (USB and CMD "config erase [wifi]"; the caller refuses it while the vehicle drives)
bool msc_fat_erase(DeviceConfig* cfg, bool wifi, bool keep_link)
{
    const uint8_t radio_mode = cfg->radio_mode;
    const bool espnow_enabled = cfg->espnow_enabled, telemetry_enabled = cfg->telemetry_enabled;
    const long cmd_key = cfg->cmd_key;
    NvsStore s;
    if (s.open(true)) { nvs_erase_all(s.h); s.close(); }
    if (wifi) msc_fat_wifi_clear();
    populate_defaults(*cfg);
    settings_check(*cfg, SRC_SAVE);
    if (keep_link) {  // a command over a radio link keeps that link
        cfg->radio_mode        = radio_mode;
        cfg->espnow_enabled    = espnow_enabled;
        cfg->telemetry_enabled = telemetry_enabled;
        cfg->cmd_key           = cmd_key;
        prefs_save(*cfg);
    }
    return json_save(*cfg);  // config.json overrides NVS at the next start: it must hold the defaults too
}

// ─────────────────────────────────────────────────────────────────────────────
// Wi-Fi network (radio_mode "wifi"): NVS only. The SSID is cached for "list" and "get"; the
// password is read only by msc_fat_wifi_password(), for the Wi-Fi start.
// ─────────────────────────────────────────────────────────────────────────────
static char s_wifi_ssid[33] = "";
static bool s_wifi_pass = false;
static bool s_wifi_loaded = false;

static void wifi_load()
{
    if (s_wifi_loaded) return;
    s_wifi_loaded = true;
    Preferences p;
    if (!p.begin(NVS_WIFI_NAMESPACE, true)) return;  // namespace not made yet: nothing stored
    String ssid = p.getString("ssid", "");
    snprintf(s_wifi_ssid, sizeof(s_wifi_ssid), "%s", ssid.c_str());
    s_wifi_pass = p.isKey("pass");
    p.end();
}

bool msc_fat_wifi_set(const char* ssid, const char* pass)
{
    Preferences p;
    if (!p.begin(NVS_WIFI_NAMESPACE, false)) return false;
    bool ok = p.putString("ssid", ssid) == strlen(ssid);
    if (pass && pass[0]) ok = ok && p.putString("pass", pass) == strlen(pass);
    else if (p.isKey("pass")) p.remove("pass");  // an open network: no password stored
    p.end();
    s_wifi_loaded = false;
    wifi_load();
    return ok;
}

void msc_fat_wifi_clear()
{
    Preferences p;
    if (p.begin(NVS_WIFI_NAMESPACE, false)) {
        p.clear();
        p.end();
    }
    s_wifi_ssid[0] = 0;
    s_wifi_pass = false;
    s_wifi_loaded = true;
}

const char* msc_fat_wifi_ssid()
{
    wifi_load();
    return s_wifi_ssid;
}

bool msc_fat_wifi_has_password()
{
    wifi_load();
    return s_wifi_pass;
}

void msc_fat_wifi_password(char* out, size_t cap)
{
    if (!cap) return;
    out[0] = 0;
    Preferences p;
    if (!p.begin(NVS_WIFI_NAMESPACE, true)) return;
    if (p.isKey("pass")) p.getString("pass", out, cap);
    p.end();
}

#ifdef SETTINGS_SELFTEST
#include "settings_selftest.inc"
#endif
