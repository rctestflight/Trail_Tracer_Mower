// Self-check build only (env settings_selftest, -D SETTINGS_SELFTEST), included at the end of src/usb_msc_fat.cpp.
// USB "cfgselftest": the settings code of 0.5.111 (old_*, src/settings_selftest_old.inc) against the table code,
// field by field. No flash write: NVS goes to a RAM store. One line per difference, then a count.
//   DIFF    the table code differs from 0.5.111 where it must not (a fault)
//   CHANGE  an out-of-range value: 0.5.111 clamped it, the table code takes the default (config.json, NVS) or
//           refuses it (set). Expected; counted apart
//   INFO    a fact for the report (not a fault)

#include "settings_selftest_old.inc"
#include "soc/rtc_wdt.h"
#include <stdarg.h>

// RAM store with NVS rules: one entry per key, of one type; a blob holds its length. It answers both the
// Preferences calls of the old code and the Store calls of the table code.
class RamPrefs {
  public:
    enum { MAXE = 120, KB = 9 };
    struct Ent { char key[16]; uint8_t kind, len; uint8_t data[34]; };
    Ent* e;
    int n = 0;
    RamPrefs() : e(new Ent[MAXE]) {}
    ~RamPrefs() { delete[] e; }
    void copy_from(const RamPrefs& o) { n = o.n; memcpy(e, o.e, sizeof(Ent) * o.n); }
    Ent* find(const char* k) { for (int i = 0; i < n; i++) if (!strcmp(e[i].key, k)) return &e[i]; return nullptr; }
    void raw(const char* k, uint8_t kind, const void* v, size_t len)
    {
        Ent* x = find(k);
        if (!x) { if (n >= MAXE) return; x = &e[n++]; snprintf(x->key, sizeof(x->key), "%s", k); }
        x->kind = kind;
        x->len = (uint8_t)len;
        memcpy(x->data, v, len);
    }
    void remove(const char* k) { Ent* x = find(k); if (x) { *x = e[n - 1]; n--; } }
    // Preferences calls (the old code)
    size_t putUChar(const char* k, uint8_t v) { raw(k, K_U8, &v, 1); return 1; }
    size_t putBool(const char* k, bool v) { return putUChar(k, v ? 1 : 0); }
    size_t putInt(const char* k, int32_t v) { raw(k, K_I32, &v, 4); return 4; }
    size_t putLong(const char* k, int32_t v) { return putInt(k, v); }
    size_t putFloat(const char* k, float v) { raw(k, KB, &v, 4); return 4; }
    size_t putDouble(const char* k, double v) { raw(k, KB, &v, 8); return 8; }
    size_t putString(const char* k, const char* v) { raw(k, K_STR, v, strlen(v) + 1); return strlen(v); }
    bool isKey(const char* k) { return find(k) != nullptr; }
    uint8_t getUChar(const char* k, uint8_t d) { Ent* x = find(k); return x && x->kind == K_U8 ? x->data[0] : d; }
    bool getBool(const char* k, bool d) { return getUChar(k, d ? 1 : 0) == 1; }
    int32_t getInt(const char* k, int32_t d) { Ent* x = find(k); if (x && x->kind == K_I32) memcpy(&d, x->data, 4); return d; }
    int32_t getLong(const char* k, int32_t d) { return getInt(k, d); }
    float getFloat(const char* k, float d) { Ent* x = find(k); if (x && x->kind == KB && x->len <= 4) memcpy(&d, x->data, x->len); return d; }
    double getDouble(const char* k, double d) { Ent* x = find(k); if (x && x->kind == KB && x->len <= 8) memcpy(&d, x->data, x->len); return d; }
    String getString(const char* k, String d) { Ent* x = find(k); return x && x->kind == K_STR ? String((const char*)x->data) : d; }
    // Store calls (the table code), as NvsStore
    int get(const char* k, uint8_t kind, void* out, size_t cap)
    {
        Ent* x = find(k);
        if (!x) return 1;
        const uint8_t want = (kind == K_B4 || kind == K_B8) ? KB : kind;
        if (x->kind != want) return 2;
        if (want == KB) { if (x->len > cap) return 2; memcpy(out, x->data, x->len); return x->len == cap ? 0 : 2; }
        if (want == K_STR) { if (x->len > cap) return 2; memcpy(out, x->data, x->len); return 0; }
        memcpy(out, x->data, kind == K_U8 ? 1 : 4);
        return 0;
    }
    void put(const char* k, uint8_t kind, const void* v, size_t len)
    {
        if (kind == K_STR) raw(k, K_STR, v, strlen((const char*)v) + 1);
        else if (kind == K_B4 || kind == K_B8) raw(k, KB, v, len);
        else raw(k, kind, v, kind == K_U8 ? 1 : 4);
    }
};

static void (*st_print)(const char*);
static int st_cmp, st_diff, st_change, st_info;

static void st_line(const char* fmt, ...)
{
    char b[240];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    st_print(b);
}
static void st_yield() { rtc_wdt_feed(); feedLoopWDT(); vTaskDelay(1); }  // the RTC watchdog (8 s), the loop task watchdog (5 s, bench 2026-10-16: the check reset at 5 s), the idle task

static size_t field_size(const SettingDef& e)
{
    switch (e.type) {
        case T_TEXT: return sizeof(((DeviceConfig*)nullptr)->vehicle_name);
        case T_U8: case T_BOOL: case T_CHOICE: return 1;
        case T_F64: return 8;
        default: return 4;
    }
}
static bool field_eq(const SettingDef& e, const DeviceConfig& a, const DeviceConfig& b)
{
    if (e.type == T_TEXT) return !strcmp((const char*)fptr(a, e), (const char*)fptr(b, e));
    return !memcmp(fptr(a, e), fptr(b, e), field_size(e));
}

// Every table field of a against b. Returns the count of fields that differ; each is a DIFF line.
static int st_compare(const char* what, const DeviceConfig& a, const DeviceConfig& b)
{
    int d = 0;
    for (const SettingDef& e : SETTINGS) {
        st_cmp++;
        if (field_eq(e, a, b)) continue;
        d++;
        char va[40], vb[40];
        field_text(a, e, va, sizeof(va));
        field_text(b, e, vb, sizeof(vb));
        st_line("DIFF %s %s: old=%s new=%s", what, e.name, va, vb);
    }
    st_diff += d;
    return d;
}

// The config.json text (eff false) or the get and list keys (eff true) of both codes, byte for byte.
static void st_compare_doc(const char* what, const DeviceConfig& o, const DeviceConfig& n, bool eff)
{
    String s1, s2;
    {
        JsonDocument d1, d2;
        old_config_to_doc(o, d1, eff);
        config_to_doc(n, d2, eff);
        serializeJsonPretty(d1, s1);
        serializeJsonPretty(d2, s2);
    }
    st_cmp++;
    if (s1 == s2) return;
    st_diff++;
    unsigned i = 0;
    while (i < s1.length() && i < s2.length() && s1[i] == s2[i]) i++;
    const unsigned a = i > 30 ? i - 30 : 0;
    st_line("DIFF %s %s text at byte %u: old ...%s... new ...%s...", what, eff ? "list" : "config.json", i,
            s1.substring(a, i + 30).c_str(), s2.substring(a, i + 30).c_str());
}

static void old_defaults(DeviceConfig& c) { memset(&c, 0, sizeof(c)); old_populate_defaults(c); old_validate_config(c); }
static void new_defaults(DeviceConfig& c) { memset(&c, 0, sizeof(c)); populate_defaults(c); settings_check(c, SRC_SAVE); }
static void old_from_text(DeviceConfig& c, const String& t) { old_defaults(c); old_config_from_json_text(c, t); old_validate_config(c); }
static void new_from_text(DeviceConfig& c, const String& t) { new_defaults(c); config_from_json_text(c, t); settings_check(c, SRC_JSON); }

static String st_num(double v, bool is_int)
{
    char b[32];
    if (is_int) snprintf(b, sizeof(b), "%ld", (long)v);
    else snprintf(b, sizeof(b), "%.10g", v);
    return String(b);
}

// A valid value that is not the default, as a JSON literal; i makes each key's value its own, so a swap of two
// keys (field, NVS key, JSON name) shows, and a float value that a float cannot hold exactly shows a float or double
// mix-up in the JSON text.
static String st_other_value(const SettingDef& e, int i)
{
    switch (e.type) {
        case T_TEXT: return "\"Self Test_1\"";
        case T_BOOL: case T_IBOOL: return e.def != 0 ? "false" : "true";
        case T_CHOICE: {
            for (int v = 0; v <= 255; v++) {
                size_t n;
                const char* s = choice_name(e.opts, v, &n);
                if (s && v != (int)e.def) return "\"" + String(s).substring(0, n) + "\"";
            }
            return "null";
        }
        case T_F32: case T_F64: {
            double v;
            if (!isnan(e.lo) && !isnan(e.hi)) v = e.lo + (e.hi - e.lo) * (0.1 + 0.8 * ((i * 37) % 97) / 97.0);
            else v = (isnan(e.lo) ? e.def : e.lo) + 0.3 + 0.01 * i;
            if (v == e.def) v += (isnan(e.hi) ? 0.5 : -(e.hi - e.lo) * 0.01);
            return st_num(v, false);
        }
        default: {
            if (isnan(e.lo) || isnan(e.hi)) return st_num((isnan(e.lo) ? e.def : e.lo) + 1 + i, true);
            const long lo = (long)e.lo, hi = (long)e.hi, span = hi - lo;
            if (span <= 2) return st_num(e.def != e.hi ? e.hi : e.lo, true);
            long v = lo + 1 + (i * 7) % (span - 1);
            if (v == (long)e.def) v = v + 1 < hi ? v + 1 : lo + 1;
            return st_num(v, true);
        }
    }
}

struct StCfgs { DeviceConfig o, n, x, a, b, c, d; };

// The old code kept a number literal as it is (a value it took as valid)
static bool st_kept(const SettingDef& e, const DeviceConfig& o, const String& lit)
{
    if (e.type == T_TEXT || e.type == T_BOOL || e.type == T_IBOOL || e.type == T_CHOICE) return false;
    char* end = nullptr;
    const double v = strtod(lit.c_str(), &end);
    if (end == lit.c_str() || *end) return false;
    const double f = num_get(o, e);
    return e.type == T_F32 ? (float)v == (float)f : v == f;
}

// One value of one key: through config.json and through set, old code against table code.
static void st_value(StCfgs& k, int i, const String& lit, bool valid)
{
    const SettingDef& e = SETTINGS[i];
    char what[72];
    snprintf(what, sizeof(what), "%s=%s", e.name, lit.c_str());
    const String text = String("{\"") + e.name + "\":" + lit + "}";
    // config.json
    old_from_text(k.o, text);
    const int bad0 = s_bad_count;
    new_from_text(k.n, text);
    if (valid) {
        st_compare(what, k.o, k.n);
        st_cmp++;
        if (s_bad_count != bad0) { st_diff++; st_line("DIFF %s json: a valid value took the default", what); }
    } else {
        new_defaults(k.d);
        st_compare(what, k.d, k.n);  // the table code: the default
        st_cmp++;
        if (s_bad_count == bad0) { st_diff++; st_line("DIFF %s json: no bad-value note", what); }
        st_cmp++;
        if (st_kept(e, k.o, lit)) { st_diff++; st_line("DIFF %s json: 0.5.111 kept this value; the table range is narrower", what); }
        if (!field_eq(e, k.o, k.n)) {
            char vo[40], vn[40];
            field_text(k.o, e, vo, sizeof(vo));
            field_text(k.n, e, vn, sizeof(vn));
            st_change++;
            st_line("CHANGE %s json: 0.5.111=%s table=%s (default)", what, vo, vn);
        }
    }
    if (e.flags & F_LEGACY) return;  // config.json only
    // set
    old_defaults(k.o);
    const int ro = old_set_key(&k.o, e.name, lit.c_str());
    new_defaults(k.n);
    String why;
    const int rn = msc_fat_set_key(&k.n, e.name, lit.c_str(), &why);
    if (valid) {
        st_cmp++;
        if (ro != rn) { st_diff++; st_line("DIFF %s set: answer old=%d new=%d", what, ro, rn); }
        st_compare(what, k.o, k.n);
        // the get answer (and the set reply) of this key
        JsonDocument d1, d2;
        old_config_to_doc(k.o, d1, true);
        config_to_doc(k.n, d2, true);
        String g1, g2;
        serializeJson(d1[e.name], g1);
        serializeJson(d2[e.name], g2);
        st_cmp++;
        if (g1 != g2) { st_diff++; st_line("DIFF %s get: old=%s new=%s", what, g1.c_str(), g2.c_str()); }
    } else {
        new_defaults(k.d);
        st_cmp++;
        if (rn != 2) { st_diff++; st_line("DIFF %s set: answer %d, not 2 (refused)", what, rn); }
        st_cmp++;
        if (ro == 0 && st_kept(e, k.o, lit)) { st_diff++; st_line("DIFF %s set: 0.5.111 kept this value; the table range is narrower", what); }
        st_compare(what, k.d, k.n);  // refused: no change
        if (ro != rn || !field_eq(e, k.o, k.n)) {
            char vo[40];
            field_text(k.o, e, vo, sizeof(vo));
            st_change++;
            st_line("CHANGE %s set: 0.5.111 answer %d value %s; table answer %d %s", what, ro, vo, rn,
                    why.length() ? why.c_str() : "(refused)");
        }
    }
}

int msc_fat_selftest(void (*print)(const char* line))
{
    st_print = print;
    st_cmp = st_diff = st_change = st_info = 0;
    const bool quiet0 = s_bad_quiet;
    s_bad_quiet = true;  // no "SETTING bad value" lines from the test values
    StCfgs* kp = (StCfgs*)calloc(1, sizeof(StCfgs));
    RamPrefs* ro = new RamPrefs;
    RamPrefs* rn = new RamPrefs;
    if (!kp || !ro || !rn) { st_line("no memory"); free(kp); delete ro; delete rn; s_bad_quiet = quiet0; return -1; }
    StCfgs& k = *kp;
    const uint32_t t0 = millis();

    // 0. The table itself: one field each, no double name or NVS key, the bytes it covers
    {
        size_t bytes = 0;
        int legacy = 0;
        for (int i = 0; i < N_SETTINGS; i++) {
            bytes += field_size(SETTINGS[i]);
            legacy += (SETTINGS[i].flags & F_LEGACY) ? 1 : 0;
            for (int j = 0; j < i; j++) {
                st_cmp++;
                if (!strcmp(SETTINGS[i].name, SETTINGS[j].name) || !strcmp(SETTINGS[i].nvs, SETTINGS[j].nvs)
                    || SETTINGS[i].off == SETTINGS[j].off) {
                    st_diff++;
                    st_line("DIFF table: %s and %s share a name, an NVS key or a field", SETTINGS[j].name, SETTINGS[i].name);
                }
            }
        }
        st_info++;
        st_line("INFO table lines=%d (settings %d, config.json-only %d) field bytes=%u of DeviceConfig %u (the rest is padding)",
                N_SETTINGS, N_SETTINGS - legacy, legacy, (unsigned)bytes, (unsigned)sizeof(DeviceConfig));
    }

    // 1. Defaults
    old_defaults(k.o);
    new_defaults(k.n);
    st_compare("defaults", k.o, k.n);
    st_compare_doc("defaults", k.o, k.n, false);
    st_compare_doc("defaults", k.o, k.n, true);
    {   // before the range check (never seen outside: every path checks before use)
        memset(&k.a, 0, sizeof(k.a));
        memset(&k.b, 0, sizeof(k.b));
        old_populate_defaults(k.a);
        populate_defaults(k.b);
        for (const SettingDef& e : SETTINGS)
            if (!field_eq(e, k.a, k.b)) {
                char va[40], vb[40];
                field_text(k.a, e, va, sizeof(va));
                field_text(k.b, e, vb, sizeof(vb));
                st_info++;
                st_line("INFO default before the range check %s: old=%s new=%s (both %s after it)", e.name, va, vb, vb);
            }
    }
    st_yield();

    // 2. A config.json with every key at a valid value that is not its default
    String text = "{";
    for (const SettingDef& e : SETTINGS) {
        if (e.flags & F_LEGACY) continue;
        String v = st_other_value(e, (int)(&e - SETTINGS));
        // values that the special rules leave where they are: two outputs apart, the servo centre between its
        // limits, the log rates on their grid, the radio power on its 0.25 dB grid
        static const char* const FIXED[][2] = {
            {"steer_output", "4"}, {"throttle_output", "3"},
            {"steer_min_us", "950"}, {"steer_center_us", "1450"}, {"steer_max_us", "2050"},
            {"throttle_min_us", "1050"}, {"throttle_center_us", "1550"}, {"throttle_max_us", "1950"},
            {"telemetry_rate_hz", "50"}, {"telemetry_bins_hz", "25"}, {"espnow_tx_power_dbm", "10.75"},
            {"CHARGE_MAX_SECONDS", "1000"}};
        for (auto& f : FIXED) if (!strcmp(e.name, f[0])) v = f[1];
        if (text.length() > 1) text += ",";
        text += String("\"") + e.name + "\":" + v;
    }
    text += "}";
    {
        old_from_text(k.o, text);
        const int bad0 = s_bad_count;
        new_from_text(k.n, text);
        st_cmp++;
        if (s_bad_count != bad0) { st_diff++; st_line("DIFF all-keys json: %d valid values took the default", s_bad_count - bad0); }
        st_compare("all-keys json", k.o, k.n);
        st_compare_doc("all-keys", k.o, k.n, false);
        st_compare_doc("all-keys", k.o, k.n, true);
        new_defaults(k.d);
        int moved = 0, total = 0;
        for (const SettingDef& e : SETTINGS) {
            if (e.flags & F_LEGACY) continue;
            total++;
            if (!field_eq(e, k.n, k.d)) moved++;
            else st_line("INFO all-keys json: %s stays at its default", e.name);
        }
        st_info++;
        st_line("INFO all-keys json: %d of %d keys away from the default (%u bytes of config.json)", moved, total,
                (unsigned)text.length());
        k.x = k.o;  // the source of the NVS round trip
    }
    // 2b. A config.json from 0.5.68 or older: the old servo names, the half charge time, the old beacon key
    {
        const String legacy = "{\"PWM_OUT_1_DEFAULT\":1400,\"PWM_OUT_1_MIN\":900,\"PWM_OUT_1_MAX\":2100,"
                              "\"PWM_OUT_2_DEFAULT\":1450,\"PWM_OUT_3_DEFAULT\":1300,\"PWM_OUT_4_MAX\":1900,"
                              "\"CHARGE_MAX_SECONDS\":200,\"BTxThreshold\":100,\"BTxFollowThreshold\":300}";
        old_from_text(k.o, legacy);
        new_from_text(k.n, legacy);
        st_compare("legacy json", k.o, k.n);
        st_compare_doc("legacy", k.o, k.n, false);
    }
    st_yield();

    // 3. NVS: save then load, in RAM
    {
        old_prefs_save(*ro, k.x);
        prefs_save_to(*rn, k.x);
        st_cmp++;
        if (ro->n != rn->n) { st_diff++; st_line("DIFF nvs save: old %d keys, new %d keys", ro->n, rn->n); }
        for (int i = 0; i < ro->n; i++) {
            const RamPrefs::Ent& a = ro->e[i];
            RamPrefs::Ent* b = rn->find(a.key);
            st_cmp++;
            if (!b || b->kind != a.kind || b->len != a.len || memcmp(a.data, b->data, a.len)) {
                st_diff++;
                st_line("DIFF nvs save %s: %s", a.key, !b ? "missing" : b->kind != a.kind ? "another type" : "another value");
            }
        }
        st_info++;
        st_line("INFO nvs keys written: old %d, new %d", ro->n, rn->n);
        // load: old code from the old image; table code from the old image (a board updated from 0.5.111) and its own
        old_defaults(k.a); old_populate_defaults(k.a); old_prefs_load(*ro, k.a); old_validate_config(k.a);
        new_defaults(k.b); populate_defaults(k.b); prefs_load_from(*ro, k.b); settings_check(k.b, SRC_NVS);
        new_defaults(k.c); populate_defaults(k.c); prefs_load_from(*rn, k.c); settings_check(k.c, SRC_NVS);
        st_compare("nvs round trip (old save, old load = source)", k.x, k.a);
        st_compare("nvs old image: old load / table load", k.a, k.b);
        st_compare("nvs table image: old load / table load", k.a, k.c);
        // an image from 0.5.68 or older: no drvOut (CAN drive), no chgReal (half charge time)
        RamPrefs* r2 = new RamPrefs;
        r2->copy_from(*ro);
        r2->remove("drvOut");
        r2->remove("chgReal");
        old_defaults(k.a); old_populate_defaults(k.a); old_prefs_load(*r2, k.a); old_validate_config(k.a);
        new_defaults(k.b); populate_defaults(k.b); prefs_load_from(*r2, k.b); settings_check(k.b, SRC_NVS);
        st_compare("nvs 0.5.68 image", k.a, k.b);
        // odd raw values: bools stored as 2
        r2->copy_from(*ro);
        r2->putUChar("enS2", 2);
        r2->putUChar("rvEn", 2);
        old_defaults(k.a); old_populate_defaults(k.a); old_prefs_load(*r2, k.a); old_validate_config(k.a);
        new_defaults(k.b); populate_defaults(k.b); prefs_load_from(*r2, k.b); settings_check(k.b, SRC_NVS);
        st_compare("nvs bool stored as 2", k.a, k.b);
        // an empty namespace (first start)
        r2->n = 0;
        old_defaults(k.a); old_populate_defaults(k.a);
        const bool la = old_prefs_load(*r2, k.a);
        old_validate_config(k.a);
        new_defaults(k.b); populate_defaults(k.b);
        const bool lb = prefs_load_from(*r2, k.b);
        settings_check(k.b, SRC_NVS);
        st_cmp++;
        if (la != lb) { st_diff++; st_line("DIFF nvs empty: load answer old=%d new=%d", la, lb); }
        st_compare("nvs empty", k.a, k.b);
        delete r2;
    }
    st_yield();

    // 4. Each key at its min and max, one step outside, 0 for a key where 0 = off; each choice; both bools
    for (int i = 0; i < N_SETTINGS; i++) {
        const SettingDef& e = SETTINGS[i];
        const bool is_int = e.type == T_U8 || e.type == T_I32 || e.type == T_LONG;
        switch (e.type) {
            case T_TEXT:
                st_value(k, i, "\"Abc-1_2 x\"", true);
                st_value(k, i, "\"ABCDEFGHIJKLMNOP\"", true);
                st_value(k, i, "5.5", false);  // a number; not a bare word for set either
                break;
            case T_BOOL: case T_IBOOL:
                st_value(k, i, "true", true);
                st_value(k, i, "false", true);
                st_value(k, i, "1", false);
                break;
            case T_CHOICE:
                for (int v = 0; v <= 255; v++) {
                    size_t n;
                    const char* s = choice_name(e.opts, v, &n);
                    if (!s) continue;
                    st_value(k, i, "\"" + String(s).substring(0, n) + "\"", true);
                    if (e.flags & F_CHOICE_INT) st_value(k, i, String(v), true);
                }
                st_value(k, i, "\"zz\"", false);
                if (e.flags & F_CHOICE_INT) st_value(k, i, "9", false);
                break;
            default: {
                if (!isnan(e.lo)) {
                    const double step = is_int ? 1 : fmax(1e-3, fabs(e.lo) * 1e-3);
                    st_value(k, i, st_num(e.lo, is_int), true);
                    st_value(k, i, st_num(e.lo - step, is_int), false);
                }
                if (!isnan(e.hi)) {
                    const double step = is_int ? 1 : fmax(1e-3, fabs(e.hi) * 1e-3);
                    st_value(k, i, st_num(e.hi, is_int), true);
                    st_value(k, i, st_num(e.hi + step, is_int), false);
                }
                st_value(k, i, "0", in_range(e, 0));  // a key where 0 = off, and a lost "0 = off"
                if (!isnan(e.lo) && !isnan(e.hi)) {     // an odd value inside the range (grid rules)
                    const double odd = e.lo + (e.hi - e.lo) * 0.3137;
                    st_value(k, i, st_num(is_int ? floor(odd) : odd, is_int), true);
                }
                if (isnan(e.lo) && isnan(e.hi)) st_value(k, i, st_num(e.def + 1, is_int), true);
                if (is_int) st_value(k, i, "1.5", false);  // the wrong type
                break;
            }
        }
        st_yield();
    }

    st_line("end comparisons=%d differences=%d changes=%d info=%d ms=%lu", st_cmp, st_diff, st_change, st_info,
            (unsigned long)(millis() - t0));
    free(kp);
    delete ro;
    delete rn;
    s_bad_quiet = quiet0;
    return st_diff;
}
