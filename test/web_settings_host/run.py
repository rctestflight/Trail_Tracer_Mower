"""Desk check of the settings page code on the PC: the JSON of GET /api/settings and the set path of POST /api/set.

It cuts the code out of the firmware sources as it is (no copy by hand): the settings table code of src/usb_msc_fat.cpp
(msc_fat_setting_json, set, get, list), the body parser of src/web_settings.cpp (set_text) and the command code of
src/main.cpp (configCommandRun, stationaryKey, the update period floor). Shims stand in for the hardware parts.
Build: MSVC and ArduinoJson from the PlatformIO library folder, as test/settings_host.

Checks:
  1. JSON: every table key once, in table order, with the type, default, range and choices of its table line
     (tools/gen_settings_def.py reads the table); the value is the default; the key list is the "list" key list
     without the read-only keys (_eff, wifi_).
  2. Set path, for every key: a valid value that is not the default, as page text, and as a JSON number or
     true/false where the type allows it. The page command text must equal the USB text; the page answer (code, text,
     settings after) must equal the radio command answer; against USB, a difference is allowed only for a key that
     would cut the radio link.
  3. Out of range, for every key with a limit: the page answer equals the USB answer, and nothing changes.
  4. Bad bodies: refused by the parser.

Usage (repository root, Windows, Visual Studio Build Tools):
    python test/web_settings_host/run.py [--newkey]
Exit code: 0 all pass, 1 a failure, 2 a build error. Last line: RESULT {json}.
"""
import glob
import json
import math
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import gen_settings_def  # noqa: E402

LINK_KEYS = {'radio_mode', 'espnow_enabled', 'telemetry_enabled', 'cmd_key'}  # radio rules (src/main.cpp)


def cut(text, start, end):
    a = text.index(start)
    return text[a:text.index(end, a)]


SHIMS = r'''
int8_t drive_mode_status = 0;
bool safe_mode = false;
static bool g_stationary = true;
bool vehicleStationary() { return g_stationary; }
struct UsbOutShim { void drain(int) {} void printf(const char*, ...) {} } usbOut;
bool msc_fat_save_all(const DeviceConfig*) { return true; }
bool msc_fat_erase(DeviceConfig* c, bool, bool) { populate_defaults(*c); settings_check(*c, SRC_SAVE); return true; }
void applyDeviceConfig(const DeviceConfig&) {}
uint8_t stm_link_ver = 0;  // no Rx STM32: a new update period answers at once
int32_t sweep_sent_bins = 0;
uint32_t stm_ans_print_until = 0;
#define WFRX_BINS 8
struct StmLinkShim { void pushInt(uint32_t, int32_t, uint8_t) {} void pushFloat(uint32_t, float, uint8_t) {} } stmLink;
struct StmDecShim { int wireBin(int) { return -1; } } stmDec;
int bin_poll_idx = -1; float bin_poll_hz = 0; uint32_t bin_poll_t0 = 0, bin_poll_last = 0;
bool period_ack_want = false; long period_ack_asked = 0; uint32_t period_ack_answers0 = 0, dsp_period_answers = 0;
DeviceConfig g_deviceConfig;
'''

MAIN = r'''
static DeviceConfig defaults() {
    static DeviceConfig d;
    static bool done = false;
    if (!done) { populate_defaults(d); settings_check(d, SRC_SAVE); done = true; }
    return d;
}
// The page runs in Wi-Fi mode: the set path starts from the defaults with radio_mode wifi.
static DeviceConfig page_start() { DeviceConfig d = defaults(); d.radio_mode = RADIO_MODE_WIFI; return d; }
static String cfg_json(const DeviceConfig& c) { JsonDocument doc; config_to_doc(c, doc); String s; serializeJson(doc, s); return s; }
static void esc(const String& s) { for (const char* p = s.c_str(); *p; p++) putchar(*p == '\t' || *p == '\n' ? ' ' : *p); }

int main(int argc, char** argv) {
    if (argc > 1 && !strcmp(argv[1], "json")) {
        DeviceConfig c = defaults();
        char buf[1024];
        int n = 0;
        for (int i = 0;; i++) {
            const int r = msc_fat_setting_json(&c, i, buf, sizeof(buf));
            if (r == -1) break;
            if (r == -2) printf("TOOSMALL %d\n", i);
            else if (r == 0) printf("LEGACY %d\n", i);
            else { printf("JSON %s\n", buf); n++; }
        }
        String line;
        int k = 0;
        for (; msc_fat_key_at(&c, k, &line); k++) printf("LIST %s\n", line.c_str());
        printf("END json=%d list=%d\n", n, k);
        return 0;
    }
    // plan lines on stdin: <body>\t<usb command text>
    radio_wifi = true;  // the page runs in Wi-Fi mode, over the Wi-Fi link
    char in[1024];
    while (fgets(in, sizeof(in), stdin)) {
        in[strcspn(in, "\r\n")] = 0;
        char* tab = strchr(in, '\t');
        if (!tab) continue;
        *tab = 0;
        const char* body = in;
        const char* usb = tab + 1;
        char text[112], key[48];
        const bool ok = set_text(body, text, sizeof(text), key, sizeof(key));
        String r1, r2, r3, v1, v2, v3;
        int c1 = -1, c2, c3;
        String j1, j2, j3;
        if (ok) {
            g_deviceConfig = page_start();
            c1 = configCommandRun(text, true, r1);
            msc_fat_get_key(&g_deviceConfig, key, &v1);
            j1 = cfg_json(g_deviceConfig);
        }
        g_deviceConfig = page_start();
        c3 = configCommandRun(usb, true, r3);
        j3 = cfg_json(g_deviceConfig);
        g_deviceConfig = page_start();
        c2 = configCommandRun(usb, false, r2);
        j2 = cfg_json(g_deviceConfig);
        const String j0 = cfg_json(page_start());
        printf("CASE %d\t", ok ? 1 : 0); esc(ok ? String(text) : String("")); printf("\t%d\t", c1); esc(r1); printf("\t"); esc(v1);
        printf("\t%d\t", c3); esc(r3); printf("\t%d\t", c2); esc(r2);
        printf("\t%d\t%d\t%d\t%d\n", ok && j1 == j3, ok && j1 == j2, ok && j1 == j0, j2 == j0);
    }
    return 0;
}
'''


def build(newkey):
    fat = open(os.path.join(ROOT, 'src', 'usb_msc_fat.cpp'), encoding='utf-8').read().replace('\r\n', '\n')
    web = open(os.path.join(ROOT, 'src', 'web_settings.cpp'), encoding='utf-8').read().replace('\r\n', '\n')
    mn = open(os.path.join(ROOT, 'src', 'main.cpp'), encoding='utf-8').read().replace('\r\n', '\n')
    parts = [
        cut(fat, '// Settings table (include/settings_table.h)', '// Flash block device'),
        cut(fat, 'double g_dsp_eff[DSP_EFF_N]', 'static bool config_write_json('),
        'static bool s_rate_config_missing = false;\n',
        cut(fat, '// One value for one table key', '// Config file change detection helpers'),
        cut(fat, 'bool msc_fat_get_key(', '// Erase (USB and CMD'),
        cut(web, '// The text of a set from the body', 'esp_err_t set_post('),
        SHIMS,
        cut(mn, 'bool radio_wifi = false;', 'bool espnowWanted(){'),
        cut(mn, '// One config command, "get <key>", "set <key> <value>" or "save", from ESP-NOW', '// Run commands (docs/control_api.md'),
        cut(mn, 'uint8_t configCommandRun(const char* t, bool over_espnow, String& reply){', '// Answer a PING and send THRU frames'),
    ]
    tu = ['// Generated by test/web_settings_host/run.py from the firmware sources: do not edit',
          '#include <Arduino.h>', '#include "decimator.h"', '#include "usb_msc_fat.h"', '#include <ArduinoJson.h>',
          '#include <math.h>', '#include <stddef.h>', '#include <type_traits>', '#include <utility>', '#include "nvs.h"',
          'static const char* TAG = "msc_fat";', '#define NVS_NAMESPACE "wfrx_cfg"',
          '#define PWM_ABS_MIN 800', '#define PWM_ABS_MAX 2200',
          'const char* msc_fat_wifi_ssid() { return ""; }', 'bool msc_fat_wifi_has_password() { return false; }',
          'static uint32_t s_ms = 0; uint32_t millis() { return s_ms += 1; }',
          ] + parts + [MAIN, '']
    os.makedirs(BUILD, exist_ok=True)
    tu_path = os.path.join(BUILD, 'web_tu.cpp')
    open(tu_path, 'w', encoding='utf-8').write('\n'.join(tu))
    vswhere = r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    vs = subprocess.run([vswhere, '-latest', '-products', '*', '-property', 'installationPath'],
                        capture_output=True, text=True).stdout.strip()
    vcvars = os.path.join(vs, 'VC', 'Auxiliary', 'Build', 'vcvars64.bat')
    ajson = glob.glob(os.path.join(ROOT, '.pio', 'libdeps', '*', 'ArduinoJson', 'src'))[0]
    incs = [os.path.join(ROOT, 'test', 'settings_host', 'shim'), os.path.join(ROOT, 'include'), os.path.join(ROOT, 'src'),
            os.path.join(ROOT, 'lib', 'rover_telemetry'), ajson]
    defs = ['ARDUINOJSON_ENABLE_ARDUINO_STRING=1', 'ARDUINOJSON_ENABLE_STD_STRING=0', 'ARDUINOJSON_ENABLE_STD_STREAM=0']
    if newkey:
        defs.append('SETTINGS_NEWKEY')
    exe = os.path.join(BUILD, 'web_test.exe')
    cl = ('cl /nologo /std:c++17 /EHsc /O1 /W3 /wd4244 /wd4267 /wd4996 /wd4305 '
          + ' '.join(f'/I"{i}"' for i in incs) + ' ' + ' '.join(f'/D{d}' for d in defs)
          + f' /Fo"{BUILD}\\\\" /Fe"{exe}" "{tu_path}" "{os.path.join(ROOT, "lib", "rover_telemetry", "decimator.cpp")}"')
    bat = os.path.join(BUILD, 'build.bat')
    open(bat, 'w').write(f'@call "{vcvars}" >nul\r\n{cl}\r\n')
    r = subprocess.run(['cmd', '/c', bat], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout[-6000:], r.stderr[-3000:])
        return None
    return exe


def same(a, b):
    if isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, (int, float)) or not isinstance(b, (int, float)):
        return a == b
    return math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9)


FIXED = {'steer_output': '4', 'throttle_output': '3', 'steer_min_us': '950', 'steer_center_us': '1450',
         'steer_max_us': '2050', 'throttle_min_us': '1050', 'throttle_center_us': '1550', 'throttle_max_us': '1950',
         'telemetry_rate_hz': '50', 'telemetry_bins_hz': '25', 'espnow_tx_power_dbm': '10.75',
         'CHARGE_MAX_SECONDS': '1000', 'dsp_frame_period_us': '12000', 'cmd_key': '4242',
         'vehicle_name': 'Desk Check_2', 'radio_mode': 'espnow'}


def value_for(s, i):
    """A valid value that is not the default, as the text after 'set <key> '."""
    k, t, d = s['k'], s['t'], s['d']
    if k in FIXED:
        return FIXED[k]
    if k.startswith('dsp_bin_hz_'):
        return str(47000 + 1100 * int(k[-1]) + 0.5)
    if t == 'bool':
        return 'false' if d else 'true'
    if t == 'choice':
        return next(c for c in s['c'] if c != d)
    if t == 'string':
        return 'Desk Check'
    lo, hi = s.get('min'), s.get('max')
    if lo is not None and hi is not None:
        v = lo + (hi - lo) * (0.1 + 0.8 * ((i * 37) % 97) / 97.0)
    else:
        v = (lo if lo is not None else 0) + 1.25 + 0.01 * i
    v = int(round(v)) if t == 'int' else round(v, 3)
    if same(v, d):
        v = v + 1 if t == 'int' else round(v + 0.125, 3)
    return str(v)


def out_of_range(s):
    t, lo, hi = s['t'], s.get('min'), s.get('max')
    if t not in ('int', 'float'):
        return None
    if hi is not None:
        return str(int(hi) + 1000 if t == 'int' else hi * 2 + 1)
    if lo is not None and lo > 0:
        return str(int(lo) - 1 if t == 'int' else lo / 2) if not s.get('z') or lo / 2 != 0 else None
    return None


def main():
    newkey = '--newkey' in sys.argv
    exe = build(newkey)
    if not exe:
        print('RESULT ' + json.dumps({'build': 'failed'}))
        return 2
    fails = []
    res = {'newkey': newkey}

    # 1. JSON
    out = subprocess.run([exe, 'json'], capture_output=True, text=True).stdout.splitlines()
    objs = [json.loads(ln[5:]) for ln in out if ln.startswith('JSON ')]
    listed = [ln[5:].split('=', 1)[0] for ln in out if ln.startswith('LIST ')]
    toosmall = [ln for ln in out if ln.startswith('TOOSMALL')]
    defines = {'SETTINGS_NEWKEY'} if newkey else set()
    table = [e for e in gen_settings_def.parse_table(defines) if 'F_LEGACY' not in e['flags']]
    names = [o['k'] for o in objs]
    want_names = [e['name'] for e in table]
    if names != want_names:
        fails.append(f'json key order: {names} vs table {want_names}')
    list_settings = [k for k in listed if not k.endswith('_eff') and not k.startswith('wifi_')]
    if names != list_settings:
        fails.append('json keys differ from the list keys')
    if toosmall:
        fails.append(f'items too large: {toosmall}')
    by = {o['k']: o for o in objs}
    field_checks = 0
    for e in table:
        o = by.get(e['name'])
        if not o:
            continue
        t = gen_settings_def.TYPES[e['type']]
        checks = [('t', o['t'], t), ('d', o['d'], gen_settings_def.json_default(e)), ('v', o['v'], o['d'])]
        if t in ('int', 'float'):
            checks += [('min', o.get('min'), e['min']), ('max', o.get('max'), e['max']),
                       ('z', o.get('z', 0), 1 if 'F_ZERO_OFF' in e['flags'] else 0)]
        if t == 'choice':
            checks.append(('c', o.get('c'), [n for n, _ in gen_settings_def.choices(e['opts'])]))
        if t == 'string':
            checks.append(('n', o.get('n'), int(e['max'])))
        for col, src in (('u', 'unit'), ('g', 'group'), ('l', 'label'), ('h', 'help')):
            checks.append((col, o.get(col, ''), e[src]))
        checks.append(('live', o.get('live', 0), 1 if 'F_LIVE' in e['flags'] else 0))
        for col, got, want in checks:
            field_checks += 1
            if isinstance(want, float) and isinstance(got, (int, float)) and not isinstance(got, bool):
                ok = same(got, want)
            elif isinstance(want, float) and isinstance(got, str):
                ok = False
            else:
                ok = got == want or (isinstance(got, (int, float)) and isinstance(want, (int, float)) and same(got, want))
            if not ok:
                fails.append(f'json {e["name"]}.{col}: {got!r}, table {want!r}')
    sizes = [len(json.dumps(o, separators=(',', ':'))) for o in objs]
    res['json'] = {'keys': len(objs), 'table_keys': len(want_names), 'list_keys': len(list_settings),
                   'field_checks': field_checks, 'bytes': sum(sizes) + len(objs) + 120, 'largest_item': max(sizes)}
    print(f"JSON: {len(objs)} keys (table {len(want_names)}, list {len(list_settings)}), {field_checks} field checks, "
          f"about {res['json']['bytes']} B, largest item {max(sizes)} B")
    if newkey:
        o = by.get('test_new_setting', {})
        print(f"NEWKEY in JSON: {json.dumps(o)}")
        res['newkey_item'] = o

    # 2-4. set path
    plan = []
    for i, s in enumerate(objs):
        v = value_for(s, i)
        plan.append(('valid', s['k'], json.dumps({'key': s['k'], 'value': v}), f"set {s['k']} {v}"))
        if s['t'] in ('int', 'float', 'bool'):
            native = json.loads(v)
            plan.append(('native', s['k'], json.dumps({'key': s['k'], 'value': native}), f"set {s['k']} {v}"))
        bad = out_of_range(s)
        if bad is not None:
            plan.append(('range', s['k'], json.dumps({'key': s['k'], 'value': bad}), f"set {s['k']} {bad}"))
    bodies = ['{"value": 3}', '{"key": "kp 1", "value": 3}', '{"key": "kp", "value": "3\\nsave"}',
              '{"key": "kp", "value": {"a": 1}}', '{"key": "kp", "value": null}', 'not json',
              '{"key": "' + 'k' * 60 + '", "value": 1}', '{"key": "vehicle_name", "value": "' + 'x' * 80 + '"}']
    for b in bodies:
        plan.append(('badbody', '', b, 'set kp 3.5'))
    stdin = ''.join(f'{p[2]}\t{p[3]}\n' for p in plan)
    lines = [ln for ln in subprocess.run([exe, 'plan'], input=stdin, capture_output=True, text=True).stdout.splitlines()
             if ln.startswith('CASE ')]
    if len(lines) != len(plan):
        fails.append(f'plan: {len(plan)} cases, {len(lines)} answers')
    counts = {'valid': 0, 'native': 0, 'range': 0, 'badbody': 0}
    passed = dict(counts)
    usb_diff = []
    for p, ln in zip(plan, lines):
        kind, key, body, usb = p
        f = ln[5:].split('\t')
        ok_body, text, c1, r1, v1, c3, r3, c2, r2, eq_radio, eq_usb, unchanged_page, unchanged_usb = f
        counts[kind] += 1
        good = True
        if kind == 'badbody':
            good = ok_body == '0'
        else:
            good = ok_body == '1' and text == usb and c1 == c3 and r1 == r3 and eq_radio == '1'
            if kind == 'valid' or kind == 'native':
                if c1 != '0' and key not in LINK_KEYS:
                    good = False
                if (c1 != c2 or r1 != r2 or eq_usb != '1'):
                    usb_diff.append(f'{key}: page {c1} {r1!r}, USB {c2} {r2!r}')
                    if key not in LINK_KEYS:
                        good = False
            if kind == 'range':
                good = good and c1 == '2' and c1 == c2 and r1 == r2 and unchanged_page == '1' and unchanged_usb == '1'
        passed[kind] += good
        if not good:
            fails.append(f'{kind} {key} {body}: {ln}')
    res['set'] = {k: f'{passed[k]}/{counts[k]}' for k in counts}
    res['usb_differences'] = usb_diff
    for k in counts:
        print(f'SET {k}: {passed[k]} of {counts[k]} pass')
    for d in usb_diff:
        print(f'PAGE vs USB (radio rules): {d}')
    for fl in fails:
        print(f'FAIL {fl}')
    res['fails'] = len(fails)
    print('RESULT ' + json.dumps(res))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
