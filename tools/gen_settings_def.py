"""Write docs/settings_def.json (the settings definition for the telemetry app) from the settings table,
include/settings_table.h.

From the table: the keys, their order of first appearance in the app file, type, default, and every word of a new
key (unit, group, label, help). First step (version 13): a key that the committed settings_def.json already has
keeps its words (label, description, group, unit, flags such as live or advanced) and its app range from that file;
the table gives its type and default, and the script stops if they differ from the file. Read-only keys (dsp_*_eff,
wifi_ssid, wifi_password) are not in the table and stay as they are.

Checks: no double name or NVS key, NVS keys of 15 characters at most, each default inside its range. A range of the
table is the app range (0.6.26: the app keeps no range of its own).

Usage (repository root):
    python tools/gen_settings_def.py                 write docs/settings_def.json
    python tools/gen_settings_def.py --check         exit 1 if docs/settings_def.json is not the generated text
    python tools/gen_settings_def.py -D SETTINGS_NEWKEY -o .pio/build/settings_newkey/settings_def.json
"""
import argparse
import copy
import json
import math
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
VERSION = 15
TYPES = {'T_TEXT': 'string', 'T_U8': 'int', 'T_I32': 'int', 'T_LONG': 'int', 'T_IBOOL': 'bool', 'T_BOOL': 'bool',
         'T_F32': 'float', 'T_F64': 'float', 'T_CHOICE': 'choice'}
COLS = ['name', 'field', 'type', 'default', 'min', 'max', 'opts', 'nvs', 'flags', 'unit', 'group', 'label', 'help']


def split_args(s):
    out, cur, depth, q = [], '', 0, False
    i = 0
    while i < len(s):
        c = s[i]
        if q:
            cur += c
            if c == '\\':
                cur += s[i + 1]
                i += 1
            elif c == '"':
                q = False
        elif c == '"':
            q = True
            cur += c
        elif c in '([':
            depth += 1
            cur += c
        elif c in ')]':
            depth -= 1
            cur += c
        elif c == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += c
        i += 1
    out.append(cur.strip())
    return out


def header_defines():
    d = {}
    for line in open(os.path.join(ROOT, 'include', 'usb_msc_fat.h'), encoding='utf-8'):
        m = re.match(r'#define\s+(\w+)\s+(-?[\d.]+)\b', line)
        if m:
            d[m.group(1)] = float(m.group(2))
    return d


def parse_table(defines):
    macros = header_defines()
    lines, stack = [], []
    for n, raw in enumerate(open(os.path.join(ROOT, 'include', 'settings_table.h'), encoding='utf-8'), 1):
        s = raw.strip()
        m = re.match(r'#if(n?)def\s+(\w+)', s)
        if m:
            stack.append((m.group(2) in defines) != bool(m.group(1)))
            continue
        if s.startswith('#endif'):
            stack.pop()
            continue
        if not (s.startswith('S(') or s.startswith('SF(')) or not all(stack):
            continue
        kind = s[:s.index('(')]
        body = s[s.index('(') + 1:s.rindex(')')]
        a = split_args(body)
        if len(a) != len(COLS):
            sys.exit(f'settings_table.h:{n}: {len(a)} columns, not {len(COLS)}')
        e = dict(zip(COLS, a))
        e['line'] = n
        e['sf'] = kind == 'SF'
        for k in ('opts', 'nvs', 'unit', 'group', 'label', 'help'):
            e[k] = json.loads(e[k])  # C string literal = JSON string here (no escapes beyond \" and \\)
        for k in ('default', 'min', 'max'):
            v = e[k]
            if v == 'NOLIM':
                e[k] = None
            elif v in macros:
                e[k] = macros[v]
            else:
                e[k] = float(v)
        e['flags'] = set() if e['flags'] == '0' else {f.strip() for f in e['flags'].split('|')}
        lines.append(e)
    return lines


def choices(opts):
    out = []
    for pos, seg in enumerate(opts.split('|')):
        name, _, val = seg.partition('=')
        out.append((name, int(val) if val else pos))
    return out


def json_default(e, like=None):
    t = e['type']
    if t == 'T_TEXT':
        return e['opts']
    if t == 'T_CHOICE':
        return next(n for n, v in choices(e['opts']) if v == int(e['default']))
    if t in ('T_BOOL', 'T_IBOOL'):
        return e['default'] != 0
    if TYPES[t] == 'int':
        return int(e['default'])
    v = e['default']
    if isinstance(like, int) and v == int(v):
        return int(v)
    return v


def check(lines):
    errs = []
    names, nvs = {}, {}
    for e in lines:
        if e['name'] in names:
            errs.append(f"line {e['line']}: name {e['name']} also on line {names[e['name']]}")
        names[e['name']] = e['line']
        if e['nvs'] in nvs:
            errs.append(f"line {e['line']}: NVS key {e['nvs']} also on line {nvs[e['nvs']]}")
        nvs[e['nvs']] = e['line']
        if len(e['nvs']) > 15:
            errs.append(f"line {e['line']}: NVS key {e['nvs']} longer than 15 characters")
        if e['type'] not in TYPES:
            errs.append(f"line {e['line']}: type {e['type']}")
        if e['type'] not in ('T_TEXT', 'T_CHOICE', 'T_BOOL', 'T_IBOOL'):
            d = e['default']
            ok = ('F_ZERO_OFF' in e['flags'] and d == 0) or (
                (e['min'] is None or d >= e['min']) and (e['max'] is None or d <= e['max']))
            if not ok:
                errs.append(f"line {e['line']}: {e['name']} default {d} outside {e['min']} to {e['max']}")
    return errs


def app_range(e):
    lo, hi = e['min'], e['max']
    if 'F_ZERO_OFF' in e['flags']:
        lo = 0.0
    return lo, hi


def generate(defines):
    lines = parse_table(defines)
    errs = check(lines)
    if errs:
        sys.exit('settings_table.h: ' + '; '.join(errs))
    base = json.load(open(os.path.join(ROOT, 'docs', 'settings_def.json'), encoding='utf-8'))
    out = copy.deepcopy(base)
    out['version'] = VERSION
    table = {e['name']: e for e in lines if 'F_LEGACY' not in e['flags']}
    notes = []
    settings = {}
    for key, old in base['settings'].items():
        e = table.get(key)
        if not e:
            settings[key] = old  # read-only keys (dsp_*_eff, wifi_*), or a key of the newkey build only
            continue
        ent = copy.deepcopy(old)
        t = TYPES[e['type']]
        if old.get('type') != t:
            sys.exit(f"{key}: table type {t}, settings_def.json type {old.get('type')}")
        d = json_default(e, old.get('default'))
        if d != old.get('default') or type(d) is not type(old.get('default')):
            sys.exit(f"{key}: table default {d!r}, settings_def.json default {old.get('default')!r}")
        ent['type'], ent['default'] = t, d
        # The range is the firmware's (0.6.26, operator): the app, the settings page and the programmer take it from
        # this file, so no other program keeps its own range.
        if t in ('int', 'float') and 'values' not in old and 'choices' not in old:
            lo, hi = app_range(e)
            for k, v in (('min', lo), ('max', hi)):
                if v is None:
                    ent.pop(k, None)
                else:
                    ent[k] = int(v) if t == 'int' else v
        settings[key] = ent
    for key, e in table.items():  # new keys: every word from the table line
        if key in base['settings']:
            continue
        t = TYPES[e['type']]
        ent = {'type': t}
        if e['unit']:
            ent['unit'] = e['unit']
        if t == 'choice':
            ent['values'] = [n for n, _ in choices(e['opts'])]
        elif t == 'string':
            ent['max_len'] = int(e['max'])
        elif t in ('int', 'float'):
            lo, hi = app_range(e)
            if lo is not None:
                ent['min'] = int(lo) if t == 'int' else lo
            if hi is not None:
                ent['max'] = int(hi) if t == 'int' else hi
        ent['default'] = json_default(e)
        ent['live'] = 'F_LIVE' in e['flags']
        if not e['group'] or e['group'] not in [g['name'] for g in base['groups']]:
            sys.exit(f"{key}: group {e['group']!r} is not in settings_def.json groups")
        ent['group'] = e['group']
        ent['description'] = e['help']
        ent['label'] = e['label']
        settings[key] = ent
    out['settings'] = settings
    return json.dumps(out, indent=2, ensure_ascii=False) + '\n', notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-D', action='append', default=[], help='a build flag that the table tests with #ifdef')
    ap.add_argument('-o', default=os.path.join(ROOT, 'docs', 'settings_def.json'))
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    text, notes = generate(set(a.D))
    if not a.quiet:
        for n in notes:
            print(n, file=sys.stderr)
    if a.check:
        cur = open(a.o, encoding='utf-8').read().replace('\r\n', '\n') if os.path.exists(a.o) else ''
        if cur != text:
            print(f'{a.o} differs from the table: run python tools/gen_settings_def.py', file=sys.stderr)
            return 1
        print(f'{a.o}: as generated')
        return 0
    os.makedirs(os.path.dirname(os.path.abspath(a.o)), exist_ok=True)
    open(a.o, 'w', encoding='utf-8', newline='\n').write(text)
    print(f'{a.o}: written, version {VERSION}, {len(json.loads(text)["settings"])} keys')
    return 0


if __name__ == '__main__':
    sys.exit(main())
