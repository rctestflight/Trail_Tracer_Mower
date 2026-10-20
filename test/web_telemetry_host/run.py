"""Desk test of the telemetry page answers (GET /api/telemetry?def=1, GET /api/telemetry[?all=1]).

The firmware code runs on the PC (host_main.cpp): lib/rover_telemetry packs random status records and samples, lib/telemlog
holds the status records among other SLOW records, lib/tlm_page takes the snapshot and writes both answers. This script:
  1. builds host_main.cpp with MSVC twice: no table value, and the example lines (-D TELEMETRY_NEWVALUE);
  2. writes telemetry_def.json for both tables (tools/gen_telemetry_def.py);
  3. field list: every page field against the generated definition (name, label, unit, decimals, group, kind), and the
     set of fields against the rules (status 0x21, samples 0x2D with the IMU axes, table values; no TIME field, no
     status field that the sample also has); the example values present on the example build only;
  4. values: decodes the records that hold the newest status and sample with tools/rover_log.py and compares each page
     value with the log value (numbers within half a unit of the last decimal; bits bit by bit; enum by name; text);
     the table values against the firmware packing rule; the short answer against the all answer; the ages;
  5. fault check: 4 deliberate faults in a copy of lib/tlm_page/tlm_page.cpp; each must give differences.

Usage (repository root, Windows, Visual Studio Build Tools installed):
    python test/web_telemetry_host/run.py [--cases 3000]
Exit code: 0 pass, 1 a check failed, 2 a build error.
"""
import importlib.util
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')
CASES = int(sys.argv[sys.argv.index('--cases') + 1]) if '--cases' in sys.argv else 3000


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


TT = load_module('tt_run', os.path.join(ROOT, 'test', 'telemetry_table_host', 'run.py'))  # expected(): the packing rule


def build(name, page_src, defs):
    """MSVC build of host_main.cpp with the firmware sources; the exe path, or None after a build error."""
    vswhere = r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    vs = subprocess.run([vswhere, '-latest', '-products', '*', '-property', 'installationPath'],
                        capture_output=True, text=True).stdout.strip()
    vcvars = os.path.join(vs, 'VC', 'Auxiliary', 'Build', 'vcvars64.bat')
    incs = [os.path.join(ROOT, 'include')] + [os.path.join(ROOT, 'lib', x) for x in
                                              ('tlm', 'tlm_page', 'rover_telemetry', 'telemlog')]
    srcs = [os.path.join(HERE, 'host_main.cpp'), page_src, os.path.join(ROOT, 'lib', 'tlm', 'tlm.cpp'),
            os.path.join(ROOT, 'lib', 'rover_telemetry', 'rover_records.cpp'),
            os.path.join(ROOT, 'lib', 'telemlog', 'telemlog.cpp')]
    defs = defs + ['TELEMLOG_FAST_SLOTS=10', 'TELEMLOG_SLOW_SLOTS=24']  # as platformio.ini
    out = os.path.join(BUILD, name)
    os.makedirs(out, exist_ok=True)
    exe = os.path.join(out, 'test.exe')
    cl = ('cl /nologo /std:c++17 /EHsc /O2 /W3 /wd4244 /wd4267 /wd4996 /wd4305 /constexpr:steps100000000 '
          '/constexpr:depth1024 ' + ' '.join(f'/D{d}' for d in defs) + ' ' + ' '.join(f'/I"{i}"' for i in incs)
          + f' /Fo"{out}\\\\" /Fe"{exe}" ' + ' '.join(f'"{x}"' for x in srcs))
    bat = os.path.join(out, 'build.bat')
    open(bat, 'w').write(f'@call "{vcvars}" >nul\r\n{cl}\r\n')
    r = subprocess.run(['cmd', '/c', bat], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout[-6000:], r.stderr[-3000:])
        return None
    return exe


def run(exe, out, cases, seed):
    subprocess.run([exe, out, str(cases), str(seed)], check=True)
    return open(out).read().splitlines()


def def_fields(tdef):
    """The page field set by the rules, from telemetry_def.json: {(record, name): field entry}."""
    st = tdef['records']['0x21']['fields']
    sm = tdef['records']['0x2d']['repeat']['fields']
    sample_names = {f.get('list', f['name']) for f in sm}
    out = {}
    for f in st:
        if f['kind'] != 'time' and f.get('list', f['name']) not in sample_names:
            out[('status', f['name'])] = f
    for f in sm:
        if f['kind'] != 'time':
            out[('samples', f['name'])] = f
    for b in ('0x34', '0x35', '0x36'):
        r = tdef['records'].get(b)
        if r:
            for f in r['repeat']['fields']:
                out[('tlm', f['name'])] = f
    return out


KIND = {'value': 'v', 'bits': 'bits', 'enum': 'enum', 'text': 'text'}


def check_def(page, tdef, label, expect_tlm, fails):
    want = def_fields(tdef)
    got = {(f['r'], f['n']): f for f in page['fields']}
    bad = 0
    for k in want:
        if k not in got:
            print(f'  {label}: field {k} missing on the page')
            bad += 1
    for k in got:
        if k not in want:
            print(f'  {label}: field {k} on the page, not in the rules')
            bad += 1
    words = 0
    for k, f in got.items():
        w = want.get(k)
        if not w:
            continue
        for pk, wk in (('l', 'label'), ('u', 'unit'), ('d', 'decimals'), ('g', 'group')):
            words += 1
            if f[pk] != w[wk]:
                print(f'  {label}: {k} {pk} page {f[pk]!r} table {w[wk]!r}')
                bad += 1
        words += 1
        if f['k'] != KIND.get(w['kind'], w['kind']):
            print(f'  {label}: {k} kind page {f["k"]} table {w["kind"]}')
            bad += 1
    tl = [f['n'] for f in page['fields'] if f['r'] == 'tlm']
    tl_short = [f['n'] for f in page['fields'] if f['r'] == 'tlm' and f['p'] > 0]
    print(f'CHECK field list {label}: {len(got)} page fields, {len(want)} by the rules, {words} words compared, '
          f'{bad} differences; table values {tl} (in the short set: {len(tl_short)})')
    if bad:
        fails.append(f'field list {label}: {bad}')
    if tl != expect_tlm or tl_short != expect_tlm:
        print(f'  {label}: table values {tl}, want {expect_tlm}')
        fails.append(f'table values {label}')
    if page['hash'] != tdef['version']:
        print(f'  {label}: hash {page["hash"]} definition {tdef["version"]}')
        fails.append(f'hash {label}')


def num_equal(p, q, dec):
    """Page number p against log value q, shown with dec decimals."""
    if q is None or (isinstance(q, float) and (math.isnan(q) or math.isinf(q))):
        return p is None
    if p is None:
        return False
    if abs(q) >= 1e15:
        return math.isclose(p, q, rel_tol=1e-5)
    return abs(p - q) <= 0.5 * 10 ** -dec * (1 + 1e-9) + 1e-9 * abs(q)


def check_values(lines, tdef, page, label, fails):
    RL = load_module('rover_log_' + label, os.path.join(ROOT, 'tools', 'rover_log.py'))
    RL.DEFS.clear()
    RL.DEF_ORDER.clear()
    p = os.path.join(BUILD, label, 'telemetry_def.json')
    RL.load_defs([p])
    fields = page['fields']
    tl_def = {}
    for b in ('0x34', '0x35', '0x36'):
        r = tdef['records'].get(b)
        if r:
            for f in r['repeat']['fields']:
                tl_def[f['name']] = f
    want_def = def_fields(tdef)
    short_idx = [i for i, f in enumerate(fields) if f['p'] > 0]
    stats = {'compared': 0, 'equal': 0, 'different': 0, 'null_both': 0, 'short_mismatch': 0, 'age_bad': 0,
             'status_kept': 0, 'status_found_bad': 0}
    by_rec = {}
    status_log = None
    case = None
    shown = 0
    for ln in lines:
        tag, _, rest = ln.partition(' ')
        if tag == 'CASE':
            case = dict(kv.split('=') for kv in rest.split()[1:])
            case = {k: int(v) for k, v in case.items()}
            tlmv = {}
            if case['status_found'] != (1 if case['others'] <= 23 else 0):
                stats['status_found_bad'] += 1
            if not case['status_found'] and status_log is not None:
                stats['status_kept'] += 1
        elif tag == 'STATUS':
            ms, hx = rest.split()
            status_log = RL.decode_body(bytes.fromhex(hx), int(ms))
            status_ms = int(ms)
        elif tag == 'SAMPLES':
            ms, hx = rest.split()
            d = RL.decode_body(bytes.fromhex(hx), int(ms))
            sample_log = d['samples'][-1]
            sample_imu = d['imu']
        elif tag == 'TLMV':
            i, w = rest.split()
            tlmv[int(i)] = struct.unpack('<f', struct.pack('<I', int(w, 16)))[0]
        elif tag == 'SHORT':
            short = json.loads(rest)
        elif tag == 'ALL':
            a = json.loads(rest)
            if [a['v'][i] for i in short_idx] != short['v']:
                stats['short_mismatch'] += 1
            want_st = None if status_log is None else case['now'] - status_ms
            want_sa = case['now'] - sample_log['ms']
            if a['st'] != want_st or a['sa'] != want_sa:
                stats['age_bad'] += 1
            tl_names = [f['n'] for f in fields if f['r'] == 'tlm']
            for i, f in enumerate(fields):
                pv = a['v'][i]
                key = (f['r'], f['n'])
                w = want_def[key]
                if f['r'] == 'tlm':
                    q = TT.expected(tl_def[f['n']], tlmv[tl_names.index(f['n'])])
                    ok = num_equal(pv, q, f['d'])
                else:
                    src = status_log if f['r'] == 'status' else sample_log
                    if src is None or (f['r'] == 'samples' and not sample_imu and w['offset'] >= 50):
                        ok, q = pv is None, None
                    elif f['k'] == 'bits':
                        q = [src.get(b) for b in w['bits'] if b]
                        names = f['b'].split('|')
                        got = [(pv >> j & 1) if pv is not None else None for j, b in enumerate(names) if b]
                        ok = got == q
                    else:
                        q = src[w['list']][w['index'][0]] if 'list' in w else src.get(w['name'])
                        if f['k'] == 'enum':
                            nm = dict(x.split('=', 1) for x in f['e'].split('|') if '=' in x)
                            got = None if pv is None else nm.get(str(pv), nm.get('*', pv))
                            ok = got == q
                        elif f['k'] == 'text':
                            ok = pv == q
                        else:
                            ok = num_equal(pv, q, f['d'])
                stats['compared'] += 1
                r = by_rec.setdefault(f['r'], [0, 0])
                r[0] += 1
                if ok:
                    stats['equal'] += 1
                    if pv is None:
                        stats['null_both'] += 1
                else:
                    stats['different'] += 1
                    r[1] += 1
                    if shown < 12:
                        shown += 1
                        print(f'  {label} case {case}: {key} page {pv!r} log {q!r}')
    print(f'CHECK values {label}: {stats["compared"]} compared, {stats["equal"]} equal ({stats["null_both"]} no value on '
          f'both), {stats["different"]} different; by record {by_rec}; short answer = all answer at its fields: '
          f'{stats["short_mismatch"]} mismatches; ages wrong: {stats["age_bad"]}; status kept from an earlier copy: '
          f'{stats["status_kept"]}; status found against the ring rule: {stats["status_found_bad"]} wrong')
    n_bad = stats['different'] + stats['short_mismatch'] + stats['age_bad'] + stats['status_found_bad']
    if n_bad:
        fails.append(f'values {label}: {n_bad}')
    return stats


def one(label, defs, expect_tlm, fails, page_src=None, cases=CASES, quiet=False):
    exe = build(label, page_src or os.path.join(ROOT, 'lib', 'tlm_page', 'tlm_page.cpp'), defs)
    if not exe:
        return None
    dpath = os.path.join(BUILD, label, 'telemetry_def.json')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'gen_telemetry_def.py'), '--quiet']
                   + sum((['-D', d] for d in defs), []) + ['-o', dpath], check=True)
    tdef = json.load(open(dpath, encoding='utf-8'))
    lines = run(exe, os.path.join(BUILD, label, 'out.txt'), cases, 7)
    info = next(ln for ln in lines if ln.startswith('INFO '))
    sizes = next(ln for ln in lines if ln.startswith('SIZES '))
    page = json.loads(next(ln for ln in lines if ln.startswith('DEF '))[4:])
    if not quiet:
        print(f'{label}: {info[5:]}; {sizes[6:]}')
        if 'missing_short=0' not in info:
            fails.append(f'{label}: a short-set name not in the table')
    check_def(page, tdef, label, expect_tlm, fails)
    return check_values(lines, tdef, page, label, fails)


FAULTS = [
    ('IMU axes read 2 bytes late', 'off += rec::SAMPLE3_BYTES;', 'off += rec::SAMPLE3_BYTES + 2;'),
    ('signed fields read as unsigned', 'if (t.sign && (u >>', 'if (false && (u >>'),
    ('the first sample of the record, not the newest', 'b + hdr + (n - 1) * stride', 'b + hdr'),
    ('the status search looks at the newest record only', 'k < TELEMLOG_SLOW_SLOTS', 'k < 1'),
]


def main():
    fails = []
    ex = ['test_frames', 'test_r1_mm', 'test_drive', 'test_heap_kb']
    if one('empty', [], [], fails) is None or one('newvalue', ['TELEMETRY_NEWVALUE'], ex, fails) is None:
        return 2
    src = os.path.join(ROOT, 'lib', 'tlm_page', 'tlm_page.cpp')
    text = open(src, encoding='utf-8').read()
    for i, (name, a, b) in enumerate(FAULTS):
        if text.count(a) != 1:
            print(f'fault {name}: text not found once')
            fails.append(f'fault text {i}')
            continue
        d = os.path.join(BUILD, f'fault{i}')
        os.makedirs(d, exist_ok=True)
        fsrc = os.path.join(d, 'tlm_page.cpp')
        open(fsrc, 'w', encoding='utf-8').write(text.replace(a, b))
        ff = []
        st = one(f'fault{i}', ['TELEMETRY_NEWVALUE'], ex, ff, page_src=fsrc, cases=500, quiet=True)
        n = (st['different'] + st['status_found_bad'] + st['age_bad']) if st else None
        print(f'CHECK fault "{name}": {n} differences ({"detected" if n else "NOT DETECTED"})')
        if not n:
            fails.append(f'fault not detected: {name}')
    print('RESULT ' + ('PASS' if not fails else 'FAIL: ' + '; '.join(fails)))
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
