"""Desk test of the telemetry table: the firmware code packs records on the PC, tools/rover_log.py decodes them from
the telemetry_def.json that tools/gen_telemetry_def.py writes, and the result is compared with a reference.

Part A, table values (bodies 0x34 to 0x36): lib/tlm packs, each decoded value is compared with the packed value,
quantised as the firmware does.
Part B, fixed records (0x20-0x25, 0x2A-0x2D, 0x21, 0x2E, 0x30, 0x31, 0x32, 0x33): lib/rover_telemetry packs (fixed_main.cpp),
each decoded field is compared with the decode of tools/rover_log.py of 0.5.115 (git show b76ff27:tools/rover_log.py),
field by field (the JSON text of each leaf value).

Steps:
  1. Build with MSVC: host_main.cpp + lib/tlm with the example lines (-D TELEMETRY_NEWVALUE) and with no table value;
     fixed_main.cpp + lib/rover_telemetry.
  2. Write telemetry_def.json for both tables. The firmware hash (compile time) must equal the file "version".
  3. Part A: 600 s of simulated time; records as "LOG <hex>" lines, packed values as TRUTH lines;
     tools/rover_log.py csv; compare. Two bad records must be refused.
  4. Part B: the fixed records as "LOG <hex>" lines; decode each frame with both decoders; compare every leaf.

Usage (repository root, Windows, Visual Studio Build Tools installed, git on the path):
    python test/telemetry_table_host/run.py
    python test/telemetry_table_host/run.py --log <recorded log> [--def telemetry_def.json]   part B on a recorded log
Exit code: 0 pass, 1 a check failed, 2 a build error.
"""
import csv
import importlib.util
import json
import math
import os
import struct
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')
OLD_REV = 'b76ff27'  # 0.5.115


def build(name, srcs, defs):
    vswhere = r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    vs = subprocess.run([vswhere, '-latest', '-products', '*', '-property', 'installationPath'],
                        capture_output=True, text=True).stdout.strip()
    vcvars = os.path.join(vs, 'VC', 'Auxiliary', 'Build', 'vcvars64.bat')
    incs = [os.path.join(ROOT, 'include'), os.path.join(ROOT, 'lib', 'tlm'), os.path.join(ROOT, 'lib', 'rover_telemetry'),
            os.path.join(ROOT, 'lib', 'telemlog')]
    out = os.path.join(BUILD, name)
    os.makedirs(out, exist_ok=True)
    exe = os.path.join(out, 'test.exe')
    cl = ('cl /nologo /std:c++17 /EHsc /O2 /W3 /wd4244 /wd4267 /wd4996 /wd4305 /constexpr:steps100000000 '
          '/constexpr:depth1024 /DTLM_HOST_TRACE '
          + ' '.join(f'/D{d}' for d in defs) + ' ' + ' '.join(f'/I"{i}"' for i in incs)
          + f' /Fo"{out}\\\\" /Fe"{exe}" ' + ' '.join(f'"{s}"' for s in srcs))
    bat = os.path.join(out, 'build.bat')
    open(bat, 'w').write(f'@call "{vcvars}" >nul\r\n{cl}\r\n')
    r = subprocess.run(['cmd', '/c', bat], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout[-6000:], r.stderr[-3000:])
        return None
    return exe


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


LIM = {'u8': (0, 254, 255), 'i8': (-127, 127, -128), 'u16': (0, 65534, 65535), 'i16': (-32767, 32767, -32768),
       'u32': (0, 4294967040, 4294967295), 'i32': (-2147483520, 2147483520, -2147483648)}


def expected(v, value):
    """The value tools/rover_log.py must give for a packed float, by the firmware rules (lib/tlm/tlm.cpp pack())."""
    x = f32(f32(value) * f32(1.0 / v['scale']))
    if v['type'] == 'f32':
        return None if x != x else f32(x) * v['scale']
    lo, hi, none = LIM[v['type']]
    if x != x:
        return None
    if x <= lo:
        w = lo
    elif x >= hi:
        w = hi
    else:
        w = int(math.copysign(math.floor(abs(x) + 0.5), x))
    return None if w == none else w * v['scale']


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def leaves(x, p=''):
    if isinstance(x, dict):
        for k, v in x.items():
            yield from leaves(v, f'{p}.{k}' if p else k)
    elif isinstance(x, list):
        for i, v in enumerate(x):
            yield from leaves(v, f'{p}[{i}]')
    else:
        yield p, x


def part_a(exe_new, exe_empty, def_new, def_empty, fails):
    rec_new = os.path.join(BUILD, 'newvalue', 'records.txt')
    truth_new = os.path.join(BUILD, 'newvalue', 'truth.txt')
    out_new = subprocess.run([exe_new, rec_new, truth_new], capture_output=True, text=True).stdout
    out_empty = subprocess.run([exe_empty, os.path.join(BUILD, 'empty', 'r.txt'), os.path.join(BUILD, 'empty', 't.txt')],
                               capture_output=True, text=True).stdout
    sys.stdout.write(out_new + out_empty)
    for out, dpath, label in ((out_new, def_new, 'example table'), (out_empty, def_empty, 'no table value')):
        fw = out.split()[1]
        fh = json.load(open(dpath))['version']
        print(f'CHECK hash {label}: firmware {fw}, telemetry_def.json version {fh}: {"equal" if fw == fh else "DIFFERENT"}')
        if fw != fh:
            fails.append(f'hash {label}')
    if 'TOO_LONG 0' not in out_new:
        fails.append('a record over the stream body limit')

    csv_path = os.path.join(BUILD, 'newvalue', 'decoded.csv')
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'rover_log.py'), 'csv', rec_new, '--def', def_new,
                        '--out', csv_path], capture_output=True, text=True)
    sys.stdout.write(r.stderr)
    refused = sum(1 for ln in r.stderr.splitlines() if ln.startswith('record '))
    print(f'CHECK bad records refused by the decoder: {refused} of 2')
    if refused != 2:
        fails.append('bad records')

    d = json.load(open(def_new))
    vals = {}  # name -> (body, field)
    for key, rec in d['records'].items():
        if rec['name'] == 'tlm':
            for f in rec['repeat']['fields']:
                vals[f['name']] = (int(key, 16), f)
    rate_body = {rec['const']['rate']: int(k, 16) for k, rec in d['records'].items() if rec['name'] == 'tlm'}
    truth = {}
    for ln in open(truth_new):
        body, t, name, value = ln.split()
        truth[(int(body), int(t), name)] = float(value)
    seen = set()
    match = diff = nones = rows = 0
    examples = []
    for row in csv.DictReader(open(csv_path, newline='')):
        rows += 1
        body, t = rate_body[row['rate']], int(row['ms'])
        for name, (vb, v) in vals.items():
            if vb != body:
                continue
            key = (body, t, name)
            if key not in truth:
                diff += 1
                examples.append(f'{key}: decoded, not packed')
                continue
            seen.add(key)
            want = expected(v, truth[key])
            got = None if row[name] == '' else float(row[name])
            if want is None:
                nones += 1
            if (got is None and want is None) or (got is not None and want is not None
                                                  and abs(got - want) <= 1e-9 * max(1, abs(want))):
                match += 1
            else:
                diff += 1
                if len(examples) < 10:
                    examples.append(f'{key}: decoded {got}, packed {truth[key]} -> want {want}')
    unsent, per_body = {}, {}
    for key in truth:
        per_body[key[0]] = per_body.get(key[0], 0) + 1
        if key not in seen:
            unsent[key[0]] = unsent.get(key[0], 0) + 1
    print(f'CHECK A: CSV rows {rows}; values packed {len(truth)}, decoded and equal {match}, different {diff}, '
          f'of them "no value" {nones}')
    for b in sorted(per_body):
        print(f'CHECK A body 0x{b:02X}: values packed {per_body[b]}, in the open record at the end (not sent) {unsent.get(b, 0)}')
    for e in examples:
        print('DIFF', e)
    if diff:
        fails.append('table values differ')
    smax = {int(k, 16): r['repeat']['samples_max'] * len(r['repeat']['fields'])
            for k, r in d['records'].items() if r['name'] == 'tlm'}
    for b, n in unsent.items():
        if n > smax.get(b, 0):
            fails.append(f'body 0x{b:02X}: {n} values never sent')


def part_b(exe_fixed, def_empty, fails, rec_path=None):
    """rec_path: a recorded log ("LOG <hex>" or monitor "R ..." lines) in place of the packed records."""
    os.makedirs(os.path.join(BUILD, 'fixed'), exist_ok=True)
    if rec_path is None:
        rec_path = os.path.join(BUILD, 'fixed', 'records.txt')
        out = subprocess.run([exe_fixed, rec_path], capture_output=True, text=True).stdout
        sys.stdout.write(out)
    old_path = os.path.join(BUILD, 'fixed', 'rover_log_0.5.115.py')
    src = subprocess.run(['git', '-C', ROOT, 'show', f'{OLD_REV}:tools/rover_log.py'], capture_output=True,
                         check=True).stdout
    open(old_path, 'wb').write(src)
    old = load_module('rover_log_old', old_path)
    new = load_module('rover_log_new', os.path.join(ROOT, 'tools', 'rover_log.py'))
    new.load_defs([def_empty])
    stats = {}  # body name -> [records, leaves, equal, different, only old, only new]
    examples = {}
    table_records = 0
    for _, fr in new.frames_from_lines(open(rec_path, encoding='utf-8', errors='replace')):
        if len(fr) > 14 and fr[0] == 0x40 and fr[14] in new.TLM_BODIES:
            table_records += 1  # 0.5.115 has no table records: not compared
            new.decode_frame(fr)
            continue
        a, b = old.decode_frame(fr), new.decode_frame(fr)
        key = a.get('body') or '?'
        if key == 'status':
            key = 'status %d B' % (len(fr) - 14)
        st = stats.setdefault(key, [0, 0, 0, 0, 0, 0])
        st[0] += 1
        la, lb = dict(leaves(a)), dict(leaves(b))
        for p in la.keys() | lb.keys():
            st[1] += 1
            if p not in lb:
                st[4] += 1
                kind = 'only old'
            elif p not in la:
                st[5] += 1
                kind = 'only new'
            elif json.dumps(la[p]) == json.dumps(lb[p]):
                st[2] += 1
                continue
            else:
                st[3] += 1
                kind = 'different'
            field = p.split('.')[-1].split('[')[0] if '.' in p else p.split('[')[0]
            ex = examples.setdefault((key, kind, field), [0, f'{p}: 0.5.115 {la.get(p, "-")!r}, table {lb.get(p, "-")!r}'])
            ex[0] += 1
    tot = [0] * 6
    print('CHECK B: body | records | fields compared | equal | different | only 0.5.115 | only table decode')
    for k in sorted(stats):
        s = stats[k]
        tot = [x + y for x, y in zip(tot, s)]
        print(f'CHECK B {k} | ' + ' | '.join(str(x) for x in s))
    print('CHECK B all | ' + ' | '.join(str(x) for x in tot))
    print(f'CHECK B table records (not compared): {table_records}')
    for (k, kind, field), (n, e) in sorted(examples.items()):
        print(f'DIFF B {k} {kind} {field}: {n} times, for example {e}')
    # Expected: status records under 74 B (firmware 0.5.21 and older) have pc_unix_ms None in place of no key.
    unexpected = [(k, kind, f) for (k, kind, f) in examples
                  if not (kind == 'only new' and f == 'pc_unix_ms' and k in ('status 67 B', 'status 68 B'))]
    if unexpected:
        fails.append('fixed records differ: ' + ', '.join(f'{k} {kind} {f}' for k, kind, f in unexpected[:8]))


def main():
    if '--log' in sys.argv:  # bench: compare a recorded log only (python run.py --log <file> --def <telemetry_def.json>)
        log = sys.argv[sys.argv.index('--log') + 1]
        dpath = sys.argv[sys.argv.index('--def') + 1] if '--def' in sys.argv else os.path.join(ROOT, 'docs', 'telemetry_def.json')
        fails = []
        part_b(None, dpath, fails, log)
        print('RESULT ' + ('PASS' if not fails else 'FAIL: ' + '; '.join(fails)))
        return 0 if not fails else 1
    lib_tlm = [os.path.join(ROOT, 'lib', 'tlm', 'tlm.cpp'), os.path.join(ROOT, 'lib', 'rover_telemetry', 'rover_records.cpp')]
    exe_new = build('newvalue', [os.path.join(HERE, 'host_main.cpp')] + lib_tlm, ['TELEMETRY_NEWVALUE'])
    exe_empty = build('empty', [os.path.join(HERE, 'host_main.cpp')] + lib_tlm, [])
    exe_fixed = build('fixed', [os.path.join(HERE, 'fixed_main.cpp'),
                                os.path.join(ROOT, 'lib', 'rover_telemetry', 'rover_records.cpp')], [])
    if not exe_new or not exe_empty or not exe_fixed:
        return 2
    fails = []
    gen = os.path.join(ROOT, 'tools', 'gen_telemetry_def.py')
    def_new = os.path.join(BUILD, 'newvalue', 'telemetry_def.json')
    def_empty = os.path.join(BUILD, 'empty', 'telemetry_def.json')
    subprocess.run([sys.executable, gen, '--quiet', '-D', 'TELEMETRY_NEWVALUE', '-o', def_new], check=True)
    subprocess.run([sys.executable, gen, '--quiet', '-o', def_empty], check=True)
    part_a(exe_new, exe_empty, def_new, def_empty, fails)
    part_b(exe_fixed, def_empty, fails)
    print('RESULT ' + ('PASS' if not fails else 'FAIL: ' + '; '.join(fails)))
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
