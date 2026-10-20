"""Write telemetry_def.json (the definition of every telemetry log record, for the telemetry app and tools/rover_log.py)
from the telemetry table, include/telemetry_table.h and the user lines it includes (include/user_telemetry.h).

The table has three kinds of line (columns in the table header): TLR describes a fixed record, TLF a field of a fixed
record, TLM a table value. This script writes one entry per record body: its fields with type, offset, scale, unit,
decimals, label, group, kind and "no value" code, and its repeated block (samples) if it has one. The table values give
the records 0x34 (1 Hz), 0x35 (10 Hz) and 0x36 (the log rate), one for each rate in use.

"version" is the table hash: the CRC-32 of the table text, which lib/tlm computes from the same lines at compile time.
The vehicle sends it in every table record and answers it to "tlm" and "get telemetry_def".
The format is in docs/espnow_protocol.md, "Telemetry definition".

Checks: the column count, known types, kinds, rates and layouts, a plain number for a scale (not 0), decimals 0 to 9,
the TLM rules of part 1 (C name used once, bytes a sample per rate: LOG 16, 10HZ 64, 1HZ 77), a TLM name that is not a
fixed field name, a block field inside its block.

Usage (repository root):
    python tools/gen_telemetry_def.py                     write docs/telemetry_def.json (the default build)
    python tools/gen_telemetry_def.py --check             exit 1 if docs/telemetry_def.json is not the generated text
    python tools/gen_telemetry_def.py -D TELEMETRY_NEWVALUE -o .pio/build/telemetry_newvalue/telemetry_def.json
"""
import argparse
import itertools
import json
import math
import os
import re
import sys
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
FORMAT = 2
TLM_HDR = 6  # bytes after the body type: u32 table hash, u8 sample count n, u8 rate in Hz
COLS = {
    'TLR': ['body', 'name', 'stream', 'rate', 'header', 'block', 'count_offset', 'block_offset', 'stride', 'min_bytes',
            'consts'],
    'TLF': ['layout', 'name', 'type', 'offset', 'scale', 'unit', 'decimals', 'kind', 'none', 'group', 'label', 'names'],
    'TLM': ['name', 'type', 'scale', 'unit', 'decimals', 'rate', 'group', 'label'],
}
STRINGS = {'TLR': ('header', 'block', 'consts'), 'TLF': ('unit', 'group', 'label', 'names'),
           'TLM': ('unit', 'group', 'label')}
HASH_TAG = {'TLR': 'R', 'TLF': 'F', 'TLM': 'V'}
# Fixed field types: bytes (0: to the body end), JSON type
FTYPES = {'U8': (1, 'u8'), 'I8': (1, 'i8'), 'U16': (2, 'u16'), 'I16': (2, 'i16'), 'U24': (3, 'u24'), 'I24': (3, 'i24'),
          'U32': (4, 'u32'), 'I32': (4, 'i32'), 'F32': (4, 'f32'), 'BF16': (2, 'bf16'), 'TEXT': (0, 'text'),
          'PTEXT': (0, 'text'), 'VERSION': (3, 'version'), 'UNIXMS': (6, 'unix_ms')}
KINDS = ('VALUE', 'BITS', 'ENUM', 'TEXT', 'TIME', 'HEX')
# TLM value types: bytes, "no value" code
TYPES = {
    'U8': (1, 255), 'I8': (1, -128), 'U16': (2, 65535), 'I16': (2, -32768),
    'U32': (4, 4294967295), 'I32': (4, -2147483648), 'F32': (4, 'NaN'),
}
# TLM rate token: rate name, record body, stream, rate in Hz (None: the log rate), bytes a sample at most
RATES = {
    '1HZ': ('1hz', 0x34, 1, 1, 77),
    '10HZ': ('10hz', 0x35, 0, 10, 64),
    'LOG': ('log', 0x36, 0, None, 16),
}
# TLR rate tokens that are not a number: what sets the rate
RATE_TEXT = {'LOG': 'telemetry_rate_hz', 'BINS': 'telemetry_bins_hz', 'CHANGE': 'on a change',
             'SWEEP': 'each sweep page'}
BODY_MAX = {0: 228, 1: 84}  # telemlog body limits (bytes with the body type)


def split_args(s):
    out, cur, depth, q, i = [], '', 0, False, 0
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


def strip_comment(s):
    q = False
    for i, c in enumerate(s):
        if c == '"' and (i == 0 or s[i - 1] != '\\'):
            q = not q
        elif not q and s.startswith('//', i):
            return s[:i].rstrip()
    return s


def parse_file(path, defines, lines, errs):
    stack = []
    for n, raw in enumerate(open(path, encoding='utf-8'), 1):
        s = strip_comment(raw.strip())
        where = f'{os.path.basename(path)}:{n}'
        m = re.match(r'#\s*if(n?)def\s+(\w+)', s)
        if m:
            stack.append((m.group(2) in defines) != bool(m.group(1)))
            continue
        if re.match(r'#\s*else\b', s):
            stack[-1] = not stack[-1]
            continue
        if re.match(r'#\s*endif\b', s):
            stack.pop()
            continue
        if not all(stack):
            continue
        m = re.match(r'#\s*include\s+"([^"]+)"', s)
        if m:
            parse_file(os.path.join(os.path.dirname(path), m.group(1)), defines, lines, errs)
            continue
        m = re.match(r'(TLR|TLF|TLM)\(', s)
        if not m:
            continue
        macro = m.group(1)
        if not s.endswith(')'):
            errs.append(f'{where}: a {macro} line ends with ")" (one line each)')
            continue
        a = split_args(s[4:-1])
        cols = COLS[macro]
        if len(a) != len(cols):
            errs.append(f'{where}: {len(a)} columns, not {len(cols)}')
            continue
        e = dict(zip(cols, a))
        try:
            for k in STRINGS[macro]:
                e[k] = json.loads(e[k])
        except ValueError:
            errs.append(f'{where}: {", ".join(STRINGS[macro])} are "text"')
            continue
        e['macro'] = macro
        e['where'] = where
        lines.append(e)


def tok(s):
    return ' '.join(s.split())  # the C preprocessor's # operator: no outer spaces, inner runs to one space


def hash_line(e):
    """The line lib/tlm builds for the hash: the tokens as the # operator writes them, the strings as their text."""
    m = e['macro']
    parts = [e[k] if k in STRINGS[m] else tok(e[k]) for k in COLS[m]]
    return HASH_TAG[m] + '|' + '|'.join(parts) + '\n'


def number(s):
    """A plain number or a fraction a/b."""
    s = tok(s)
    if '/' in s:
        a, b = s.split('/')
        return float(a) / float(b)
    return float(s)


def jnum(x):
    """A whole float as an int in the JSON (1, not 1.0)."""
    return int(x) if isinstance(x, float) and x.is_integer() and abs(x) < 2 ** 53 else x


def opt_int(s):
    s = tok(s)
    return None if s == '-' else int(s, 0)


def fixed_fields(e, errs):
    """One TLF line to its fields: an array name[n] or name[n][m] gives one field per element."""
    w = e['where']
    m = re.fullmatch(r'([A-Za-z_]\w*)((?:\[\d+\])*)', tok(e['name']).replace(' ', ''))
    if not m:
        errs.append(f'{w}: name {e["name"]!r}: a C name, or name[n] or name[n][m]')
        return []
    base, dims = m.group(1), [int(x) for x in re.findall(r'\[(\d+)\]', m.group(2))]
    tm = re.fullmatch(r'([A-Z0-9]+)(?:(>>|&)(\d+))?', tok(e['type']).replace(' ', ''))
    if not tm or tm.group(1) not in FTYPES:
        errs.append(f'{w}: type {e["type"]}: use one of {", ".join(FTYPES)} (U8>>4, U8&15 for a bit field)')
        return []
    tname = tm.group(1)
    size, jtype = FTYPES[tname]
    kind = tok(e['kind'])
    if kind not in KINDS:
        errs.append(f'{w}: kind {kind}: use one of {", ".join(KINDS)}')
        return []
    try:
        scale = number(e['scale'])
        off = int(tok(e['offset']), 0)
        dec = int(tok(e['decimals']))
        none = opt_int(e['none'])
        if not (math.isfinite(scale) and scale != 0 and 0 <= dec <= 9 and off >= 0):
            raise ValueError
    except (ValueError, ZeroDivisionError):
        errs.append(f'{w}: offset, scale, decimals or none: numbers (scale not 0, decimals 0 to 9, none or -)')
        return []
    names = [x for x in e['names'].split('|')] if e['names'] else []
    f0 = {'type': jtype, 'scale': jnum(scale), 'unit': e['unit'], 'decimals': dec, 'label': e['label'],
          'group': e['group'], 'kind': kind.lower(), 'none': none}
    if tm.group(2) == '>>':
        f0['shift'] = int(tm.group(3))
        f0['mask'] = (1 << (8 * size - int(tm.group(3)))) - 1
    elif tm.group(2) == '&':
        f0['mask'] = int(tm.group(3))
    if kind == 'BITS':
        f0['bits'] = names
        if len(names) > 8 * size:
            errs.append(f'{w}: {len(names)} bit names for {8 * size} bits')
    elif kind == 'ENUM':
        f0['names'] = {}
        for x in names:
            k, _, v = x.partition('=')
            if k == '*':
                f0['unknown'] = v
            elif k.startswith('@'):
                f0['when'] = [k[1:], json.loads(v)]
            else:
                f0['names'][str(int(k, 0))] = v
    elif names and not dims:
        errs.append(f'{w}: names only for BITS, ENUM or an array')
    if dims and names and len(names) != dims[-1]:
        errs.append(f'{w}: {len(names)} element names for {dims[-1]} elements')
    out = []
    for flat, idx in enumerate(itertools.product(*[range(d) for d in dims]) if dims else [()]):
        f = {'name': base + ''.join(f'_{i}' for i in idx), **f0}
        f['offset'] = off + flat * size
        if tname == 'PTEXT':
            f['offset'], f['length_offset'] = off + 1, off
        if idx:
            f['list'], f['index'] = base, list(idx)
            f['label'] = e['label'] + ''.join(f' {i}' for i in idx[:-1]) + (
                f', {names[idx[-1]]}' if names else f' {idx[-1]}')
        out.append(f)
    return out


def ordered(f):
    """Field keys in a fixed order: name, type, offset first."""
    keys = ['name', 'type', 'offset', 'length_offset', 'scale', 'unit', 'decimals', 'label', 'group', 'kind', 'none',
            'bits', 'names', 'unknown', 'when', 'shift', 'mask', 'list', 'index']
    return {k: f[k] for k in keys if k in f}


def place(layouts, spec, w, errs):
    """'LAYOUT[@base] ...' to a list of fields with the base added to each offset."""
    out = []
    for part in spec.split():
        name, _, base = part.partition('@')
        if name not in layouts:
            errs.append(f'{w}: layout {name} has no TLF line')
            continue
        b = int(base, 0) if base else 0
        out += [{**f, 'offset': f['offset'] + b, **({'length_offset': f['length_offset'] + b} if 'length_offset' in f
                                                    else {})} for f in layouts[name]]
    return out


def fixed_records(lines, errs):
    layouts = {}
    for e in lines:
        if e['macro'] == 'TLF':
            layouts.setdefault(tok(e['layout']), []).extend(fixed_fields(e, errs))
    records = {}
    for e in lines:
        if e['macro'] != 'TLR':
            continue
        w = e['where']
        try:
            body = int(tok(e['body']), 0)
            stream = int(tok(e['stream']))
            cnt, blk, stride = (opt_int(e[k]) for k in ('count_offset', 'block_offset', 'stride'))
            mn = int(tok(e['min_bytes']), 0)
        except ValueError:
            errs.append(f'{w}: body, stream, count_offset, block_offset, stride, min_bytes: numbers or -')
            continue
        key = '0x%02x' % body
        if key in records:
            errs.append(f'{w}: body {key} twice')
        rate = tok(e['rate'])
        r = {'name': tok(e['name']), 'stream': stream}
        if rate in RATE_TEXT:
            r['rate_hz'], r['rate'] = None, RATE_TEXT[rate]
        else:
            try:
                r['rate_hz'] = jnum(float(rate))
            except ValueError:
                errs.append(f'{w}: rate {rate}: a number or one of {", ".join(RATE_TEXT)}')
        r['min_bytes'] = mn
        if e['consts']:
            r['const'] = {}
            for kv in e['consts'].split():
                k, _, v = kv.partition('=')
                r['const'][k] = json.loads(v)
        r['fields'] = [ordered(f) for f in place(layouts, e['header'], w, errs)]
        if e['block'] == 'SPECIAL':
            r['special'] = f'the part after these fields: docs/espnow_protocol.md, body {key}'
        elif e['block']:
            if None in (cnt, blk, stride):
                errs.append(f'{w}: a block needs count_offset, block_offset and stride')
                continue
            fs = place(layouts, e['block'], w, errs)
            for f in fs:
                if f['offset'] + FTYPES_SIZE(f) > stride:
                    errs.append(f'{w}: field {f["name"]} ends at {f["offset"] + FTYPES_SIZE(f)}, after the block ({stride} B)')
            r['repeat'] = {'name': 'samples', 'count_offset': cnt, 'offset': blk, 'stride': stride,
                           'fields': [ordered(f) for f in fs]}
        records[key] = r
    return records, layouts


def FTYPES_SIZE(f):
    return {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u24': 3, 'i24': 3, 'u32': 4, 'i32': 4, 'f32': 4, 'bf16': 2,
            'version': 3, 'unix_ms': 6, 'text': 0}[f['type']]


def table_values(lines, fixed_names, errs):
    names = {}
    values = []
    for e in lines:
        if e['macro'] != 'TLM':
            continue
        w = e['where']
        name, typ, rate = tok(e['name']), tok(e['type']), tok(e['rate'])
        if not re.fullmatch(r'[A-Za-z_]\w*', name):
            errs.append(f'{w}: name {name!r} is not a C name')
        if name in names:
            errs.append(f'{w}: name {name} also at {names[name]}')
        if name in fixed_names:
            errs.append(f'{w}: name {name} is a fixed field name: choose another')
        names[name] = w
        if typ not in TYPES:
            errs.append(f'{w}: type {typ}: use one of {", ".join(TYPES)}')
            continue
        if rate not in RATES:
            errs.append(f'{w}: rate {rate}: use one of {", ".join(RATES)}')
            continue
        try:
            scale = float(tok(e['scale']))
            ok = math.isfinite(scale) and scale != 0
        except ValueError:
            ok = False
        if not ok:
            errs.append(f'{w}: scale {e["scale"]!r}: a plain number, not 0')
            continue
        try:
            dec = int(tok(e['decimals']))
            ok = 0 <= dec <= 9
        except ValueError:
            ok = False
        if not ok:
            errs.append(f'{w}: decimals {e["decimals"]!r}: 0 to 9')
            continue
        values.append({'name': name, 'type': typ.lower(), 'offset': None, 'scale': jnum(scale), 'unit': e['unit'],
                       'decimals': dec, 'label': e['label'], 'group': e['group'], 'kind': 'value',
                       'none': TYPES[typ][1], 'rate': rate, 'bytes': TYPES[typ][0]})
    records = {}
    for rate, (rname, body, stream, hz, most) in RATES.items():
        vs = [v for v in values if v['rate'] == rate]
        off = 0
        for v in vs:
            v['offset'] = off
            off += v['bytes']
        if off > most:
            errs.append(f'rate {rate}: {off} B a sample, {most} B at most: move a value to another rate')
        if not vs:
            continue
        r = {'name': 'tlm', 'stream': stream, 'rate_hz': hz}
        if hz is None:
            r['rate'] = RATE_TEXT['LOG']
        r['min_bytes'] = TLM_HDR
        r['const'] = {'rate': rname}
        r['fields'] = [
            {'name': 'hash', 'type': 'u32', 'offset': 0, 'scale': 1, 'unit': '', 'decimals': 0,
             'label': 'Table hash (telemetry_def.json "version")', 'group': 'Log', 'kind': 'hex', 'none': None},
            {'name': 'rate_hz', 'type': 'u8', 'offset': 5, 'scale': 1, 'unit': 'Hz', 'decimals': 0,
             'label': 'Sample rate', 'group': 'Log', 'kind': 'value', 'none': None}]
        room = BODY_MAX[stream] - 1 - TLM_HDR
        r['repeat'] = {'name': 'samples', 'count_offset': 4, 'offset': TLM_HDR, 'stride': off, 'rate_offset': 5,
                       'samples_max': min(room // off, hz or 100),
                       'fields': [ordered({k: x for k, x in v.items() if k not in ('rate', 'bytes')}) for v in vs]}
        records['0x%02x' % body] = r
    return records, len(values)


def build(defines):
    lines, errs = [], []
    parse_file(os.path.join(ROOT, 'include', 'telemetry_table.h'), defines, lines, errs)
    records, layouts = fixed_records(lines, errs)
    fixed_names = {f['name'] for fs in layouts.values() for f in fs}
    tv, n_values = table_values(lines, fixed_names, errs)
    for k, r in tv.items():
        if k in records:
            errs.append(f'body {k}: a fixed record and a table value record')
        records[k] = r
    if errs:
        sys.exit('telemetry table: ' + '; '.join(errs))
    text = f'tlm {FORMAT}\n' + ''.join(hash_line(e) for e in lines)
    h = zlib.crc32(text.encode('utf-8')) & 0xFFFFFFFF
    out = {
        'format': 'telemetry_def',
        'format_version': FORMAT,
        'version': '%08x' % h,
        'about': 'Every record of the rover telemetry log, generated by tools/gen_telemetry_def.py from '
                 'include/telemetry_table.h (Trail_Tracer_Mower). "version" is the table hash. Reader rules: '
                 'docs/espnow_protocol.md, "Telemetry definition".',
        'table_values': n_values,
        'records': dict(sorted(records.items())),
    }
    return dumps(out) + '\n', h, text


def dumps(o, ind=''):
    """JSON with one field (or other small object) per line."""
    one = lambda x: json.dumps(x, ensure_ascii=False, separators=(', ', ': '))  # noqa: E731
    if isinstance(o, dict) and not ('type' in o or len(one(o)) < 100):
        inner = ind + ' '
        return '{\n' + ',\n'.join(f'{inner}{one(k)}: {dumps(v, inner)}' for k, v in o.items()) + '\n' + ind + '}'
    if isinstance(o, list) and o and isinstance(o[0], dict):
        inner = ind + ' '
        return '[\n' + ',\n'.join(inner + dumps(v, inner) for v in o) + '\n' + ind + ']'
    return one(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-D', action='append', default=[], help='a build flag that the table tests with #ifdef')
    ap.add_argument('-o', default=os.path.join(ROOT, 'docs', 'telemetry_def.json'))
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--hash-text', help='also write the hash text to this file')
    a = ap.parse_args()
    text, h, htext = build(set(a.D))
    if a.hash_text:
        open(a.hash_text, 'w', encoding='utf-8', newline='\n').write(htext)
    if a.check:
        cur = open(a.o, encoding='utf-8').read().replace('\r\n', '\n') if os.path.exists(a.o) else ''
        if cur != text:
            print(f'{a.o} differs from the table: run python tools/gen_telemetry_def.py', file=sys.stderr)
            return 1
        print(f'{a.o}: as generated')
        return 0
    os.makedirs(os.path.dirname(os.path.abspath(a.o)), exist_ok=True)
    open(a.o, 'w', encoding='utf-8', newline='\n').write(text)
    if not a.quiet:
        d = json.loads(text)
        nf = sum(len(r['fields']) + len(r.get('repeat', {}).get('fields', [])) for r in d['records'].values())
        print(f'{a.o}: written, version (hash) {h:08x}, {len(d["records"])} records, {nf} fields, '
              f'{d["table_values"]} table values')
    return 0


if __name__ == '__main__':
    sys.exit(main())
