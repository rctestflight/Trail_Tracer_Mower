'use strict';
const ESPRESSIF = 0x303A, ROM_PID = 0x0002, APP_PID = 0x80C2;
const PAGE_VERSION = '1.12.0';
let pageVer = `Page v${PAGE_VERSION} (local copy)`;
const ESPTOOL = 'https://unpkg.com/esptool-js@0.4.7/bundle.js';
const ESPTOOL_SRI = 'sha384-esoYAtcIwSlHrIf+6+uZc/SPs/S8v44K26De13juLayamqHofUvCPscyHBXPxVIc';
async function loadEsptool() {
  const text = await (await fetch(ESPTOOL, { integrity: ESPTOOL_SRI, cache: 'no-store' })).text();
  return import(URL.createObjectURL(new Blob([text], { type: 'text/javascript' })));
}
const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const sleep = ms => new Promise(r => setTimeout(r, ms));
const enc = new TextEncoder();
const boards = new Map();
let builds = [];
let job = null;
let bridgeNext = false;

async function open(port) {
  if (boards.has(port)) return boards.get(port);
  const b = { port, info: port.getInfo(), state: 'opening', lines: [], buf: '', waiters: [], binMode: false, bin: new Uint8Array(0), binWait: null, t0: Date.now() };
  boards.set(port, b);
  b.mode = b.info.usbProductId === ROM_PID ? 'rom' : 'app';
  if (b.mode === 'rom') { b.state = 'open'; b.firmware = 'none running (ROM download mode)'; render(); boardOpened(b); return b; }
  render();
  try { await port.open({ baudRate: 115200 }); }
  catch (ex) { b.state = `does not open: ${ex.message}`; render(); return b; }
  b.state = 'open'; render();
  if (bridgeNext) { b.bridge = true; b.binMode = true; b.firmware = 'STM32 bridge (IAP)'; b.readDone = readLoop(b); boardOpened(b); return b; }
  b.readDone = readLoop(b);
  detect(b);
  boardOpened(b);
  return b;
}
async function readLoop(b) {
  b.reader = b.port.readable.getReader();
  const dec = new TextDecoder();
  try {
    for (;;) {
      const { value, done } = await b.reader.read();
      if (done) break;
      if (b.binMode) { const n = new Uint8Array(b.bin.length + value.length); n.set(b.bin); n.set(value, b.bin.length); b.bin = n; if (b.binWait) b.binWait(); continue; }
      b.buf += dec.decode(value, { stream: true });
      const parts = b.buf.split(/\r?\n/); b.buf = parts.pop();
      for (const line of parts) if (line.trim()) gotLine(b, line);
      render();
    }
  } catch (ex) { b.state = `closed: ${ex.message}`; }
  finally { try { b.reader.releaseLock(); } catch { /* gone */ } }
  if (!b.closing) { b.closing = true; await b.port.close().catch(() => {}); boards.delete(b.port); }
  render();
}
function gotLine(b, line) {
  b.lines.push(line); if (b.lines.length > 600) b.lines.shift();
  learn(b, line);
  for (const w of [...b.waiters]) if (w.test(line)) { b.waiters.splice(b.waiters.indexOf(w), 1); w.resolve(line); }
}
async function send(b, text) {
  if (!b.port.writable) throw new Error('The board left USB.');
  const w = b.port.writable.getWriter();
  try { await w.write(typeof text === 'string' ? enc.encode(text + '\n') : text); } finally { w.releaseLock(); }
}
function waitLine(b, re, ms) {
  return new Promise(resolve => {
    const w = { test: l => re.test(l), resolve };
    b.waiters.push(w);
    setTimeout(() => { const i = b.waiters.indexOf(w); if (i >= 0) { b.waiters.splice(i, 1); resolve(null); } }, ms);
  });
}
async function ask(b, cmd, re, ms = 3000) { const p = waitLine(b, re, ms); await send(b, cmd); return p; }
async function closeBoard(b) {
  b.closing = true;
  try { if (b.reader) await b.reader.cancel(); } catch { /* gone */ }
  await b.readDone;
  try { await b.port.close(); } catch (ex) { if (b.port.readable) say(`port close: ${ex.message}`); }
  boards.delete(b.port); render();
}

const openWaiters = [];
function boardOpened(b) { for (const w of [...openWaiters]) if (w.test(b)) { openWaiters.splice(openWaiters.indexOf(w), 1); w.resolve(b); } }
const BUTTON_STEP = 'Hold button while connecting USB, and for 3s after.';
let lastConnect = 0;
let disconnects = 0;
const plain = msg => Object.assign(new Error(msg), { plain: true });
function waitGone(ms) {
  const d0 = disconnects, t0 = Date.now();
  return new Promise(res => { const iv = setInterval(() => { if (disconnects !== d0 || Date.now() - t0 > ms) { clearInterval(iv); res(disconnects !== d0); } }, 100); });
}
async function personPowerCycle(test, button = false) {
  instruct('Unplug the USB cable and the battery from the board.');
  if (!await waitGone(60000)) {
    instruct('The board is still connected. Unplug the USB cable and the battery from the board.');
    if (!await waitGone(120000)) throw plain('The board stayed connected. Start the install again.');
  }
  for (let s = 3; s > 0; s--) { instruct(`Unplugged. Wait ${s} s.`); await sleep(1000); }
  const plug = button ? BUTTON_STEP : 'Plug the USB cable back in.';
  instruct(plug);
  let b = await waitBoard(test, 10000);
  while (!b && Date.now() - lastConnect < 6000) b = await waitBoard(test, 3000);
  if (!b) { instruct(plug + '\nIf the page does not show the board, press "Connect an Rx" and choose it.'); b = await waitBoard(test, 170000); }
  if (!b) throw plain('The board did not come back. Start the install again.');
  if (button) { instruct('You can let go of the button now.'); await sleep(1500); }
  instruct('');
  return b;
}
async function waitBoardHint(test, ms, choose) {
  let b = await waitBoard(test, 8000), waited = 8000;
  while (!b && Date.now() - lastConnect < 8000 && waited < ms) { b = await waitBoard(test, 4000); waited += 4000; }
  if (b) return b;
  instruct('If the page does not show the board, press "Connect an Rx" and choose it.');
  return waitBoard(test, Math.max(1000, ms - waited));
}
function waitBoard(test, ms, label) {
  return new Promise(resolve => {
    for (const b of boards.values()) if (b.state === 'open' && test(b)) return resolve(b);
    const w = { test, resolve }; openWaiters.push(w);
    if (label) say(label);
    setTimeout(() => { const i = openWaiters.indexOf(w); if (i >= 0) { openWaiters.splice(i, 1); resolve(null); } }, ms);
  });
}

function learn(b, line) {
  let m;
  if ((m = line.match(/^BOOT rover-firmware (\S+)/))) {
    Object.assign(b, { firmware: 'rover firmware', version: m[1], role: b.role || 'vehicle' });
    const r = line.match(/ board=(\S+)/); if (r) b.board = r[1];
    const a = line.match(/ mac=(\S+)/); if (a) b.mac = a[1];
  } else if ((m = line.match(/^OTA status .*table=(\S+) table_crc=([0-9a-fA-F]+)/))) Object.assign(b, { table: m[1], tableCrc: m[2].toLowerCase() });
  else if ((m = line.match(/Trail-Tracer-Rx-Monitor v(\S+) MAC (\S+)/))) {
    const rover = / role monitor/.test(line);
    Object.assign(b, { firmware: rover ? 'rover firmware' : 'Monitor firmware (old)', version: m[1], role: 'monitor', mac: m[2] });
  } else if (/^HB /.test(line)) { b.hb = true; if (!b.firmware) b.firmware = 'rover firmware'; if (!b.role) b.role = 'vehicle'; }
  else if ((m = line.match(/^STM ans .*cid=30r .*value=(\d+)/))) {
    const v = +m[1] >>> 0; b.stm = `app ${v & 255}.${(v >>> 8) & 255}.${(v >>> 16) & 255}`; b.stmLink = (v >>> 24) & 255;
  } else if (/^STM link v1/.test(line)) b.stm = 'app 1.x (link v1)';
}
async function readStm(b) {
  b.stm = null;
  await ask(b, 'stm read 30', /^STM ans .*cid=30r/, 3000);
  return b.stm || 'version not reported (an older STM32 app)';
}
async function detect(b) {
  b.detecting = true; render();
  const hb = await waitLine(b, /^HB |Trail-Tracer-Rx-Monitor v/, 2500);
  if (!hb) {
    const boot = await ask(b, 'boot', /^BOOT rover-firmware/, 2000);
    if (!boot) b.firmware = 'other firmware (no version report)';
  } else if (!b.version) await ask(b, b.role === 'monitor' ? 'V' : 'boot', /^BOOT rover-firmware|Trail-Tracer-Rx-Monitor v/, 2000);
  if (b.firmware === 'rover firmware' && b.role !== 'monitor' && !b.table) await ask(b, 'ota status', /^OTA status/, 2000);
  if (b.firmware === 'rover firmware' && b.role !== 'monitor' && !b.stm) b.stm = await readStm(b);
  b.detecting = false; render();
  boardOpened(b);
}

function say(text) { if (job) { job.log.push(`${new Date().toTimeString().slice(0, 8)}  ${text}`); job.prog = false; } render(); }
function progress(text) { if (!job) return; if (job.prog) job.log.pop(); say(text); job.prog = true; }
function step(id) {
  if (!job || !job.steps) return;
  const s = job.steps.find(x => x.id === id);
  if (!s) return;
  for (const x of job.steps) if (x.state === 'now') x.state = 'done';
  s.state = 'now'; say(`STEP ${id}: ${s.text}`);
}
function stage(text, pct = null) { if (!job) return; job.stage = text; job.pct = pct; say('STEP: ' + text); }
function setPct(p) { if (job && job.pct !== p) { job.pct = p; render(); } }
async function run(name, fn) {
  if (job && !job.done) return alert('A job runs. Wait until it ends.');
  job = { name, log: [], done: false, stage: 'Starting', pct: null, result: null, devOpen: false };
  say(pageVer);
  say(`${name}: start`);
  try {
    const r = await fn(); job.result = { ok: true, text: typeof r === 'string' ? r : 'Done.' }; say(`${name}: done`);
    for (const x of job.steps || []) if (x.state !== 'skip') x.state = 'done';
  } catch (ex) {
    job.result = { ok: false, text: ex.plain ? ex.message : 'The install stopped. Start it again. If it stops again, contact support.' };
    say(`${name}: STOPPED: ${ex.message}`); instruct('');
    for (const x of job.steps || []) if (x.state === 'now') x.state = 'fail';
  }
  job.done = true; job.pct = null; render();
}
async function fetchBuild(build, path) {
  const f = build.m.files.find(x => x.path === path);
  if (!f) throw new Error(`${path} is not in the manifest`);
  const data = new Uint8Array(await (await fetch(build.base + path)).arrayBuffer());
  const sha = await sha256hex(data);
  if (sha !== f.sha256) throw new Error(`${path}: SHA-256 ${sha.slice(0, 12)} is not ${f.sha256.slice(0, 12)} (manifest)`);
  return data;
}
async function sha256hex(data) { return [...new Uint8Array(await crypto.subtle.digest('SHA-256', data))].map(x => x.toString(16).padStart(2, '0')).join(''); }
const CRC32 = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
function crc32(d) { let c = 0xFFFFFFFF; for (const x of d) c = CRC32[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; }
const hex = d => [...d].map(x => x.toString(16).padStart(2, '0')).join('');

async function otaUpdate(b0, build) {
  stage('Checking the firmware file');
  const data = await fetchBuild(build, build.m.esp32.usb_update.file);
  if (data[0] !== 0xE9 || (data[12] | data[13] << 8) !== 2 || new DataView(data.buffer).getUint32(0x20, true) !== 0xABCD5432) throw new Error('firmware.bin is not an ESP32-S2 app image');
  const sha = await sha256hex(data);
  let b = b0;
  const status = async () => { const r = await ask(b, 'ota status', /^OTA status/, 5000); return r; };
  step('esp');
  stage('Sending the new firmware to the board', 0);
  for (let attempt = 1; ; attempt++) {
    const r = await ask(b, `ota begin ${data.length} ${sha} confirm crc`, /^OTA (begin ok|error)/, 8000);
    say(`attempt ${attempt}: ${r}`);
    if (!r || !r.startsWith('OTA begin ok')) throw new Error('the board refused "ota begin"');
    const crc = /crc=1/.test(r);
    let off = 0, resends = 0, bad = false;
    while (off < data.length) {
      const chunk = data.subarray(off, off + 512);
      const line = `ota w ${off} ${hex(chunk)}` + (crc ? ` ${crc32(chunk).toString(16).padStart(8, '0')}` : '');
      const a = await ask(b, line, /^OTA (ok|error)/, 5000);
      if (a === `OTA ok ${off + chunk.length}`) { off += chunk.length; setPct(Math.floor(100 * off / data.length)); progress(`ESP32 ${Math.floor(100 * off / data.length)} %  (${off} of ${data.length} B)`); continue; }
      const st = await status(), w = st && /active=1/.test(st) ? +st.split('written=')[1].split(/\s/)[0] : null;
      if (w === off + chunk.length) { off = w; continue; }
      if (w === off && ++resends <= 20) { say(`line at ${off} not taken (${a}); sent again`); continue; }
      say(`answer at ${off}: ${a}; board count ${w}: abort, start again`); await send(b, 'ota abort'); bad = true; break;
    }
    if (!bad) break;
    if (attempt === 3) throw new Error('3 attempts failed');
  }
  step('start');
  stage('The board checks the new firmware and restarts');
  const end = await ask(b, 'ota end', /^OTA (done|error)/, 15000);
  say(String(end || 'no answer to "ota end" (the board can restart before it): checking'));
  if (end && !end.startsWith('OTA done')) throw new Error('"ota end" failed');
  const oldPort = b.port; await closeBoard(b);
  b = await waitBoard(x => x.mode === 'app' && x.port !== oldPort || x.t0 > Date.now() - 100000 && x.mode === 'app' && x.lines.some(l => /^BOOT rover-firmware/.test(l)), 100000,
    'The board restarts with the new firmware; the page opens it again.');
  if (!b) throw new Error('The board did not come back on USB within 100 s. It returns to its old firmware by itself after 2 min.');
  stage('Confirming the new firmware');
  await sleep(1500);
  let st = null;
  for (let i = 0; i < 6 && !st; i++) { st = await status(); if (!st) await sleep(4000); }
  say(String(st));
  if (!st || !/state=1/.test(st)) throw new Error('The board did not start the new firmware. It still runs its old firmware.');
  say(String(await ask(b, 'ota confirm', /^OTA/, 5000)));
  say(String(await status()));
  say(`ESP32 now: ${b.firmware} ${b.version || ''}`);
  return `Installed. The ESP32 now runs ${b.firmware} ${b.version || ''}.`;
}

function nvsIapholdPage() {
  const crc = d => { let c = 0; for (const x of d) c = CRC32[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; };
  const p = new Uint8Array(4096).fill(0xFF), v = new DataView(p.buffer);
  v.setUint32(0, 0xFFFFFFFE, true); v.setUint32(4, 0, true); p[8] = 0xFE;
  v.setUint32(28, crc(p.subarray(4, 28)), true);
  p[32] = 0xFA;
  [[0, 'wfrx_bench'], [1, 'iaphold']].forEach(([ns, key], i) => {
    const e = p.subarray(64 + 32 * i, 96 + 32 * i);
    e.set([ns, 0x01, 1, 0xFF]); e.fill(0, 8, 24); e.set(enc.encode(key), 8); e[24] = 1;
    new DataView(e.buffer, e.byteOffset).setUint32(4, crc(Uint8Array.from([...e.subarray(0, 4), ...e.subarray(8, 32)])), true);
  });
  return p;
}
function nvsPartition(table) {
  for (let i = 0; i + 32 <= table.length; i += 32) {
    const v = new DataView(table.buffer, table.byteOffset + i);
    if (v.getUint16(0, true) !== 0x50AA) break;
    if (table[i + 2] === 0x01 && table[i + 3] === 0x02) return { offset: v.getUint32(4, true), size: v.getUint32(8, true) };
  }
  return null;
}
function md5hex(d) {
  const K = new Uint32Array(64), S = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];
  for (let i = 0; i < 64; i++) K[i] = Math.floor(Math.abs(Math.sin(i + 1)) * 2 ** 32) >>> 0;
  const n = ((d.length + 8) >>> 6) + 1, m = new Uint8Array(n * 64);
  m.set(d); m[d.length] = 0x80;
  const v = new DataView(m.buffer); v.setUint32(n * 64 - 8, (d.length * 8) >>> 0, true); v.setUint32(n * 64 - 4, Math.floor(d.length / 0x20000000), true);
  let a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
  for (let o = 0; o < m.length; o += 64) {
    let a = a0, b = b0, c = c0, e = d0;
    for (let i = 0; i < 64; i++) {
      let f, g;
      if (i < 16) { f = (b & c) | (~b & e); g = i; } else if (i < 32) { f = (e & b) | (~e & c); g = (5 * i + 1) % 16; }
      else if (i < 48) { f = b ^ c ^ e; g = (3 * i + 5) % 16; } else { f = c ^ (b | ~e); g = (7 * i) % 16; }
      const s = S[(i >> 4) * 4 + (i & 3)], x = (a + f + K[i] + v.getUint32(o + g * 4, true)) >>> 0;
      a = e; e = c; c = b; b = (b + ((x << s) | (x >>> (32 - s)))) >>> 0;
    }
    a0 = (a0 + a) >>> 0; b0 = (b0 + b) >>> 0; c0 = (c0 + c) >>> 0; d0 = (d0 + e) >>> 0;
  }
  const out = new DataView(new ArrayBuffer(16)); [a0, b0, c0, d0].forEach((w, i) => out.setUint32(i * 4, w, true));
  return [...new Uint8Array(out.buffer)].map(x => x.toString(16).padStart(2, '0')).join('');
}
async function watchdogReset(loader) {
  say('watchdog reset (esptool watchdog_reset)');
  await loader.writeReg(0x3F4080AC, 0x50D83AA1);
  await loader.writeReg(0x3F408098, 2000);
  await loader.writeReg(0x3F408094, ((1 << 31) | (5 << 28) | (1 << 8) | 2) >>> 0);
  await loader.writeReg(0x3F4080AC, 0).catch(() => {});
  await sleep(500);
}
async function withLoader(b, fn) {
  const { ESPLoader, Transport } = await loadEsptool();
  const term = { clean() {}, writeLine: t => say('esptool: ' + t), write: t => { if (t.trim()) say('esptool: ' + t.trim()); } };
  if (b.port.readable) await b.port.close().catch(() => {});
  const transport = new Transport(b.port, true);
  try {
    const loader = new ESPLoader({ transport, baudrate: 115200, romBaudrate: 115200, terminal: term, debugLogging: true });
    await loader.detectChip('no_reset');
    return await fn(loader, transport);
  } finally { await transport.disconnect().catch(() => {}); if (b.port.readable) await b.port.close().catch(() => {}); boards.delete(b.port); render(); }
}
let mergedBridge = null;
const S2 = { STRAP: 0x3F404038, SPI_BOOT: 0x8, OPTION1: 0x3F408128, FORCE_DL: 0x1 };
async function romLoad(b0, build, erase, stmBuild = null) {
  stage('Checking the firmware files');
  const fl = build ? build.m.esp32.full_load : { parts: [] }, blobs = [];
  for (const x of fl.parts) { blobs.push({ data: await fetchBuild(build, x.file), address: parseInt(x.offset, 16) }); say(`${x.file} at ${x.offset}: ${blobs.at(-1).data.length} B, SHA-256 checked`); }
  const fillRest = () => {
    const SECTOR = 0x1000, SIZE = 0x400000, used = blobs.map(x => [Math.floor(x.address / SECTOR) * SECTOR, Math.ceil((x.address + x.data.length) / SECTOR) * SECTOR]).sort((a, c) => a[0] - c[0]);
    let at = 0;
    for (const [s, e] of [...used, [SIZE, SIZE]]) {
      if (s > at) { blobs.unshift({ data: new Uint8Array(s - at).fill(0xFF), address: at }); say(`erase 0x${at.toString(16)} to 0x${s.toString(16)}`); }
      at = Math.max(at, e);
    }
  };
  let b = b0;
  if (b.mode !== 'rom') {
    step('loader');
    stage('Preparing the board');
    const oldPort = b.port;
    if (b.firmware === 'rover firmware') {
      const a = await ask(b, 'dld', /^(BENCH restart into ROM|dld refused)/, 1500);
      say(`"dld": ${a || 'no answer'}`);
      if (a && /refused/.test(a)) throw plain('The vehicle moves, so the board refuses the install. Stop the vehicle, then start the install again.');
      await closeBoard(b).catch(() => {}); b = null;
    }
    else {
      await closeBoard(b).catch(() => {});
      say('1200-baud touch');
      try { await oldPort.open({ baudRate: 1200 }); await oldPort.setSignals({ dataTerminalReady: false }).catch(() => {}); await sleep(100); await oldPort.close(); }
      catch (ex) { say(`1200-baud touch: ${ex.message}`); }
      b = await waitBoard(x => x.mode === 'rom', 8000, 'The board restarts into ROM download mode.');
      if (!b && !oldPort.gone) {
        say('no ROM download mode 8 s after the touch: USB reset');
        try {
          await oldPort.open({ baudRate: 115200 });
          for (const [d, r] of [[false, true], [true, true], [true, false], [false, false]]) { await oldPort.setSignals({ dataTerminalReady: d, requestToSend: r }).catch(() => {}); await sleep(60); }
          await oldPort.close();
        } catch (ex) { say(`USB reset: ${ex.message}`); }
      }
    }
    stage('Preparing the board');
    if (!b) b = await waitBoardHint(x => x.mode === 'rom', 60000, '"ESP32-S2"');
    if (!b) {
      stage('Waiting for you');
      say('no ROM download mode by itself: the button');
      b = await personPowerCycle(x => x.mode === 'rom', true);
    }
    instruct('');
    if (b.port === oldPort) say('same port object for the load');
  }
  step('esp');
  stage('Connecting to the board');
  const { ESPLoader, Transport } = await loadEsptool();
  const term = { clean() {}, writeLine: t => say('esptool: ' + t), write: t => { if (t.trim()) say('esptool: ' + t.trim()); } };
  if (b.port.readable) await b.port.close().catch(() => {});
  const transport = new Transport(b.port, true);
  let entry = null;
  try {
  const loader = new ESPLoader({ transport, baudrate: 115200, romBaudrate: 115200, terminal: term, debugLogging: true });
  await loader.detectChip('no_reset');
  say(`chip: ${await loader.chip.getChipDescription(loader)}`);
  const strap = await loader.readReg(S2.STRAP), opt1 = await loader.readReg(S2.OPTION1);
  const byButton = (strap & S2.SPI_BOOT) === 0 && (opt1 & S2.FORCE_DL) === 0;
  say(`download mode entry: ${byButton ? 'button "0" (watchdog reset)' : 'firmware (RTS reset)'}; strap 0x${(strap >>> 0).toString(16)}, option1 0x${(opt1 >>> 0).toString(16)}`);
  if (byButton) {
    const held = async () => ((await loader.readReg(0x3F40403C).catch(() => 1)) & 1) === 0;
    if (await held()) {
      instruct('Let go of the button.');
      for (let i = 0; i < 200 && await held(); i++) await sleep(300);
      instruct('');
    }
  }
  let merged = false;
  const tb = blobs.find(x => x.address === 0x8000), nvs = tb && nvsPartition(tb.data);
  if (build && stmBuild) {
    if (nvs) {
      blobs.push({ data: nvsIapholdPage(), address: nvs.offset });
      if (nvs.size > 4096) blobs.push({ data: new Uint8Array(nvs.size - 4096).fill(0xFF), address: nvs.offset + 4096 });
      merged = true; say(`NVS at 0x${nvs.offset.toString(16)}: iaphold = 1, so the first start of the new firmware opens the STM32 bridge`);
    } else say('no NVS partition in the partition table: the STM32 update takes the normal path');
  }
  let tableSame = false;
  if (tb) {
    const fm = await loader.flashMd5sum(tb.address, tb.data.length).catch(() => null);
    tableSame = !!fm && fm === md5hex(tb.data);
    say(`partition table: ${tableSame ? 'the same' : 'differs: full write'}`);
  }
  if (erase && (!build || !tableSame)) fillRest();
  stage('Checking the board', null);
  for (let i = blobs.length - 1; i >= 0; i--) {
    const x = blobs[i];
    const fm = await loader.flashMd5sum(x.address, x.data.length).catch(ex => { say(`MD5 0x${x.address.toString(16)}: ${ex.message}`); return null; });
    if (fm && fm === md5hex(x.data)) { say(`0x${x.address.toString(16)} (${x.data.length} B): equal, not written`); blobs.splice(i, 1); }
  }
  loader.FLASH_WRITE_SIZE = 0x400; if (loader.chip) loader.chip.FLASH_WRITE_SIZE = 0x400;
  const sizes = blobs.map(x => x.data.length), total = sizes.reduce((a, c) => a + c, 0);
  stage('Writing the ESP32 firmware', 0);
  const bstr = d => { let s = ''; for (let i = 0; i < d.length; i += 0x8000) s += String.fromCharCode.apply(null, d.subarray(i, i + 0x8000)); return s; };
  if (blobs.length) await loader.writeFlash({ fileArray: blobs.map(x => ({ data: bstr(x.data), address: x.address })), flashSize: '4MB', flashMode: 'dio', flashFreq: '80m',
    eraseAll: false, compress: true, reportProgress: (i, w, t) => { setPct(Math.floor(100 * (sizes.slice(0, i).reduce((a, c) => a + c, 0) + sizes[i] * w / t) / total)); progress(`file ${i + 1} of ${blobs.length}: ${Math.floor(100 * w / t)} %`); } });
  else say('every region equal: nothing written');
  if (build) step('start');
  if (build) {
    stage('Starting the new firmware');
    await loader.writeReg(S2.OPTION1, 0, S2.FORCE_DL).catch(ex => say(`force-download bit not cleared: ${ex.message}`));
    await watchdogReset(loader);
  }
  entry = byButton ? 'button' : 'firmware';
  } finally { await transport.disconnect().catch(() => {}); if (b.port.readable) await b.port.close().catch(() => {}); }
  boards.delete(b.port); render();
  if (!build) return 'The ESP32 is erased: 4 MB, nothing kept. It runs no firmware and stays in bootloader mode until a Full install. Press "Connect an Rx" to see it.';
  const rover = build.m.esp32.usb_update, back = x => x.mode === 'app' && !x.detecting && (rover ? x.firmware === 'rover firmware' && !!x.version : !!x.firmware);
  b = null;
  const appTest = merged ? (x => x.mode === 'app' && x.bridge) : back;
  stage('The board restarts with the new firmware');
  if (merged) bridgeNext = true;
  try {
    for (let i = 0; i < 2 && !b; i++) {
      const nb = await waitBoardHint(x => appTest(x) || x.mode === 'rom', 45000, 'the board');
      if (nb && nb.mode === 'rom') {
        say('the board came back in ROM download mode');
        if (entry === 'button') { instruct('Let go of the button.'); await sleep(3000); instruct(''); }
        await withLoader(nb, async (loader, tr) => {
          await loader.writeReg(S2.OPTION1, 0, S2.FORCE_DL).catch(() => {});
          say('fallback: RTS reset'); await tr.setRTS(true); await sleep(200); await tr.setRTS(false); await sleep(200);
        });
        continue;
      }
      b = nb;
    }
  } finally { bridgeNext = false; }
  if (b && merged) { mergedBridge = b; return 'Installed. The ESP32 firmware is written.'; }
  if (!b && merged) {
    say('no new firmware after the reset: the fallback, a power cycle');
    bridgeNext = true;
    stage('Waiting for you');
    try { mergedBridge = await personPowerCycle(x => x.mode === 'app' && x.bridge); } finally { bridgeNext = false; }
    return 'Installed. The ESP32 firmware is written.';
  }
  if (!b) {
    say('no new firmware after the reset: the fallback, a power cycle');
    stage('Waiting for you');
    b = await personPowerCycle(back);
  }
  say(`ESP32 now: ${b.firmware} ${b.version || ''}`);
  return rover ? `Installed. The ESP32 now runs ${b.firmware} ${b.version || ''}.` : `Installed. The ESP32 now runs ${b.firmware}.`;
}

function crc16(d) { let c = 0xFFFF; for (const x of d) { c ^= x << 8; for (let k = 0; k < 8; k++) c = c & 0x8000 ? ((c << 1) ^ 0x8005) & 0xFFFF : (c << 1) & 0xFFFF; } return c; }
function frame(cid, payload = new Uint8Array(0)) {
  const f = new Uint8Array(6 + payload.length), v = new DataView(f.buffer);
  v.setUint16(0, 6 + payload.length, true); v.setUint16(2, cid, true); f.set(payload, 4);
  v.setUint16(4 + payload.length, crc16(f.subarray(0, 4 + payload.length)), true);
  return f;
}
async function readFrame(b, ms) {
  const end = Date.now() + ms;
  for (;;) {
    while (b.bin.length >= 4) {
      const v = new DataView(b.bin.buffer, b.bin.byteOffset), len = v.getUint16(0, true), cid = v.getUint16(2, true);
      if (len < 6 || len > 1100 || cid < 1 || cid > 5) { b.bin = b.bin.slice(1); b.skipped = (b.skipped || 0) + 1; continue; }
      if (b.bin.length < len) break;
      const f = b.bin.slice(0, len), crc = new DataView(f.buffer).getUint16(len - 2, true);
      if (crc !== crc16(f.subarray(0, len - 2))) { b.bin = b.bin.slice(1); b.skipped = (b.skipped || 0) + 1; continue; }
      b.bin = b.bin.slice(len);
      if (b.skipped) { say(`${b.skipped} bytes before the IAP frame skipped (ESP32 text)`); b.skipped = 0; }
      return { cid, p: f.slice(4, len - 2) };
    }
    const left = end - Date.now();
    if (left <= 0) throw new Error(`no IAP answer in ${ms} ms`);
    await new Promise(r => { b.binWait = r; setTimeout(r, Math.min(left, 200)); });
  }
}
async function stmLoad(b0, build) {
  step('stm');
  stage('Checking the STM32 files');
  const header = await fetchBuild(build, build.m.stm32.update.header);
  const image = await fetchBuild(build, build.m.stm32.update.image);
  const hv = new DataView(header.buffer), n = image.length / 1040;
  if (header.length !== 52 || image.length % 1040 || n !== hv.getUint16(18, true)) throw new Error(`header ${header.length} B, image ${image.length} B, chunk count ${hv.getUint16(18, true)}`);
  say(`STM32 image: ${n} chunks`);
  let b = b0;
  stage('Preparing the board for the STM32 update');
  const set = await ask(b, 'stm iaphold', /STM iaphold (set|:)/, 10000);
  say(String(set));
  if (set && !/STM iaphold set/.test(set)) throw new Error(set);
  if (!set) throw new Error('no "STM iaphold set" (rover firmware 0.5.26 or later needed)');
  const oldPort = b.port; await closeBoard(b);
  bridgeNext = true;
  stage('Waiting for you');
  try { b = await personPowerCycle(x => x.mode === 'app' && x.port !== oldPort && x.bridge); } finally { bridgeNext = false; }
  return stmBridge(b, build, header, image);
}
const NO_LOADER = 'The STM32 on this board cannot take an update over USB. Nothing was written to it. Contact support.';
async function stmBridge(b, build, header = null, image = null) {
  step('stm');
  if (!header) {
    header = await fetchBuild(build, build.m.stm32.update.header);
    image = await fetchBuild(build, build.m.stm32.update.image);
  }
  const n = image.length / 1040;
  const t0 = Date.now(), dec = new TextDecoder();
  for (;;) {
    const txt = dec.decode(b.bin), k = txt.indexOf('no text until it ends');
    if (k >= 0) {
      const e = txt.indexOf('\n', k);
      if (e >= 0) {
        const line = txt.slice(txt.lastIndexOf('STM bridge on', k), e).trim();
        say(line);
        if (/HELLO not answered/.test(line)) { say('no HELLO answer (bridge line)'); throw plain(NO_LOADER); }
        b.bin = b.bin.slice(new TextEncoder().encode(txt.slice(0, e + 1)).length); break;
      }
    }
    if (/BOOT rover-firmware/.test(txt)) throw new Error('the board started normally, not as an STM32 bridge: the update flag was gone at this power-on. Start the STM32 install again.');
    if (Date.now() - t0 > 4000) { say('no "STM bridge on" line (sent before the port opened); HELLO anyway'); b.bin = new Uint8Array(0); break; }
    await sleep(100);
  }
  stage('Checking the STM32');
  await send(b, frame(1));
  const h = await readFrame(b, 2000).then(f => f.cid === 1 && f.p.length >= 7 ? f : Promise.reject(new Error('not a HELLO answer'))).catch(() => { say('no HELLO answer'); throw plain(NO_LOADER); });
  say(`HELLO: boot status ${h.p[0]}, app ${h.p[1]}.${h.p[2]}.${h.p[3]}, IAP ${h.p[4]}.${h.p[5]}.${h.p[6]}`);
  let r;
  if (h.p[0] === 0 && `${h.p[1]}.${h.p[2]}.${h.p[3]}` === build.m.version) {
    say(`the STM32 app is ${build.m.version} already: BOOT only, no write`);
  } else {
  await send(b, frame(2, header));
  r = await readFrame(b, 2000);
  if (r.cid !== 2 || r.p[0]) throw new Error(`BEGIN_UPDATE refused: answer cid ${r.cid}, status ${r.p[0]}`);
  stage('Writing the STM32 app', 0);
  for (let i = 0; i < n; i++) {
    const pl = new Uint8Array(2 + 1040); new DataView(pl.buffer).setUint16(0, i, true); pl.set(image.subarray(i * 1040, (i + 1) * 1040), 2);
    await send(b, frame(3, pl));
    r = await readFrame(b, 6000);
    const idx = r.p[0] | r.p[1] << 8;
    if (r.cid !== 3 || idx !== i || r.p[2]) throw new Error(`chunk ${i}: answer cid ${r.cid}, echoed ${idx}, reason ${r.p[2]}`);
    setPct(Math.floor(100 * (i + 1) / n)); progress(`STM32 chunk ${i + 1} of ${n}`);
  }
  stage('Starting the new STM32 app');
  await send(b, frame(4));
  r = await readFrame(b, 3000);
  if (r.cid === 1) { say(`boot status after FINALIZE: ${r.p[0]}, app ${r.p[1]}.${r.p[2]}.${r.p[3]}`); r = await readFrame(b, 3000); }
  if (r.cid !== 4 || r.p[0]) throw new Error(`FINALIZE failed: answer cid ${r.cid}, status ${r.p[0]}`);
  }
  await send(b, frame(5));
  r = await readFrame(b, 3000);
  if (r.cid !== 5 || r.p[0]) throw new Error(`BOOT failed: answer cid ${r.cid}, status ${r.p[0]}`);
  say('STM32 app loaded and started. The ESP32 leaves the bridge 10 s after the last byte.');
  b.binMode = false; b.buf = '';
  if (!await waitLine(b, /STM bridge off/, 30000)) say('no "STM bridge off" line within 30 s');
  step('check');
  stage('Reading the new versions');
  b.bridge = false;
  Object.assign(b, { firmware: null, version: null, role: null, stm: null, stmLink: null, table: null, tableCrc: null, hb: false });
  await detect(b);
  say(`ESP32 now: ${b.firmware} ${b.version || ''}; STM32 now: ${b.stm}`);
  return /^app /.test(b.stm) ? `Installed. The STM32 now runs ${b.stm}.` : 'Installed and started. The board did not report the STM32 version yet.';
}

const setHtml = (el, html) => { if (el._html !== html) { el._html = html; el.innerHTML = html; return true; } return false; };
function setLog(details, pre, text) {
  if (!details.open || pre._text === text) return;
  const atEnd = pre.scrollHeight - pre.scrollTop - pre.clientHeight < 4;
  pre._text = text; pre.textContent = text;
  if (atEnd) pre.scrollTop = pre.scrollHeight;
}
let boardSeq = 0;
function boardCard(b) {
  if (b.card) return b.card;
  b.id = ++boardSeq;
  const c = document.createElement('div');
  c.className = 'card';
  c.innerHTML = `<b class="title"></b><div class="kv"></div><div class="offers"></div><div class="refbox"></div>
    <details class="usb"><summary>USB lines</summary><pre class="log"></pre></details>`;
  c.querySelector('.usb').ontoggle = () => render();
  c.querySelector('.offers').onclick = e => {
    const el = e.target.closest('button[data-o]');
    if (!el || (job && !job.done)) return;
    const o = b.offers[+el.dataset.o];
    if (o && confirm(`${o.l1}\n${o.l2}\n\n${(o.plan || []).map(([x, y]) => `${x}: ${y}`).join('\n')}\n\n${o.l3}`)) run(o.l1, o.go);
  };
  b.card = c;
  return c;
}
function render() {
  $('release').textContent = builds.length ? `Builds: ${builds.length} (${[...new Set(builds.map(x => x.src))].join(', ')})` : 'No builds found.';
  $('steps').hidden = !stepText; $('steps').textContent = stepText;
  drawRefMenu();
  const idle = !job || job.done, list = $('boards'), all = [...boards.values()];
  for (const el of [...list.children]) if (!all.some(b => b.card === el)) el.remove();
  all.forEach((b, k) => {
    const c = boardCard(b);
    if (list.children[k] !== c) list.insertBefore(c, list.children[k] || null);
    const rows = b.mode === 'rom' ? [['state', 'bootloader mode detected'], ['details', 'shown after the install: no firmware runs now'], ['port', b.state]]
      : b.detecting ? [['state', 'reading the board'], ['port', b.state]]
      : [['ESP32', (b.firmware || '-') + (b.version ? ` ${b.version}` : '')], ['role', b.role || '-'],
        ['STM32', b.stm ? b.stm + (b.stmLink ? `, link v${b.stmLink}` : '') : '-'], ['board revision', b.board || (b.firmware ? 'RX_REVA (not reported)' : '-')],
        ...(b.mac ? [['radio address', b.mac]] : []), ['port', b.state]];
    setHtml(c.querySelector('.title'), `Rx ${k + 1}`);
    setHtml(c.querySelector('.kv'), rows.map(([x, y]) => `<span class="k">${esc(x)}</span><span>${esc(y)}</span>`).join(''));
    const offers = idle && b.state === 'open' && !b.detecting ? offersFor(b) : [];
    b.offers = offers.erase ? [...offers, offers.erase] : offers;
    const btn = offers.button ? `<div class="card steps">To install firmware, unplug the board.
${BUTTON_STEP}</div>` : '';
    setHtml(c.querySelector('.offers'), btn + b.offers.map((o, i) => `<button class="offer" data-o="${i}"><b>${esc(o.l1)}</b><span>${esc(o.l2)}</span><span>${esc(o.l3)}</span><div class="kv plan">${(o.plan || []).map(([x, y]) => `<span class="k">${esc(x)}</span><i>${esc(y)}</i>`).join('')}</div></button>`).join(''));
    const refused = offers.refused || [], rb = c.querySelector('.refbox'), wasOpen = !!rb.querySelector('details')?.open;
    if (setHtml(rb, refused.length ? `<details class="refused"><summary>Not compatible with this board (${refused.length})</summary>${refused.map(r => `<div class="ref"><b>${esc(r.l1)}</b><span>${esc(r.l2)}</span><span>${esc(r.reason)}</span></div>`).join('')}</details>` : '') && wasOpen && rb.querySelector('details')) rb.querySelector('details').open = true;
    setLog(c.querySelector('.usb'), c.querySelector('.usb pre'), b.lines.slice(-200).join('\n'));
  });
  if (!all.length && !list.querySelector('.note')) list.innerHTML = '<p class="note">No Rx connected.</p>';
  const jp = $('job');
  jp.hidden = !job;
  if (!job) return;
  if (jp._job !== job) {
    jp._job = job; jp._html = null;
    jp.innerHTML = `<b class="name"></b><div class="body"></div><details class="dev"><summary>Developer log</summary><pre class="log"></pre></details>`;
    jp.querySelector('.dev').ontoggle = () => render();
  }
  const res = job.result, head = res ? (res.ok ? 'Done' : 'Stopped') : job.stage, bar = `<div class="bar"><div style="width: ${job.pct}%"></div></div><div class="note">${job.pct} %</div>`;
  setHtml(jp.querySelector('.name'), esc(job.name));
  setHtml(jp.querySelector('.body'), (job.steps ? `<ol class="jobsteps">${job.steps.map(s => `<li class="${s.state}"><span>${{ wait: '·', now: '▶', done: '✓', fail: '✗' }[s.state] || '·'}</span>${esc(s.text)}${s.state === 'now' && !res ? `<div class="note">${esc(job.stage)}</div>${job.pct !== null ? bar : ''}` : ''}</li>`).join('')}</ol>` : '')
    + `<div class="stage ${res ? (res.ok ? 'good' : 'bad') : ''}">${esc(job.steps && !res ? '' : head)}</div>`
    + (!job.steps && job.pct !== null && !res ? bar : '')
    + (res ? `<div class="result ${res.ok ? 'good' : 'bad'}">${esc(res.text)}</div>` : ''));
  setLog(jp.querySelector('.dev'), jp.querySelector('.dev pre'), job.log.join('\n'));
}
let stepText = '';
function instruct(text) { stepText = text; if (text) say('USER: ' + text); render(); }

function vcmp(a, b) { const x = String(a).split(/[.+ ]/).map(Number), y = String(b).split(/[.+ ]/).map(Number); for (let i = 0; i < 3; i++) if ((x[i] || 0) !== (y[i] || 0)) return (x[i] || 0) - (y[i] || 0); return 0; }
function change(from, to) { if (!from) return 'Replaces an unknown version.'; const c = vcmp(to, from); return c === 0 ? `Same version (${from}).` : c > 0 ? `Replaces ${from}.` : `Older than the board (${from} to ${to}).`; }
const srcLine = (m, rel) => rel && rel === m.source.branch
  ? `${m.source.repo.split('/').pop()}, commit ${m.source.commit.slice(0, 7)}, release ${rel}`
  : `${m.source.repo.split('/').pop()}, branch ${m.source.branch}, commit ${m.source.commit.slice(0, 7)}`
  + (rel ? `, release ${rel}` : rel === false ? ', development branch' : '');
let refSel = (() => { try { return JSON.parse(localStorage.getItem('tt.ref') || 'null'); } catch { return null; } })();
let refInfo = {};
let allBuilds = [];
function refFromUrl() {
  const q = new URLSearchParams(location.search), fw = q.get('fw');
  if (!fw || !fw.includes('/')) return null;
  const i = fw.indexOf('/'), short = fw.slice(0, i), ref = fw.slice(i + 1), commit = (q.get('commit') || '').toLowerCase();
  const b = allBuilds.find(x => x.m.source.repo.split('/').pop() === short && x.m.source.branch === ref);
  return b ? { repo: b.m.source.repo, ref, ...(/^[0-9a-f]{7,40}$/.test(commit) ? { commit } : {}) } : null;
}
function urlFor(sel, pin) {
  const u = new URL(location.href), enc = s => s.split('/').map(encodeURIComponent).join('/');
  u.search = `?fw=${enc(sel.repo.split('/').pop() + '/' + sel.ref)}` + (pin ? `&commit=${encodeURIComponent(pin)}` : '');
  return u.toString();
}
function offerBuilds(sel) {
  if (sel === '*' || !sel.commit) return builds;
  const pinned = allBuilds.filter(x => x.m.source.repo === sel.repo && x.m.source.branch === sel.ref && x.m.source.commit.startsWith(sel.commit));
  return pinned.length ? [...pinned, ...builds.filter(x => !(x.m.source.repo === sel.repo && x.m.source.branch === sel.ref))] : builds;
}
const stmOnly = repo => builds.filter(x => x.m.source.repo === repo).every(x => x.m.chip === 'stm32g070');
const repoName = repo => ({ Trail_Tracer_Mower: 'Trail Tracer Mower', 'Trail-Tracer': 'Trail Tracer V1.0' })[repo.split('/').pop()]
  || repo.split('/').pop().replace(/[_-]+/g, ' ');
const vtext = v => /^\d/.test(v) ? 'v' + v : v;
function refsOf() {
  const m = new Map();
  for (const x of builds) {
    const s = x.m.source;
    if (stmOnly(s.repo)) continue;
    const k = s.repo + ' ' + s.branch;
    const r = m.get(k) || { repo: s.repo, ref: s.branch, release: x.release, commit: s.commit, t: s.built_utc, parts: [] };
    if (s.built_utc > r.t) Object.assign(r, { commit: s.commit, t: s.built_utc });
    if (x.m.chip === 'esp32s2' && x.m.esp32.usb_update) r.ver = vtext(x.m.version);
    r.parts.push(x.m.chip === 'stm32g070' ? `STM32 app ${vtext(x.m.version)}` : x.m.esp32.usb_update ? `ESP32 firmware ${vtext(x.m.version)}` : 'ESP32 firmware');
    m.set(k, r);
  }
  const out = [...m.values()].map(r => {
    const info = refInfo[r.repo + ' ' + r.ref] || {}, tag = info.tag !== undefined ? info.tag : r.release === r.ref;
    const kind = tag ? (/-test\.\d+$/.test(r.ref) ? 'test' : 'release') : 'dev';
    const tv = (r.ref.match(/v?(\d+\.\d+\.\d+)/) || [])[1];
    const repo = r.repo.split('/').pop();
    const title = (info.title && (info.title + (r.ver ? ' ' + r.ver : ''))) || (kind === 'release' ? `Release ${tv ? 'v' + tv : r.ref}` : kind === 'test' ? `Test release ${tv ? 'v' + tv : r.ref}`
      : r.ref === 'main' ? (repo === 'Trail_Tracer_Mower' ? 'Development build (latest, untested)' : repo === 'Trail-Tracer' ? 'Original V1.0 firmware line' : 'Development build')
      : `Test build: ${r.ref}`);
    return { ...r, kind, title, description: info.description || [...new Set(r.parts)].join(', '), git: `${r.ref}, ${r.commit.slice(0, 7)}` };
  });
  const rank = r => ({ release: 0, test: 1, dev: 2 })[r.kind] * 2 + (r.kind === 'dev' && r.ref !== 'main' ? 1 : 0);
  return out.sort((a, b) => a.repo.localeCompare(b.repo) || rank(a) - rank(b) || b.t.localeCompare(a.t));
}
function currentRef() {
  if (refSel === '*') return '*';
  const refs = refsOf(), fromUrl = refFromUrl();
  if (fromUrl) return fromUrl;
  if (refSel && refs.some(r => r.repo === refSel.repo && r.ref === refSel.ref)) return refSel;
  const newest = list => [...list].sort((a, b) => b.t.localeCompare(a.t));
  const rel = newest(refs.filter(r => r.kind === 'release')), mains = newest(refs.filter(r => r.ref === 'main'));
  const d = rel[0] || mains.find(r => /Trail_Tracer_Mower$/.test(r.repo)) || mains[0] || newest(refs)[0];
  return d ? { repo: d.repo, ref: d.ref } : '*';
}
const inSel = (x, sel) => sel === '*' || stmOnly(x.m.source.repo) || (x.m.source.repo === sel.repo && x.m.source.branch === sel.ref);
const refHtml = r => `<b>${esc(r.title)}</b><span class="rdesc">${esc(r.description)}</span><span class="rgit">${esc(repoName(r.repo))} · ${esc(r.git)}</span>`;
function drawRefMenu() {
  const refs = refsOf(), sel = currentRef(), list = $('refList');
  $('refBox').hidden = !refs.length || sel === '*';
  let cur = sel !== '*' && refs.find(r => r.repo === sel.repo && r.ref === sel.ref);
  if (cur && sel.commit) {
    const pb = offerBuilds(sel).find(x => x.m.source.repo === sel.repo && x.m.source.branch === sel.ref);
    if (pb) cur = { ...cur, git: `${sel.ref}, ${pb.m.source.commit.slice(0, 7)} (this exact version)` };
  }
  if (cur) { const h = refHtml(cur); if ($('refBtn').dataset.h !== h) { $('refBtn').dataset.h = h; $('refBtn').innerHTML = h; } }
  if (cur) {
    const newest = builds.find(x => x.m.source.repo === cur.repo && x.m.source.branch === cur.ref);
    const pinC = sel.commit ? (offerBuilds(sel).find(x => x.m.source.repo === cur.repo && x.m.source.branch === cur.ref) || newest) : newest;
    const link = urlFor(cur, pinC && pinC.m.source.commit.slice(0, 12));
    if ($('refLink').href !== link) $('refLink').href = link;
    if (!window.PROGRAM_TEST) { const want = urlFor(cur, sel.commit); if (location.href !== want) history.replaceState(null, '', want); }
  }
  const key = refs.map(r => [r.repo, r.ref, r.commit, r.title, r.description].join(' ')).join('|') + (cur ? cur.repo + cur.ref : '');
  if (list.dataset.key === key) return;
  list.dataset.key = key;
  list.innerHTML = [...new Set(refs.map(r => r.repo))].map(repo => `<div class="rgroup">${esc(repoName(repo))}</div>`
    + refs.filter(r => r.repo === repo).map(r => `<button class="refopt${cur && cur.repo === r.repo && cur.ref === r.ref ? ' on' : ''}" data-v="${esc(JSON.stringify({ repo: r.repo, ref: r.ref }))}">${refHtml(r)}</button>`).join('')).join('');
  for (const el of list.querySelectorAll('.refopt')) el.onclick = () => chooseRef(el.dataset.v);
}
function chooseRef(v) {
  refSel = JSON.parse(v); $('refList').hidden = true;
  if (!window.PROGRAM_TEST) history.replaceState(null, '', urlFor(refSel));
  try { localStorage.setItem('tt.ref', v); } catch { /* no storage */ }
  render();
}

const named = (label, v, sep = ' ') => `${label.replace(/\s*\bv?\d+\.\d+\.\d+[^\s,()]*/g, '').trim()}${sep}${v}`;
function cycles(rom, stm) {
  if (rom && stm) return 'Then power-cycle the board once, and once more if the STM32 version differs.';
  if (rom) return 'Then power-cycle the board once.';
  return stm ? 'Then power-cycle the board once if the STM32 version differs.' : 'No action from you.';
}
const stmCurrent = (b, s) => !!(b && s && b.stm === `app ${s.m.version}`);
async function stmPart(r1, stm) {
  const nb = [...boards.values()].find(x => x.mode === 'app' && x.firmware === 'rover firmware' && x.state === 'open');
  if (!nb) throw plain(`${r1} The STM32 update did not start: the board is not on USB. Start the install again.`);
  if (stmCurrent(nb, stm)) {
    step('stm'); say(`STM32 runs ${nb.stm}, the version of the entry: no STM32 update`);
    step('check'); return `${r1} The STM32 runs ${nb.stm} already: no STM32 update.`;
  }
  try { return `${r1} ${await stmLoad(nb, stm)}`; }
  catch (ex) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
}
function offersFor(b) {
  const out = [], no = [], sel = currentRef(), rev = b.board || 'RX_REVA';
  const pool = offerBuilds(sel).filter(x => inSel(x, sel));
  const stmExtra = pool.filter(x => stmOnly(x.m.source.repo) && x.m.chip === 'stm32g070');
  const groups = new Map();
  for (const x of pool) {
    if (stmOnly(x.m.source.repo)) continue;
    const k = x.m.source.repo + ' ' + x.m.source.branch, g = groups.get(k) || {};
    if (x.m.chip === 'esp32s2' && !g.esp) g.esp = x;
    if (x.m.chip === 'stm32g070' && !g.stm) g.stm = x;
    groups.set(k, g);
  }
  for (const g of groups.values()) {
    if (!g.esp) continue;
    const m = g.esp.m, rover = !!m.esp32.usb_update, stm = rover ? (g.stm || stmExtra[0]) : null;
    const what = `${rover ? named(m.label || 'ESP32 rover firmware', m.version) : named(m.label || m.source.repo.split('/').pop() + ' firmware', m.version, ', ')}`
      + (stm ? ` + ${named(stm.m.label || 'STM32 app', stm.m.version)}` : '') + ` (${srcLine(m, g.esp.release)})`;
    const bad = [g.esp, stm].filter(Boolean).find(x => !x.m.boards.includes(rev));
    if (bad) { no.push({ l1: 'Full install', l2: what, reason: `This firmware is for board revision ${bad.m.boards.join(', ')}. This board is revision ${rev}.` }); continue; }
    if (b.mode !== 'rom') continue;
    const espName = named(m.label || 'firmware', m.version), stmName = stm && named(stm.m.label || 'STM32 app', stm.m.version);
    const steps = [{ id: 'esp', text: `ESP32: write ${espName} (only the parts that differ)` }, { id: 'start', text: 'Start the new ESP32 firmware' },
      ...(stm ? [{ id: 'stm', text: `STM32: update to ${stmName} if its version differs` }] : []), { id: 'check', text: 'Read both versions' }];
    out.push({ l1: 'Install', l2: what, steps,
      l3: `Installs what differs on the ESP32${stm ? ' and the STM32' : ''}. No action from you.`,
      plan: [['Erases', stm ? 'the ESP32 settings; the whole ESP32 when its partition table differs' : 'the whole ESP32 only when its partition table differs'],
        ['Writes ESP32', `${espName}: only the parts that differ`],
        ...(stm ? [['Writes STM32', `${stmName}, only if its version differs; its settings and calibration then return to their defaults`]] : []),
        ['Keeps', 'logs and files, when the partition table is the same']],
      go: () => fullInstall(b, g.esp, stm, steps) });
  }
  out.refused = no;
  if (b.mode !== 'rom') { out.button = true; return out; }
  const esteps = [{ id: 'esp', text: 'ESP32: erase the whole flash' }];
  out.erase = { l1: 'Erase ESP32', l2: 'Erases the whole ESP32 flash and writes no firmware.',
    l3: 'The board then stays in bootloader mode until a Full install.',
    plan: [['Erases', 'the whole ESP32 flash, 4 MB: firmware, settings, logs and Wi-Fi passwords. Nothing stays.'], ['Writes', 'nothing'], ['Keeps', 'the STM32 app and its update loader']],
    go: () => { if (job) job.steps = esteps.map(x => ({ ...x, state: 'wait' })); return romLoad(b, null, true); } };
  return out;
}
async function otaInstall(b, esp, stm, steps) {
  if (job && steps) job.steps = steps.map(x => ({ ...x, state: 'wait' }));
  const r1 = await otaUpdate(b, esp);
  if (!stm) { step('check'); return r1; }
  return stmPart(r1, stm);
}
async function fullInstall(b, esp, stm, steps) {
  if (job && steps) job.steps = steps.map(x => ({ ...x, state: 'wait' }));
  mergedBridge = null;
  const r1 = await romLoad(b, esp, true, stm);
  if (!stm) { step('check'); return r1; }
  if (mergedBridge) {
    const bb = mergedBridge; mergedBridge = null;
    try { return `${r1} ${await stmBridge(bb, stm)}`; }
    catch (ex) {
      if (ex.message !== NO_LOADER) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
      say('no HELLO answer after a watchdog start: the STM32 may run its app; the normal STM32 path follows');
      bb.binMode = false; bb.buf = ''; bb.bridge = false;
      if (!await waitLine(bb, /STM bridge off/, 30000)) say('no "STM bridge off" line within 30 s');
      Object.assign(bb, { firmware: null, version: null, role: null, stm: null, stmLink: null, table: null, tableCrc: null, hb: false });
      await detect(bb);
      return stmPart(r1, stm);
    }
  }
  if (false) {
    try { return ''; }
    catch (ex) { say(`STM32 part: ${ex.message}`); throw plain(`The ESP32 firmware is installed. ${ex.plain ? ex.message : 'The STM32 update stopped. Start the install again.'}`); }
  }
  return stmPart(r1, stm);
}

async function start() {
  $('refBtn').onclick = e => { e.stopPropagation(); $('refList').hidden = !$('refList').hidden; };
  addEventListener('click', e => { if (!$('refBox').contains(e.target)) $('refList').hidden = true; });
  try {
    const v = await (await fetch('version.json', { cache: 'no-store' })).json();
    if (v && v.version) pageVer = `Page v${v.version}, ${v.date}, ${v.commit}`;
  } catch { /* no version.json: a local copy */ }
  $('pageVer').textContent = pageVer;
  if (window.PROGRAM_TEST) return render();
  try {
    let r = await fetch('sources.json').catch(() => null);
    if (!r || !r.ok) r = await fetch('api/sources');
    for (const s of (await r.json()).sources) {
      const idx = await (await fetch(s.base + 'index.json')).json();
      const rr = await fetch(s.base + 'releases.json').catch(() => null), rel = rr && rr.ok ? (await rr.json()).commits || {} : null;
      const rf = await fetch(s.base + 'refs.json').catch(() => null);
      if (rf && rf.ok) for (const e of (await rf.json()).refs || []) refInfo[e.repo + ' ' + e.ref] = e;
      for (const e of idx.builds) {
        const m = await (await fetch(s.base + e.manifest)).json();
        builds.push({ m, base: s.base + e.manifest.replace(/[^/]*$/, ''), src: s.name, release: rel ? rel[m.source.commit] || false : undefined });
      }
    }
    builds.sort((a, b) => b.m.source.built_utc.localeCompare(a.m.source.built_utc));
    allBuilds = [...builds];
    const seen = new Set();
    builds = builds.filter(x => { const k = [x.m.source.repo, x.m.source.branch, x.m.chip].join(' '); return !seen.has(k) && seen.add(k); });
  } catch (ex) { console.error(ex); }
  if (!('serial' in navigator) || !isSecureContext) {
    $('noSerial').hidden = false;
    $('noSerial').textContent = !isSecureContext
      ? `This page needs a secure address for USB access. Open http://localhost${location.port ? ':' + location.port : ''}${location.pathname} on the PC with the Rx.`
      : 'This browser has no Web Serial. Use Chrome or Edge on a PC.';
    $('connectBtn').disabled = true;
    return render();
  }
  $('connectBtn').onclick = async () => {
    try { open(await navigator.serial.requestPort({ filters: [{ usbVendorId: ESPRESSIF }] })); }
    catch { /* the chooser was closed */ }
  };
  navigator.serial.addEventListener('connect', e => { const port = e.target; port.gone = false; lastConnect = Date.now(); setTimeout(() => { if (!port.gone) open(port); }, 2000); });
  navigator.serial.addEventListener('disconnect', e => {
    const port = e.target, b = boards.get(port);
    port.gone = true;
    disconnects++;
    if (b) { b.state = 'unplugged'; closeBoard(b).catch(() => {}); }
  });
  addEventListener('pagehide', () => { for (const b of boards.values()) { b.closing = true; b.port.close().catch(() => {}); } });
  for (const p of await navigator.serial.getPorts()) open(p);
  render();
}
start();
