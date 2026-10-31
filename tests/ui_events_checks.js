// Executes static/script.js socket handlers with representative server payloads against a DOM stub
// and reports runtime exceptions plus the resulting UI state.
// Usage: node tests/ui_events_checks.js static/script.js   (or: python tests/run_tests.py ui)
const fs = require('fs');
const vm = require('vm');
const src = fs.readFileSync(process.argv[2], 'utf8');

const elements = {};
function makeElement(id) {
    const el = {
        id, value: '', textContent: '', innerHTML: '', innerText: '', style: {}, disabled: false, checked: true,
        files: [], dataset: {}, children: [], scrollTop: 0, scrollHeight: 0, title: '', listeners: {},
        classList: { set: new Set(), add(c) { this.set.add(c); }, remove(c) { this.set.delete(c); }, contains(c) { return this.set.has(c); },
            toggle(c, force) { const on = force === undefined ? !this.set.has(c) : !!force; on ? this.set.add(c) : this.set.delete(c); return on; } },
        addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
        appendChild(c) { this.children.push(c); return c; },
        removeChild(c) { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); },
        getContext() { return {}; }, querySelector() { return null; }, closest() { return null; }, focus() {}, click() {},
        get firstChild() { return this.children[0]; },
    };
    if (id === 'tolerance') el.value = '0';
    if (id === 'feed-override') el.value = '100';
    return el;
}
const document = {
    getElementById(id) { return elements[id] || (elements[id] = makeElement(id)); },
    querySelectorAll() { return []; }, querySelector(sel) { return sel === '.alarm-section' ? document.getElementById('alarm-section') : null; },
    createElement(tag) { return makeElement('<' + tag + '>'); },
    listeners: {}, activeElement: null,
    addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
    body: { appendChild() {}, removeChild() {} },
};
const sockets = [];
function makeSocket() {
    const s = { connected: true, handlers: {}, emits: [],
        on(ev, fn) { (this.handlers[ev] = this.handlers[ev] || []).push(fn); },
        off() {}, disconnect() {}, connect() {}, emit(ev, data, cb) { this.emits.push(ev); } };
    sockets.push(s);
    return s;
}
class FakeChart {
    constructor() { this.data = { datasets: [{ data: [] }, { data: [] }], labels: [] }; this.options = { scales: { x: {}, y: { grid: {} } } }; this.updates = []; }
    update(mode) { this.updates.push(mode); }
}
FakeChart.defaults = {};
const timeouts = [];
const errors = [];
const sandbox = {
    console: { log() {}, warn() {}, error(...a) { errors.push('console.error: ' + a.join(' ')); } },
    document, window: { devicePixelRatio: 1, location: { origin: 'http://localhost:5000' }, addEventListener() {} },
    io: () => makeSocket(), Chart: FakeChart,
    localStorage: { getItem() { return null; }, setItem() {}, removeItem() {} },
    fetch: () => new Promise(() => {}), alert() {},
    setTimeout: (fn) => { timeouts.push(fn); return timeouts.length; }, clearTimeout() {},
    setInterval: () => 1, clearInterval() {},
    Blob: function () {}, URL: { createObjectURL() { return ''; }, revokeObjectURL() {} }, FileReader: function () {},
    Date, Math, JSON, Number, parseFloat, parseInt, isNaN, Array, Object, String, Promise, encodeURIComponent, Infinity,
};
vm.createContext(sandbox);
vm.runInContext(src, sandbox, { filename: 'script.js' });
for (const fn of document.listeners['DOMContentLoaded'] || []) fn();
while (timeouts.length) { try { timeouts.shift()(); } catch (e) { errors.push('timer: ' + e.message); } }
const socket = sockets[sockets.length - 1];

function fire(ev, data) {
    for (const fn of socket.handlers[ev] || []) {
        try { fn(data); } catch (e) { errors.push(`${ev}: ${e.name}: ${e.message}`); }
    }
    return (socket.handlers[ev] || []).length;
}
const g = (expr) => vm.runInContext(expr, sandbox);
const results = [];
const check = (name, cond, detail = '') => { results.push(!!cond); console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };

const base = { x: 0, y: 0, z: 0, spindle: 16000, feedrate: 50, progress: 0, elapsed_time: 1, state: 'running' };
fire('position_update', { ...base, path_start: 0, path_x: [0, 1, 2], path_y: [0, 0, 0], path_z: [0, -0.2, -0.2] });
check('full path replaces local path', g('xVals.length') === 3 && g('zVals[1]') === -0.2, `len=${g('xVals.length')}`);
fire('position_update', { ...base, x: 4, path_start: 3, path_x: [3, 4], path_y: [0, 0], path_z: [-0.2, -0.2] });
check('incremental points appended', g('xVals.join(",")') === '0,1,2,3,4' && g('timeVals.length') === 5);
fire('position_update', { ...base, x: 4, path_start: 5, path_x: [], path_y: [], path_z: [] });
check('empty increment keeps path', g('xVals.length') === 5);
const emitsBefore = socket.emits.length;
fire('position_update', { ...base, path_start: 9, path_x: [9], path_y: [0], path_z: [0] });
check('gap in path -> request_status resync', socket.emits.slice(emitsBefore).includes('request_status') && g('xVals.length') === 5);
fire('position_update', { ...base, path_start: 2, path_x: [7, 8], path_y: [1, 1], path_z: [0, 0] });
check('overlapping update truncates then appends', g('xVals.join(",")') === '0,1,7,8');
check('coordinate label updated', document.getElementById('coordinate-label').textContent.includes('X: 0.00'));
const big = Array.from({ length: 200000 }, (_, i) => i * 0.01);
fire('position_update', { ...base, path_start: 0, path_x: big, path_y: big, path_z: big });
check('200k-point path handled (no Math.min(...arr) stack overflow)', g('xVals.length') === 200000 && !errors.some(e => e.includes('stack')));

fire('alarm_update', { text: '알람 상태: ALARM:3', color: 'red' });
check('alarm_update shows server text (was overwritten with "알람 없음")', document.getElementById('alarm-label').textContent === '알람 상태: ALARM:3' && document.getElementById('alarm-label').style.color === 'red');
check('alarm_update does not touch processing state', g('currentProcessingState') === 'idle', g('currentProcessingState'));
fire('state_update', { state: 'running' });
check('state_update sets processing state', g('currentProcessingState') === 'running');
fire('tolerance_update', { tolerance: 0.2 });
check('tolerance_update syncs input', document.getElementById('tolerance').value === 0.2 && g('toleranceValue') === 0.2);
fire('override_status', { spindle: 90, feed: 70 });
check('override_status updates labels/slider', document.getElementById('spindle-override-value').textContent === '90%' &&
      document.getElementById('feed-override-value').textContent === '70%' && document.getElementById('feed-override').value === 70);
fire('spindle_rpm_update', { rpm: 14400, override: 90 });
check('spindle_rpm_update shows override and RPM', document.getElementById('spindle-override-value').innerText === '90% (14400 RPM)');
for (let i = 0; i < 700; i++) fire('arduino_data', { data: JSON.stringify({ accel: { x: 0.06, y: 0.98, z: -0.04 }, gyro: { x: 1, y: 2, z: 3 } }) });
check('sensor samples stored for CSV export', g('sensorDataArray.length') === 700);
check('sensor log capped', document.getElementById('sensor-log-text').children.length <= 500, `${document.getElementById('sensor-log-text').children.length} lines`);
check('vibration_alert handled by exactly one handler', fire('vibration_alert', { show: true }) === 1 && document.getElementById('vibration-alert').classList.contains('show'));
fire('state_update', { state: 'paused' });
fire('log_update', { message: '[센서] 테스트', color: 'sensor' });
fire('connection_status', { connected: true, port: 'COM3' });
check('connection_status sets controller flag', g('controllerConnected') === true);
fire('connection_status', { connected: false });
check('disconnect clears controller flag', g('controllerConnected') === false);
check('no runtime errors in handlers', errors.length === 0, errors.join(' | '));
console.log(`\njs handler checks: ${results.filter(Boolean).length}/${results.length} passed`);
process.exit(results.every(Boolean) ? 0 : 1);
