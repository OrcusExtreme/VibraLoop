// Exercises the vibration-model modal code in static/script.js against a DOM stub with a scripted fetch.
// Usage: node tests/ui_vibration_checks.js static/script.js   (or: python tests/run_tests.py ui)
const fs = require('fs');
const vm = require('vm');
const src = fs.readFileSync(process.argv[2], 'utf8');

const elements = {};
function makeElement(id) {
    return {
        id, value: '', textContent: '', innerHTML: '', innerText: '', style: {}, disabled: false, checked: true,
        files: [], dataset: {}, children: [], scrollTop: 0, scrollHeight: 0, title: '', listeners: {},
        classList: {
            set: new Set(), add(c) { this.set.add(c); }, remove(c) { this.set.delete(c); }, contains(c) { return this.set.has(c); },
            toggle(c, force) { const on = force === undefined ? !this.set.has(c) : !!force; on ? this.set.add(c) : this.set.delete(c); return on; },
        },
        addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
        appendChild(c) { this.children.push(c); return c; },
        removeChild(c) { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); },
        getContext() { return {}; }, querySelector() { return null; }, closest() { return null; }, focus() {}, click() {},
        get firstChild() { return this.children[0]; },
    };
}
const notifications = [];
const document = {
    getElementById(id) { return elements[id] || (elements[id] = makeElement(id)); },
    querySelectorAll() { return []; }, querySelector() { return null; },
    createElement(tag) { return makeElement('<' + tag + '>'); },
    listeners: {}, addEventListener(ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
    body: { appendChild(n) { notifications.push(n.textContent + ' [' + n.className + ']'); }, removeChild() {} },
};
const fetchCalls = [];
let responder = () => new Promise(() => {});
const confirms = [];
let confirmAnswer = true;
const timeouts = [];
const errors = [];
const socketHandlers = {};
class FakeFormData { constructor() { this.entries = []; } append(k, v) { this.entries.push([k, v]); } }
const sandbox = {
    console: { log() {}, warn() {}, error(...a) { errors.push('console.error: ' + a.join(' ')); } },
    document, window: { devicePixelRatio: 1, location: { origin: 'http://localhost:5000' }, addEventListener() {} },
    io: () => ({ connected: true, on(ev, fn) { socketHandlers[ev] = fn; }, off() {}, emit() {}, disconnect() {}, connect() {} }),
    Chart: Object.assign(class { constructor() { this.data = { datasets: [{ data: [] }, { data: [] }], labels: [] }; this.options = { scales: { x: {}, y: { grid: {} } } }; } update() {} }, { defaults: {} }),
    localStorage: { getItem() { return null; }, setItem() {}, removeItem() {} },
    fetch: (url, opts) => { fetchCalls.push({ url, opts }); return responder(url, opts); },
    alert() {}, confirm: (msg) => { confirms.push(msg); return confirmAnswer; }, FormData: FakeFormData,
    setTimeout: (fn, ms) => { timeouts.push({ fn, ms }); return timeouts.length; }, clearTimeout() {},
    setInterval: () => 1, clearInterval() {},
    Blob: function () {}, URL: { createObjectURL() { return ''; }, revokeObjectURL() {} }, FileReader: function () {},
    Date, Math, JSON, Number, parseFloat, parseInt, isNaN, Array, Object, String, Promise, encodeURIComponent, Infinity,
};
vm.createContext(sandbox);
vm.runInContext(src, sandbox, { filename: 'script.js' });

const results = [];
const check = (name, cond, detail = '') => { results.push(!!cond); console.log(`  [${cond ? 'PASS' : 'FAIL'}] ${name}${detail ? ' — ' + detail : ''}`); };
const flush = () => new Promise(r => setImmediate(r));
const json = (body) => Promise.resolve({ ok: true, json: () => Promise.resolve(body) });
const $ = (id) => document.getElementById(id);
const trained = { is_trained: true, is_training: false, training_dir: 'data/학습 데이터', files: ['a.csv', 'b.csv'], samples: 123456,
    windows: 4567, thresholds: { rms_g: 0.65, spe: 1.23, t2: 31.5, calibration: 'session_holdout',
        features: ['rms_x', 'rms_y', 'rms_z', 'kurt_x', 'kurt_y', 'kurt_z', 'crest_x', 'crest_y', 'crest_z'], components: 7, explained: 0.993 },
    settings: { mitigation_enabled: true, decision_mode: 'k_of_n', shadow_two_tier: false, t2_alarm: false }, optimizer: { sld: false, methods: ['chatter_frequency', 'fallback'] }, condition_model: false,
    sensor: { device: 'MPU6050', fs: 1000, range_g: 8 },
    window: { profile: 'legacy', label: '1초 창·0.25초 간격', stream_profile: 'fast', fallback: true, window_sec: 1.0, hop_sec: 0.25 } };

(async () => {
    // startup: server still training -> 'loading' then retry -> trained
    let calls = 0;
    responder = (url) => url !== '/api/vibration/model' ? new Promise(() => {}) :
        json(++calls === 1 ? { status: 'loading', message: '진동 감지기를 준비하는 중입니다.' } : { status: 'success', model: trained });
    for (const fn of document.listeners['DOMContentLoaded'] || []) fn();
    await flush();
    check('startup asks the server for the model status', fetchCalls.some(c => c.url === '/api/vibration/model'));
    check('loading message shown in the control frame', $('vibration-status-text').textContent.includes('준비하는 중'));
    const retry = timeouts.find(t => t.ms === 2000);
    check('retries while the server is still training', !!retry);
    // run initEventListeners (500ms timer) and the retry
    for (const t of timeouts.splice(0)) { try { t.fn(); } catch (e) { errors.push('timer: ' + e.message); } }
    await flush();
    check('trained summary shown after retry', $('vibration-status-text').textContent === '진동 감지 모델: 학습됨 (정상 데이터 2개 파일)', $('vibration-status-text').textContent);
    check('detail lists counts, thresholds, condition source and sensor', $('vibration-model-status').textContent.includes('샘플 123,456개') &&
          $('vibration-model-status').textContent.includes('0.650 g') &&
          $('vibration-model-status').textContent.includes('안정 한계 모델 없음 → 채터 주파수 또는 기본 규칙') &&
          $('vibration-model-status').textContent.includes('1000 Hz'), $('vibration-model-status').textContent);
    check('PCA detector shown: SPE / T² thresholds, features -> components', $('vibration-model-status').textContent.includes('SPE 1.23, T² 31.5') &&
          $('vibration-model-status').textContent.includes('특징 9개 → 주성분 7개, 분산 99%') && !$('vibration-model-status').textContent.includes('LOF'),
          $('vibration-model-status').textContent);
    check('window setting in use shown, with a note when the sensor has no model of its own window setting',
          $('vibration-model-status').textContent.includes('판정 창: 1초 창·0.25초 간격') &&
          $('vibration-model-status').textContent.includes('학습 데이터가 없어'), $('vibration-model-status').textContent);
    check('settings controls reflect the server', $('mitigation-enabled').checked === true && $('decision-mode').value === 'k_of_n'
          && $('shadow-two-tier').checked === false && $('t2-alarm').checked === false);
    check('file list rendered as text', $('vibration-model-files').textContent === 'a.csv\nb.csv');

    // open button
    const openHandlers = $('vibration-settings-btn').listeners.click || [];
    check('open button registered once', openHandlers.length === 1);
    openHandlers.forEach(fn => fn());
    await flush();
    check('modal opened', $('vibration-settings-modal').style.display === 'block');

    // train without file
    const trainHandlers = $('train-model-btn').listeners.click || [];
    const before = fetchCalls.length;
    trainHandlers.forEach(fn => fn());
    check('no file -> warning, no request', fetchCalls.length === before && notifications.some(n => n.includes('파일을 선택')));

    // train with a file, user cancels
    $('train-model-file').files = [{ name: '정상.csv' }];
    confirmAnswer = false;
    trainHandlers.forEach(fn => fn());
    check('cancelled confirm -> no request', fetchCalls.length === before && confirms.length === 1);

    // train with a file, server error keeps the selection
    confirmAnswer = true;
    let resolveTrain;
    responder = (url) => url === '/api/vibration/train' ? new Promise(r => { resolveTrain = r; }) : json({ status: 'success', model: trained });
    $('train-model-file').value = 'C:\\fakepath\\정상.csv';
    trainHandlers.forEach(fn => fn());
    const req = fetchCalls[fetchCalls.length - 1];
    check('POST /api/vibration/train with FormData(train_file)', req.url === '/api/vibration/train' && req.opts.method === 'POST' && req.opts.body.entries[0][0] === 'train_file');
    check('buttons disabled while training', $('train-model-btn').disabled && $('reset-vibration-analyzer').disabled);
    check('busy message shown', $('vibration-model-status').textContent.includes('학습하는 중'));
    resolveTrain({ ok: true, json: () => Promise.resolve({ status: 'error', message: '가공 중에는 진동 감지 모델을 바꿀 수 없습니다.' }) });
    await flush(); await flush();
    check('error -> notification, buttons re-enabled, file kept', notifications.some(n => n.includes('가공 중') && n.includes('error')) &&
          !$('train-model-btn').disabled && $('train-model-file').value !== '');
    check('error without model -> status refreshed from server', fetchCalls[fetchCalls.length - 1].url === '/api/vibration/model');

    // success clears the selection and renders the returned model (this time with a stability-limit model, P11)
    const updated = { ...trained, files: ['a.csv', 'b.csv', '정상.csv'],
        optimizer: { sld: true, methods: ['sld', 'chatter_frequency', 'fallback'], runs: 18, speed_range: [16000, 24000], flutes: 2,
            tool_id: 'T1', kappa: 1.645 } };
    responder = (url) => json({ status: 'success', message: '정상.csv을(를) 추가했습니다.', model: updated });
    trainHandlers.forEach(fn => fn());
    await flush(); await flush();
    check('success -> file input cleared, 3 files shown', $('train-model-file').value === '' && $('vibration-status-text').textContent.includes('3개 파일'));
    check('stability-limit model shown with its ramp count, tool and speed range',
          $('vibration-model-status').textContent.includes('안정 한계 모델 (ramp 18개, T1 2날, 16000~24000 rpm) → 채터 주파수 → 기본 규칙'),
          $('vibration-model-status').textContent);

    // P12: dataset detector model per window setting, decision thresholds, share outside the training range, condition model, notes
    const p12 = { ...trained, files: ['a.csv'],
        profiles: { legacy: { label: '1초 창·0.25초 간격', source: 'folder', sessions: 1, windows: 300, fs: 90, features: 9, nonharmonic: false },
            fast: { label: '0.2초 창·0.05초 간격', source: 'dataset', sessions: 24, windows: 9000, fs: 1000, features: 11, nonharmonic: true } },
        detector_model: { path: 'models/detector_model.joblib', dataset: 'data/dataset', trained_at: '2026-11-03T10:00:00', profiles: ['fast'] },
        settings: { mitigation_enabled: true, decision_mode: 'k_of_n', shadow_two_tier: true, t2_alarm: false, confirm_sec: 0.75,
            release_sec: 0.75, release_level: 0.8, cusum_threshold: 0.3, fast_ratio: 3.0, fast_sec: 0.15, mitigation_wait_sec: 4.0, rework_min_passes: 2 },
        window: { profile: 'fast', label: '0.2초 창·0.05초 간격', stream_profile: 'fast', fallback: false, window_sec: 0.2, hop_sec: 0.05,
            k: 16, n: 16, fast_blocks: 3, shadow: true },
        t2_recent: { fraction: 0.25, windows: 1200, seconds: 60 },
        condition_model: { runs: 108, coverage: 0.988, margin: 1.64, sensor_fs: 1000, trained_at: '2026-11-02T20:00:00' },
        notes: ['조건 모델은 1000 Hz 센서로 학습되어 지금 센서(1600 Hz)에서는 쓰지 않습니다.'] };
    sandbox.renderVibrationModelStatus(p12);
    const text12 = $('vibration-model-status').textContent;
    check('model window (P12): detector type and source per window setting, decision thresholds, T² share with advice, condition model, notes',
          $('vibration-status-text').textContent === '진동 감지 모델: 학습됨 (정상 데이터 1개 파일 + 데이터셋 감지 모델)' &&
          text12.includes('감지 모델 0.2초 창·0.05초 간격: PCA-SPE, 데이터셋 data/dataset (실행 24개, 2026-11-03 10:00:00 학습), 특징 11개, 센서 약 1000 Hz') &&
          text12.includes('감지 모델 1초 창·0.25초 간격: PCA-SPE, 학습 데이터 폴더 (파일 1개), 특징 9개') &&
          text12.includes('판정 기준: 연속 초과 0.75초 (윈도우 16/16개), 해제는 지표 0.8 미만 0.75초') &&
          text12.includes('학습 범위 밖(T²): 최근 판정 60초(윈도우 1200개) 중 25% - 학습하지 않은 조건') &&
          text12.includes('조건 모델: 실행 108개, 상한 = 예측 × 1.64, 상한 포함률 98.8%, 센서 1000 Hz') &&
          text12.includes('⚠️ 조건 모델은 1000 Hz 센서로 학습되어'), text12);
    sandbox.renderVibrationModelStatus({ ...p12, t2_recent: null, notes: [],
        profiles: { fast: { ...p12.profiles.fast, nonharmonic: false } } });
    const text12b = $('vibration-model-status').textContent;
    check('no judged windows yet -> the T² line says so; a fast model without the non-harmonic ratio is marked',
          text12b.includes('학습 범위 밖(T²): 아직 판정한 윈도우 없음') && text12b.includes('(비고조파 비율 없음)') && !text12b.includes('⚠️'), text12b);

    // reset
    responder = (url) => json({ status: 'success', message: '초기화했습니다.', model: { ...trained, is_trained: false, files: [] } });
    ($('reset-vibration-analyzer').listeners.click || []).forEach(fn => fn());
    await flush(); await flush();
    const resetReq = fetchCalls[fetchCalls.length - 1];
    check('reset posts to /api/vibration/reset after confirm', resetReq.url === '/api/vibration/reset' && resetReq.opts.method === 'POST');
    check('untrained model highlighted', $('vibration-status-text').classList.contains('untrained') && $('vibration-status-text').textContent.includes('학습 안 됨'));
    check('untrained detail names the folder', $('vibration-model-status').textContent.includes('data/학습 데이터 폴더'));

    // network failure
    responder = () => Promise.reject(new Error('offline'));
    ($('reset-vibration-analyzer').listeners.click || []).forEach(fn => fn());
    await flush(); await flush(); await flush();
    check('network failure -> error notification and buttons re-enabled', notifications.some(n => n.includes('오류가 발생')) && !$('reset-vibration-analyzer').disabled);

    // close button
    ($('cancel-vibration-settings').listeners.click || []).forEach(fn => fn());
    check('close button hides the modal', $('vibration-settings-modal').style.display === 'none');

    // detector settings
    responder = (url) => json({ status: 'success', message: '설정을 저장했습니다.', model: { ...trained, settings: { mitigation_enabled: false, decision_mode: 'k_of_n' } } });
    $('mitigation-enabled').checked = false;
    ($('mitigation-enabled').listeners.change || []).forEach(fn => fn.call($('mitigation-enabled')));
    await flush(); await flush();
    const settingsReq = fetchCalls[fetchCalls.length - 1];
    check('mitigation toggle posts JSON to /api/vibration/settings', settingsReq.url === '/api/vibration/settings' &&
          JSON.parse(settingsReq.opts.body).mitigation_enabled === false);
    $('shadow-two-tier').checked = true;
    ($('shadow-two-tier').listeners.change || []).forEach(fn => fn.call($('shadow-two-tier')));
    await flush(); await flush();
    const shadowReq = fetchCalls[fetchCalls.length - 1];
    check('record-only two-tier toggle posts shadow_two_tier', shadowReq.url === '/api/vibration/settings' &&
          JSON.parse(shadowReq.opts.body).shadow_two_tier === true);
    $('t2-alarm').checked = true;
    ($('t2-alarm').listeners.change || []).forEach(fn => fn.call($('t2-alarm')));
    await flush(); await flush();
    const t2Req = fetchCalls[fetchCalls.length - 1];
    check('T² alarm toggle posts t2_alarm', t2Req.url === '/api/vibration/settings' && JSON.parse(t2Req.opts.body).t2_alarm === true);

    // live indicator from the server
    socketHandlers['vibration_status']({ evidence: 0.42, gate: true, alarm: false, stage: null, parts: { level: 0.42 } });
    check('live evidence shown', $('vibration-live-text').textContent === '· 지표 0.42' && !$('vibration-live-text').classList.contains('alarm'));
    socketHandlers['vibration_status']({ evidence: 0.7, gate: true, alarm: false, stage: null, parts: { spe: 0.7, level: 0.5 }, t2: 1.4,
        contributions: [['kurt_z', 0.46], ['rms_z', 0.21]] });
    check('outside the training range (T² >= 1) flagged; hover lists T² and the largest SPE contributions',
          $('vibration-live-text').textContent === '· 지표 0.70 · 학습 범위 밖 조건' && $('vibration-live-text').title.includes('T² 1.4 (학습 범위 밖)') &&
          $('vibration-live-text').title.includes('SPE 기여 kurt_z 46% rms_z 21%'), $('vibration-live-text').title);
    socketHandlers['vibration_status']({ evidence: 1.8, gate: true, alarm: true, stage: 'mitigating', parts: {} });
    check('mitigation stage highlighted', $('vibration-live-text').textContent.includes('온라인 대응') && $('vibration-live-text').classList.contains('warn'));
    socketHandlers['vibration_status']({ evidence: null, gate: false, alarm: false, stage: null, parts: {} });
    check('outside the material: gate message', $('vibration-live-text').textContent.includes('판정 구간 밖'));
    socketHandlers['state_update']({ state: 'idle' });
    check('indicator cleared when the job ends', $('vibration-live-text').textContent === '');

    // data collection window
    const idleRec = { active: false, last_run: null, program_meta: { run_id: 'grid_001', S: '20000', F: '100', ap: '0.4', flutes: '2', type: 'grid' },
        tool: {}, sensor: null, sensor_connected: false };
    responder = (url) => json({ status: 'success', recording: idleRec });
    ($('recording-btn').listeners.click || []).forEach(fn => fn());
    await flush(); await flush();
    check('recording window opens and fills empty fields from META', $('recording-modal').style.display === 'block' &&
          $('rec-run_id').value === 'grid_001' && $('rec-S_rpm').value === '20000' && $('rec-ap_mm').value === '0.4' && $('rec-flutes').value === '2');
    check('sensor not connected is shown', $('recording-status').textContent.includes('연결되지 않음'));
    const activeRec = { active: true, run_id: 'grid_001', elapsed: 3.2, sensor_samples: 3200, sensor_dropped: 0, machine_rows: 16, labels: 0,
        open_labels: ['chatter'], program_meta: {}, tool: {}, sensor: { device: 'MPU6050', fs: 1000, range_g: 8 }, sensor_connected: true };
    responder = (url) => json({ status: 'success', message: 'grid_001 기록을 시작했습니다.', recording: activeRec });
    ($('rec-start-btn').listeners.click || []).forEach(fn => fn());
    await flush(); await flush();
    const startReq = fetchCalls[fetchCalls.length - 1];
    check('start posts the form values', startReq.url === '/api/recording/start' && JSON.parse(startReq.opts.body).run_id === 'grid_001' &&
          JSON.parse(startReq.opts.body).S_rpm === '20000');
    check('recording state shown (status, button, enabled controls)', $('recording-status').textContent.startsWith('● 기록 중: grid_001') &&
          $('recording-status').textContent.includes('열린 구간: 채터') && $('recording-btn').textContent === '기록 중' &&
          $('recording-btn').classList.contains('active') && $('rec-start-btn').disabled && !$('rec-stop-btn').disabled);
    $('rec-label-select').value = 'transition';
    ($('rec-mark-start-btn').listeners.click || []).forEach(fn => fn());
    await flush(); await flush();
    const markReq = fetchCalls[fetchCalls.length - 1];
    check('mark posts the selected label', markReq.url === '/api/recording/mark' && JSON.parse(markReq.opts.body).label === 'transition' &&
          JSON.parse(markReq.opts.body).edge === 'start');
    responder = (url) => json({ status: 'success', message: '기록 완료', recording: { ...idleRec, program_meta: {}, last_run: { run_id: 'grid_001', duration_s: '12.0', sensor_samples: 12000, labels: 1 } } });
    ($('rec-stop-btn').listeners.click || []).forEach(fn => fn());
    await flush(); await flush(); await flush();
    check('stop clears per-run fields and shows the last run', $('rec-run_id').value === '' && $('rec-S_rpm').value === '20000' &&
          $('recording-status').textContent.includes('마지막 기록: grid_001') && !$('recording-btn').classList.contains('active'));

    // sensor baud selection
    $('sensor-port-select').value = 'COM5';
    $('sensor-baud-select').value = '500000';
    const sockEmits = [];
    sandbox.__capture = (ev, data) => sockEmits.push([ev, data]);
    vm.runInContext('socket.emit = (ev, data) => __capture(ev, data); connectSensorPort();', sandbox);
    check('sensor connect sends the selected baud rate', sockEmits.length === 1 && sockEmits[0][0] === 'connect_sensor' && sockEmits[0][1].baud === 500000);

    const unexpected = errors.filter(e => !e.includes('진동 감지 모델'));
    check('no unexpected runtime errors', unexpected.length === 0, unexpected.join(' | '));
    console.log(`\njs vibration-modal checks: ${results.filter(Boolean).length}/${results.length} passed`);
    process.exit(results.every(Boolean) ? 0 : 1);
})();
