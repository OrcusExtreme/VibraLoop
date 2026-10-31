"""Minimal GRBL 1.1 serial-port simulator for exercising cnc_app's streaming logic without hardware.

Behaviours modelled (the ones the sender logic depends on):
- 128-byte serial RX buffer (overflow is recorded as a test failure), 16-block planner
- 'ok' per parsed line, including empty lines ("?\\n" therefore yields an extra 'ok', like real GRBL)
- realtime bytes handled out-of-band: '?', '!', '~', 0x18, feed/spindle override bytes (0x90-0x9D)
  * override bytes set *flags* processed once per main-loop tick, so bursts collapse (as in GRBL)
- spindle state changes (M3/M4/M5) and G4 dwell synchronise with the planner before replying 'ok'
- soft reset while moving -> ALARM:3; G-code in alarm -> error:9; $X unlocks
- feed hold: Hold:1 while decelerating (decel_time), then Hold:0; reset after Hold:0 keeps position (no alarm);
  0x9E (spindle stop) stops the spindle only while holding
- unsupported commands -> error:20; motion modal state (G0/G1/G2/G3), G90/G91, G10 L20 work offsets
- status reports <State|MPos:..|FS:..|WCO:..|Ov:..> with Ov/WCO included periodically or after a change
"""
import math
import re
import threading
import time
from collections import deque

WORD_RE = re.compile(r'([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))')
SUPPORTED_G = {0, 1, 2, 3, 4, 10, 17, 18, 19, 20, 21, 28, 30, 53, 54, 55, 56, 57, 58, 59, 80, 90, 91, 92, 93, 94}
SUPPORTED_M = {0, 1, 2, 3, 4, 5, 7, 8, 9, 30}


class FakeGrbl:
    def __init__(self, port="SIM", time_scale=20.0, rapid_rate=5000.0, error_on=()):
        self.port = port
        self.time_scale = time_scale
        self.rapid_rate = rapid_rate
        self.error_on = tuple(error_on)
        self.timeout = 1.0
        self.is_open = True
        self._dtr = False

        self.lock = threading.RLock()
        self.rx = bytearray()           # host -> grbl serial buffer (non-realtime bytes)
        self.tx = bytearray()           # grbl -> host
        self.planner = deque()
        self.state = "Idle"
        self.hold = False
        self.hold_since = None
        self.decel_time = 0.15          # real seconds from '!' until Hold:0 while moving
        self.spindle_stops = 0          # 0x9E received while holding
        self.alarm = False
        self.mpos = [0.0, 0.0, 0.0]
        self.wco = [0.0, 0.0, 0.0]
        self.modal_motion = 0
        self.absolute = True
        self.feed = 0.0
        self.spindle = 0.0
        self.spindle_on = False
        self.spindle_action = None      # M3/M5 parsed but applied only when the planner reaches it (like GRBL)
        self.feed_ov = 100
        self.spindle_ov = 100
        self.ov_flags = set()
        self.report_ov_next = True
        self.report_count = 0
        self.current_block = None

        # test observability
        self.received_lines = []        # every complete line parsed (in order)
        self.executed_moves = []        # (line, start, end)
        self.overflows = 0
        self.realtime_log = []
        self.resets = 0

        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    # ----- pyserial-like interface -----
    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.tx)

    def write(self, data):
        with self.lock:
            for b in bytes(data):
                if b in (0x3F, 0x21, 0x7E, 0x18) or 0x80 <= b <= 0xFF:
                    self._realtime(b)
                else:
                    if len(self.rx) >= 128:
                        self.overflows += 1
                        continue
                    self.rx.append(b)
        return len(data)

    def flush(self):
        pass

    def read(self, n=1):
        deadline = time.time() + (self.timeout or 0)
        while True:
            with self.lock:
                if self.tx:
                    out = bytes(self.tx[:n])
                    del self.tx[:n]
                    return out
            if time.time() >= deadline:
                return b""
            time.sleep(0.001)

    def readline(self):
        deadline = time.time() + (self.timeout or 0)
        while True:
            with self.lock:
                idx = self.tx.find(b"\n")
                if idx >= 0:
                    out = bytes(self.tx[:idx + 1])
                    del self.tx[:idx + 1]
                    return out
            if time.time() >= deadline:
                with self.lock:
                    out = bytes(self.tx)
                    self.tx.clear()
                return out
            time.sleep(0.001)

    def reset_input_buffer(self):
        with self.lock:
            self.tx.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False
        self._stop = True

    # ----- GRBL internals -----
    def _send(self, text):
        self.tx.extend((text + "\r\n").encode())

    def _hold_complete(self):
        return self.hold and (self.current_block is None or time.time() - (self.hold_since or 0) >= self.decel_time)

    def _realtime(self, b):
        self.realtime_log.append(b)
        if b == 0x3F:  # '?'
            self._status_report()
        elif b == 0x21:  # '!'
            if self.state in ("Run", "Idle") and not self.alarm and not self.hold:
                self.hold = True
                self.hold_since = time.time()
        elif b == 0x7E:  # '~'
            self.hold = False
            self.hold_since = None
        elif b == 0x18:
            self._soft_reset()
        elif 0x90 <= b <= 0x9D:
            self.ov_flags.add(b)
        elif b == 0x9E:  # toggle spindle stop: only while holding
            if self.hold:
                self.spindle_on = False
                self.spindle_stops += 1

    def _soft_reset(self):
        self.resets += 1
        # GRBL raises ALARM:3 only when reset during motion; after a completed feed hold the position is kept
        moving = (self.current_block is not None or bool(self.planner)) and not self._hold_complete()
        self.rx.clear()
        self.planner.clear()
        self.current_block = None
        self.hold = False
        self.spindle_on = False
        self.feed_ov = self.spindle_ov = 100
        self.modal_motion = 0
        self.absolute = True
        self._send("")
        self._send("Grbl 1.1h ['$' for help]")
        if moving or self.alarm:
            self.alarm = True
            self._send("ALARM:3")
            self._send("[MSG:'$H'|'$X' to unlock]")

    def _status_report(self):
        if self.alarm:
            st = "Alarm"
        elif self.hold:
            st = "Hold:0" if self._hold_complete() else "Hold:1"
        elif self.current_block is not None or self.planner:
            st = "Run"
        else:
            st = "Idle"
        feed = 0 if self.current_block is None or self.hold else self.current_block['rate']
        rpm = self.spindle * self.spindle_ov / 100.0 if self.spindle_on else 0
        report = f"<{st}|MPos:{self.mpos[0]:.3f},{self.mpos[1]:.3f},{self.mpos[2]:.3f}|FS:{feed:.0f},{rpm:.0f}"
        self.report_count += 1
        if self.report_count % 10 == 0:
            report += f"|WCO:{self.wco[0]:.3f},{self.wco[1]:.3f},{self.wco[2]:.3f}"
        if self.report_ov_next or self.report_count % 15 == 0:
            report += f"|Ov:{self.feed_ov},100,{self.spindle_ov}"
            self.report_ov_next = False
        self._send(report + ">")

    def _apply_override_flags(self):
        flags, self.ov_flags = self.ov_flags, set()
        if not flags:
            return
        feed, spindle = self.feed_ov, self.spindle_ov
        if 0x90 in flags: feed = 100
        if 0x91 in flags: feed += 10
        if 0x92 in flags: feed -= 10
        if 0x93 in flags: feed += 1
        if 0x94 in flags: feed -= 1
        if 0x99 in flags: spindle = 100
        if 0x9A in flags: spindle += 10
        if 0x9B in flags: spindle -= 10
        if 0x9C in flags: spindle += 1
        if 0x9D in flags: spindle -= 1
        self.feed_ov = max(10, min(200, feed))
        self.spindle_ov = max(10, min(200, spindle))
        self.report_ov_next = True

    def _parse_line(self, line):
        """Returns (response_lines, planner_block_or_None, sync_required)."""
        text = re.sub(r'\([^)]*\)', '', line).split(';')[0].strip().upper()
        if text == "":
            return ["ok"], None, False
        if text.startswith("$"):
            if text == "$X":
                if self.alarm:
                    self.alarm = False
                    return ["[MSG:Caution: Unlocked]", "ok"], None, False
                return ["ok"], None, False
            if text == "$$":
                return ["$0=10", "$1=25", "$100=800.000", "$110=5000.000", "ok"], None, False
            return ["ok"], None, False
        if self.alarm:
            return ["error:9"], None, False
        if any(token in text for token in self.error_on):
            return ["error:20"], None, False
        words = [(l, float(v)) for l, v in WORD_RE.findall(text)]
        if not words:
            return ["error:1"], None, False
        axes, offsets = {}, {}
        g_codes, m_codes = [], []
        for l, v in words:
            if l == 'G':
                g_codes.append(v)
            elif l == 'M':
                m_codes.append(v)
            elif l in 'XYZ':
                axes[l] = v
            elif l in 'IJKR':
                offsets[l] = v
            elif l == 'F':
                self.feed = v
            elif l == 'S':
                self.spindle = v
            elif l in 'TNP L':
                pass
        for g in g_codes:
            if g != int(g) or int(g) not in SUPPORTED_G:
                return ["error:20"], None, False
        for m in m_codes:
            if m != int(m) or int(m) not in SUPPORTED_M:
                return ["error:20"], None, False
        sync = False
        g_int = [int(g) for g in g_codes]
        for g in g_int:
            if g in (0, 1, 2, 3):
                self.modal_motion = g
            elif g == 90:
                self.absolute = True
            elif g == 91:
                self.absolute = False
        if 10 in g_int:  # G10 L20 P1 X0 -> set work offset so current position equals given value
            for i, a in enumerate('XYZ'):
                if a in axes:
                    self.wco[i] = self.mpos_target_end()[i] - axes[a]
            return ["ok"], None, True
        if 4 in g_int:
            return ["ok"], None, True
        for m in m_codes:
            if int(m) in (3, 4, 5):
                self.spindle_action = int(m) != 5  # applied in _run once earlier motion has finished
                sync = True
        block = None
        if axes and not (set(g_int) & {10, 28, 30, 53, 92, 4}):
            start = self.mpos_target_end()
            end = list(start)
            for i, a in enumerate('XYZ'):
                if a in axes:
                    end[i] = (axes[a] + self.wco[i]) if self.absolute else end[i] + axes[a]
            if self.modal_motion != 0 and self.feed <= 0:
                return ["error:22"], None, False
            block = {'line': line, 'start': start, 'end': end, 'motion': self.modal_motion,
                     'rate': self.rapid_rate if self.modal_motion == 0 else self.feed, 'progress': 0.0,
                     'offsets': dict(offsets)}
            block['length'] = self._block_length(block)
        return ["ok"], block, sync

    def mpos_target_end(self):
        if self.planner:
            return list(self.planner[-1]['end'])
        if self.current_block is not None:
            return list(self.current_block['end'])
        return list(self.mpos)

    def _block_length(self, block):
        s, e = block['start'], block['end']
        if block['motion'] in (2, 3):
            if 'R' in block['offsets']:
                r = abs(block['offsets']['R'])
                chord = math.hypot(e[0] - s[0], e[1] - s[1])
                sweep = 2 * math.asin(min(1.0, chord / (2 * r))) if r else 0
                return max(r * sweep, math.dist(s, e))
            cx, cy = s[0] + block['offsets'].get('I', 0), s[1] + block['offsets'].get('J', 0)
            r = math.hypot(s[0] - cx, s[1] - cy)
            a0, a1 = math.atan2(s[1] - cy, s[0] - cx), math.atan2(e[1] - cy, e[0] - cx)
            sweep = (a0 - a1) if block['motion'] == 2 else (a1 - a0)
            if sweep <= 1e-9:
                sweep += 2 * math.pi
            block['arc'] = (cx, cy, r, a0, sweep, 1 if block['motion'] == 3 else -1)
            return math.hypot(r * sweep, e[2] - s[2])
        return math.dist(s, e)

    def _position_at(self, block, frac):
        s, e = block['start'], block['end']
        if 'arc' in block:
            cx, cy, r, a0, sweep, direction = block['arc']
            ang = a0 + direction * sweep * frac
            return [cx + r * math.cos(ang), cy + r * math.sin(ang), s[2] + (e[2] - s[2]) * frac]
        return [s[i] + (e[i] - s[i]) * frac for i in range(3)]

    def _run(self):
        pending_sync = None   # response lines waiting for planner to empty
        pending_spindle = None
        last = time.time()
        while not self._stop:
            time.sleep(0.002)
            now = time.time()
            dt = (now - last) * self.time_scale
            last = now
            with self.lock:
                self._apply_override_flags()
                # parse lines from RX buffer while planner has room
                while pending_sync is None and b"\n" in self.rx and len(self.planner) < 15:
                    idx = self.rx.find(b"\n")
                    line = self.rx[:idx].decode(errors="ignore").strip("\r")
                    del self.rx[:idx + 1]
                    self.received_lines.append(line)
                    responses, block, sync = self._parse_line(line)
                    action, self.spindle_action = self.spindle_action, None
                    if block is not None:
                        self.planner.append(block)
                    if sync and (self.planner or self.current_block is not None):
                        pending_sync = responses
                        pending_spindle = action
                        break
                    if action is not None:
                        self.spindle_on = action
                    for r in responses:
                        self._send(r)
                if pending_sync is not None and not self.planner and self.current_block is None:
                    if pending_spindle is not None:
                        self.spindle_on = pending_spindle
                        pending_spindle = None
                    for r in pending_sync:
                        self._send(r)
                    pending_sync = None
                # execute motion
                if self.alarm or self.hold:
                    self.state = "Alarm" if self.alarm else "Hold"
                    continue
                if self.current_block is None and self.planner:
                    self.current_block = self.planner.popleft()
                blk = self.current_block
                if blk is None:
                    self.state = "Idle"
                    continue
                self.state = "Run"
                rate = blk['rate'] * (self.feed_ov / 100.0 if blk['motion'] != 0 else 1.0)
                if blk['length'] <= 1e-9:
                    blk['progress'] = 1.0
                else:
                    blk['progress'] = min(1.0, blk['progress'] + (rate / 60.0) * dt / blk['length'])
                self.mpos = self._position_at(blk, blk['progress'])
                if blk['progress'] >= 1.0:
                    self.mpos = list(blk['end'])
                    self.executed_moves.append((blk['line'], blk['start'], blk['end']))
                    self.current_block = None
