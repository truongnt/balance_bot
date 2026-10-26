#!/usr/bin/env python3
"""
Giao diện điều khiển robot trong trình duyệt.

    python robot_ui.py                        # tự chọn thiết bị đã paired
    python robot_ui.py AA:BB:CC:DD:EE:FF      # chỉ định MAC
    python robot_ui.py --port 8800            # đổi cổng HTTP

Vì sao phải là Python giữ kết nối rồi phục vụ một trang, chứ không phải một
trang web thuần: socket RFCOMM tới HC-05 chỉ tiến trình cục bộ mở được, trình
duyệt không với tới Bluetooth. Nên Python giữ socket, trình duyệt chỉ là mặt
tiền và nói chuyện với Python qua HTTP trên localhost.

Chỉ dùng thư viện chuẩn cho phần máy chủ (http.server + SSE), không cần cài gì
thêm ngoài những thứ robot_control.py đã cần.

THIẾT KẾ: giao diện KHÔNG chứa danh sách tham số nào. Lúc kết nối nó gửi `P`,
đọc các dòng `TEN=giá_trị [min..max]` rồi tự dựng ô chỉnh cho từng tham số; cột
telemetry thì lấy từ dòng `#COLS`. Thêm một tham số vào bảng PARAMS trong
firmware là nó tự hiện ra ở đây, y như với robot_control.py.
"""
import argparse
import json
import os
import re
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_control import CONFIG_DIR, choose_device, load_config  # noqa: E402

MM_PER_STEP = 0.177
PARAM_RE = re.compile(r"^([A-Z][A-Z0-9_]*)=(-?[\d.]+)(?:\s*\[(-?[\d.]+)\.\.(-?[\d.]+)\])?")
REPLY_RE = re.compile(r"^> ([A-Z][A-Z0-9_]*)=(-?[\d.]+)")

# Tham số hay dùng nhất, đẩy lên đầu danh sách. Tên nào không có ở đây vẫn hiện,
# chỉ là nằm dưới — nên thêm tham số mới vào firmware không cần sửa file này.
PRIORITY = ["KP", "KD", "KI", "KPV", "KIV", "KPPOS", "VMAX", "DRVVEL",
            "MTO", "BIAS", "SPD", "ACC", "VFA", "UPOFF"]


class Link:
    """Giữ socket RFCOMM, đọc nền, gom trạng thái cho giao diện."""

    def __init__(self, mac):
        self.mac = mac
        self.sock = None
        self.lock = threading.Lock()
        self.buf = ""
        self.cols = []
        self.row = {}            # cột -> giá trị, mẫu mới nhất
        self.params = {}         # TÊN -> {value, min, max}
        self.msgs = []           # (t, dòng chữ)
        self.state = "?"
        self.connected = False
        self.samples = 0
        self.last_rx = 0.0
        self.origin = None       # mốc vị trí, để hiện quãng đường tương đối
        self.seq = 0             # tăng mỗi khi có gì đó mới, để client biết
        self.junk = 0            # số dòng bị bỏ vì vỡ/nhị phân

    # --- kết nối ---------------------------------------------------
    def connect(self):
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.settimeout(15)
        s.connect((self.mac, 1))
        s.settimeout(0.3)
        with self.lock:
            self.sock = s
            self.connected = True
            self.buf = ""
        self.note(f"đã kết nối {self.mac}")
        # Hỏi ngay bộ cột và toàn bộ tham số: firmware chỉ tự phát #COLS lúc khởi
        # động và lúc đổi log_mask, nên nối vào giữa chừng là phải hỏi.
        self.send("COLS")
        time.sleep(0.3)
        self.send("P")

    def send(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return False
        with self.lock:
            if not self.sock:
                return False
            try:
                self.sock.sendall((cmd + "\n").encode())
            except OSError as e:
                self.note(f"gửi lỗi: {e}")
                self.connected = False
                return False
        return True

    @staticmethod
    def clean(text):
        """Lọc rác trước khi đưa vào nhật ký.

        Đường BT thỉnh thoảng trả về khung vỡ: hàng trăm byte NUL và nhị phân.
        Nếu nhét thẳng vào nhật ký thì mỗi gói SSE phồng lên vài KB và được gửi
        20 lần/giây — đã đo thấy đúng hiện tượng đó. Giữ ký tự in được, cắt
        ngắn, và bỏ hẳn dòng nào quá nửa là rác.
        """
        keep = "".join(c for c in text if c == "	" or " " <= c <= "~" or ord(c) > 0xA0)
        if not keep or len(keep) < len(text) * 0.6:
            return None
        return keep[:200]

    def note(self, text):
        text = self.clean(text)
        if text is None:
            with self.lock:
                self.junk += 1
            return
        with self.lock:
            self.msgs.append((time.time(), text))
            del self.msgs[:-200]
            self.seq += 1

    # --- vòng đọc --------------------------------------------------
    def reader(self):
        while True:
            if not self.connected:
                try:
                    self.note("đang nối lại...")
                    self.connect()
                except Exception as e:
                    self.note(f"nối lỗi: {type(e).__name__}")
                    time.sleep(3)
                    continue
            try:
                data = self.sock.recv(4096)
                if not data:
                    raise OSError("peer closed")
            except socket.timeout:
                continue
            except OSError:
                with self.lock:
                    self.connected = False
                    self.sock = None
                self.note("rớt kết nối")
                time.sleep(2)
                continue
            self.buf += data.decode("utf-8", errors="replace")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                self.feed(line.strip("\r \x00"))

    def feed(self, line):
        if not line:
            return
        if line.startswith("#COLS"):
            with self.lock:
                self.cols = line.split()[1:]
                self.seq += 1
            self.note("cột: " + " ".join(self.cols))
            return

        parts = line.split("\t")
        if self.cols and len(parts) == len(self.cols):
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                pass
            else:
                with self.lock:
                    self.row = dict(zip(self.cols, vals))
                    self.samples += 1
                    self.last_rx = time.time()
                return

        if line[:1].isdigit() and "	" in line:
            with self.lock:
                self.junk += 1          # gói BT bị cắt giữa dòng, không phải chữ
            return

        m = PARAM_RE.match(line) or REPLY_RE.match(line)
        if m:
            name, val = m.group(1), float(m.group(2))
            lo = hi = None
            if m.re is PARAM_RE and m.group(3) is not None:
                lo, hi = float(m.group(3)), float(m.group(4))
            with self.lock:
                p = self.params.setdefault(name, {})
                p["value"] = val
                if lo is not None:
                    p["min"], p["max"] = lo, hi
                self.seq += 1
            if m.re is REPLY_RE:
                self.note(line)
            return

        if "RECOVERED" in line:
            # Firmware zero stepsL/stepsR khi phuc hoi, neu khong bo moc thi o
            # "vi tri" nhay mot phat bang dung quang duong da di truoc do.
            self.origin = None
        elif "Bat dau" in line:
            with self.lock:
                self.state = "khởi động lại"
            self.note("!! ARDUINO KHỞI ĐỘNG LẠI")
            self.origin = None
            self.send("COLS")
        self.note(line)

    # --- ảnh chụp trạng thái cho client ----------------------------
    def snapshot(self):
        with self.lock:
            row = dict(self.row)
            state = self.state
            fall = (self.params.get("FALL") or {}).get("value")
            stale = self.last_rx and time.time() - self.last_rx > 1.5
        # Trạng thái suy ra từ GÓC, không từ việc có telemetry hay không.
        # Firmware cũ im lặng khi ngã nên "có telemetry = đang đứng" từng đúng;
        # bản 2026-10-26 phát telemetry cả khi nằm, nên suy luận đó hết đúng.
        if not self.connected:
            state = "mất kết nối"
        elif stale:
            state = "im lặng"
        elif fall and "angle" in row:
            state = "NẰM" if abs(row["angle"]) > fall else "ĐANG CÂN BẰNG"

        with self.lock:
            out = {
                "connected": self.connected,
                "state": state,
                "samples": self.samples,
                "junk": self.junk,
                "cols": list(self.cols),
                "row": row,
                "params": {k: dict(v) for k, v in self.params.items()},
                "msgs": [{"t": t, "s": s} for t, s in self.msgs[-40:]],
                "seq": self.seq,
            }
        if "stepsL" in row and "stepsR" in row:
            pos = (row["stepsL"] + row["stepsR"]) / 2 * MM_PER_STEP / 10.0
            if self.origin is None:
                self.origin = pos
            out["pos_cm"] = pos - self.origin
        if "angle" in row and "accel" in row:
            out["drift"] = row["angle"] - row["accel"]
        return out


def list_configs():
    try:
        return sorted(f[:-4] for f in os.listdir(CONFIG_DIR) if f.endswith(".txt"))
    except OSError:
        return []


# ─────────────────────────────────────────────────────────────────
# HTTP
# ─────────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    link: Link = None

    def log_message(self, *a):
        pass                                  # im lặng, log của ta nằm trong UI

    def _send(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE)
        elif self.path == "/state":
            self._send(200, "application/json", json.dumps(self.link.snapshot()))
        elif self.path == "/configs":
            self._send(200, "application/json", json.dumps(list_configs()))
        elif self.path == "/events":
            self.sse()
        else:
            self._send(404, "text/plain", "no")

    def sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                # 20Hz là đủ cho mắt người. Đẩy cả 100Hz lên trình duyệt chỉ tốn
                # băng thông và làm trang giật, trong khi file .tsv mới là chỗ
                # cần đủ 100Hz để phân tích.
                payload = json.dumps(self.link.snapshot())
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n).decode("utf-8") if n else ""
        if self.path == "/cmd":
            ok = self.link.send(body)
            self._send(200, "application/json", json.dumps({"ok": ok}))
        elif self.path == "/cfg":
            try:
                cmds = load_config(body.strip())
            except OSError as e:
                self._send(200, "application/json", json.dumps({"ok": False, "err": str(e)}))
                return
            threading.Thread(target=self._send_config, args=(body.strip(), cmds),
                             daemon=True).start()
            self._send(200, "application/json", json.dumps({"ok": True, "n": len(cmds)}))
        else:
            self._send(404, "text/plain", "no")

    def _send_config(self, name, cmds):
        self.link.note(f"nạp configs/{name}.txt ({len(cmds)} lệnh)")
        for c in cmds:
            self.link.send(c.strip())
            time.sleep(0.12)
        self.link.note(f"đã nạp xong {name}")
        time.sleep(0.3)
        self.link.send("P")


PAGE = r"""<!DOCTYPE html>
<html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Balance</title>
<style>
:root{--bg:#11141a;--card:#1a1f29;--line:#2a3140;--fg:#e6eaf2;--dim:#8b95a8;
      --ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--acc:#60a5fa}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 ui-sans-serif,system-ui,"Segoe UI",sans-serif}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;
       border-bottom:1px solid var(--line);position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.3px}
.badge{padding:3px 10px;border-radius:99px;font-size:12px;font-weight:600}
.wrap{display:grid;grid-template-columns:1fr 340px;gap:14px;padding:14px;align-items:start}
@media(max-width:860px){.wrap{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{font-size:11px;text-transform:uppercase;letter-spacing:.8px;color:var(--dim);
         margin:0 0 10px;font-weight:600}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(88px,1fr));gap:8px}
.stat{background:#141922;border:1px solid var(--line);border-radius:8px;padding:8px 10px}
.stat .k{font-size:10px;color:var(--dim);text-transform:uppercase;letter-spacing:.5px}
.stat .v{font-size:19px;font-variant-numeric:tabular-nums;font-weight:600;margin-top:2px}
canvas{width:100%;height:170px;display:block;background:#0e1117;
       border:1px solid var(--line);border-radius:8px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;max-width:250px;margin:0 auto}
button{background:#232b38;color:var(--fg);border:1px solid var(--line);border-radius:8px;
       padding:10px;font:inherit;font-weight:600;cursor:pointer}
button:hover{background:#2c3545;border-color:#3b4659}
button:active{transform:translateY(1px)}
button.stop{background:#3b1d1d;border-color:#6b2b2b;color:#ffb4b4}
button.go{background:#17304a;border-color:#2b5580}
.row{display:flex;flex-wrap:wrap;gap:6px}
.row button{flex:1 1 auto;padding:7px 10px;font-size:12.5px;font-weight:500}
.p{display:grid;grid-template-columns:70px 1fr 74px;gap:7px;align-items:center;margin-bottom:5px}
.p label{font-size:12px;color:var(--dim);font-family:ui-monospace,monospace}
.p input[type=range]{width:100%}
.p input[type=number]{background:#0e1117;color:var(--fg);border:1px solid var(--line);
       border-radius:6px;padding:4px 6px;width:100%;font:12px ui-monospace,monospace}
#log{height:150px;overflow:auto;background:#0e1117;border:1px solid var(--line);
     border-radius:8px;padding:8px;font:11.5px/1.5 ui-monospace,monospace;color:var(--dim)}
#log div{white-space:pre-wrap;word-break:break-word}
.hint{font-size:11.5px;color:var(--dim);margin-top:8px}
kbd{background:#232b38;border:1px solid var(--line);border-radius:4px;padding:0 5px;
    font:11px ui-monospace,monospace}
</style></head><body>

<header>
  <h1>Robot Balance</h1>
  <span class="badge" id="conn">…</span>
  <span class="badge" id="state">…</span>
  <span style="color:var(--dim);font-size:12px" id="rate"></span>
  <span style="flex:1"></span>
  <button class="stop" style="padding:6px 14px" onclick="cmd('S')">DỪNG (space)</button>
</header>

<div class="wrap">
  <div style="display:grid;gap:14px">
    <div class="card">
      <h2>Số đo trực tiếp</h2>
      <div class="grid" id="stats"></div>
    </div>
    <div class="card">
      <h2>Góc nghiêng &amp; đích — 10 giây gần nhất</h2>
      <canvas id="chart"></canvas>
      <div class="hint">
        <span style="color:var(--acc)">▬</span> angle
        &nbsp; <span style="color:var(--warn)">▬</span> target
        &nbsp; <span style="color:var(--dim)">▬</span> ±1°
      </div>
    </div>
    <div class="card">
      <h2>Nhật ký</h2>
      <div id="log"></div>
    </div>
  </div>

  <div style="display:grid;gap:14px">
    <div class="card">
      <h2>Lái</h2>
      <div class="pad">
        <span></span><button class="go" onclick="cmd('F')">▲</button><span></span>
        <button class="go" onclick="cmd('L')">◀</button>
        <button class="stop" onclick="cmd('S')">■</button>
        <button class="go" onclick="cmd('R')">▶</button>
        <span></span><button class="go" onclick="cmd('B')">▼</button><span></span>
      </div>
      <div class="hint">Phím mũi tên lái, <kbd>space</kbd> dừng.
        Mỗi lần bấm cộng thêm <code>DRVSTEP</code>; giữ phím là tăng dần tới trần.</div>
    </div>

    <div class="card">
      <h2>Lệnh</h2>
      <div class="row" style="margin-bottom:6px">
        <button onclick="cmd('P')">đọc tham số</button>
        <button onclick="cmd('CAL')">hiệu chuẩn gyro</button>
      </div>
      <div class="row" style="margin-bottom:6px">
        <button onclick="cmd('SAVE')">lưu EEPROM</button>
        <button onclick="cmd('LOAD')">nạp EEPROM</button>
        <button onclick="cmd('RESET')">mặc định</button>
      </div>
      <div class="row" id="cfgs"></div>
      <div class="hint"><code>SAVE</code> và <code>CAL</code> chỉ chạy khi robot đang
        <b>nằm</b> — firmware chặn, vì ghi EEPROM chặn ~150ms là đủ làm đổ robot.</div>
      <div class="row" style="margin-top:8px">
        <input id="raw" placeholder="gõ lệnh thô, vd: kd 6"
          style="flex:1;background:#0e1117;color:var(--fg);border:1px solid var(--line);
                 border-radius:6px;padding:7px;font:12px ui-monospace,monospace">
      </div>
    </div>

    <div class="card">
      <h2>Tham số <span style="text-transform:none;letter-spacing:0">(lấy từ firmware)</span></h2>
      <div id="params"></div>
    </div>
  </div>
</div>

<script>
const $ = s => document.querySelector(s);
let S = {}, hist = [], builtParams = "", lastSeq = -1;

async function cmd(c){ await fetch('/cmd',{method:'POST',body:c}); }
async function cfg(n){ await fetch('/cfg',{method:'POST',body:n}); }

// ── bàn phím ────────────────────────────────────────────────────
const KEYS = {ArrowUp:'F', ArrowDown:'B', ArrowLeft:'L', ArrowRight:'R', ' ':'S'};
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  const c = KEYS[e.key];
  if (c) { e.preventDefault(); cmd(c); }
});
$('#raw').addEventListener('keydown', e => {
  if (e.key === 'Enter' && e.target.value.trim()) { cmd(e.target.value); e.target.value=''; }
});

// ── luồng trạng thái ────────────────────────────────────────────
new EventSource('/events').onmessage = ev => { S = JSON.parse(ev.data); render(); };

fetch('/configs').then(r=>r.json()).then(list => {
  $('#cfgs').innerHTML = list.map(n =>
    `<button onclick="cfg('${n}')">cfg ${n}</button>`).join('');
});

// ── hiển thị ────────────────────────────────────────────────────
const FMT = {angle:2, accel:2, target:2, gyro:1, speed:0, stepsL:0, stepsR:0,
             pidout:0, ctrlout:0, ivel:1, iang:2, velf:1, velerr:1, drive:2, turn:0, dt:4};
const SHOW = ['angle','target','gyro','speed','drive','turn'];

function render(){
  const c = $('#conn'), st = $('#state');
  c.textContent = S.connected ? 'đã nối' : 'mất kết nối';
  c.style.background = S.connected ? '#14341f' : '#3b1d1d';
  c.style.color = S.connected ? 'var(--ok)' : 'var(--bad)';
  st.textContent = S.state || '?';
  const bal = S.state === 'ĐANG CÂN BẰNG';
  st.style.background = bal ? '#14341f' : '#3a2c12';
  st.style.color = bal ? 'var(--ok)' : 'var(--warn)';
  $('#rate').textContent = (S.samples||0) + ' mẫu'
      + (S.junk ? '  ·  ' + S.junk + ' dòng vỡ' : '');

  // thẻ số: các cột quen thuộc trước, rồi mọi cột khác firmware đang gửi
  const row = S.row || {}, cols = S.cols || [];
  const order = SHOW.filter(k => k in row).concat(cols.filter(k => !SHOW.includes(k) && k!=='ms'));
  let html = order.map(k =>
    `<div class="stat"><div class="k">${k}</div><div class="v">${
      row[k]===undefined ? '–' : row[k].toFixed(FMT[k] ?? 2)}</div></div>`).join('');
  if (S.pos_cm !== undefined)
    html += `<div class="stat"><div class="k">vị trí</div><div class="v">${
      S.pos_cm.toFixed(2)}<span style="font-size:12px;color:var(--dim)"> cm</span></div></div>`;
  if (S.drift !== undefined)
    html += `<div class="stat"><div class="k">drift</div><div class="v" style="color:${
      Math.abs(S.drift)>1 ? 'var(--bad)' : 'inherit'}">${S.drift.toFixed(2)}</div></div>`;
  $('#stats').innerHTML = html;

  if ('angle' in row) {
    hist.push([row.angle, row.target ?? 0]);
    if (hist.length > 200) hist.shift();      // 200 mẫu @20Hz = 10 giây
    draw();
  }

  // Chỉ dựng lại ô chỉnh khi DANH SÁCH tham số đổi, nếu không thì mỗi khung
  // hình lại xoá mất ô người dùng đang gõ dở.
  const names = Object.keys(S.params || {});
  const key = names.join(',');
  if (key !== builtParams) { buildParams(names); builtParams = key; }
  else if (S.seq !== lastSeq) { syncParams(); }
  lastSeq = S.seq;

  $('#log').innerHTML = (S.msgs||[]).map(m =>
    `<div>${new Date(m.t*1000).toLocaleTimeString()}  ${esc(m.s)}</div>`).join('');
}

function esc(s){ return s.replace(/[<>&]/g, c => ({'<':'&lt;','>':'&gt;','&':'&amp;'}[c])); }

const PRIO = %%PRIORITY%%;
function buildParams(names){
  names.sort((a,b) => {
    const ia = PRIO.indexOf(a), ib = PRIO.indexOf(b);
    if (ia>=0 && ib>=0) return ia-ib;
    if (ia>=0) return -1;
    if (ib>=0) return 1;
    return a.localeCompare(b);
  });
  $('#params').innerHTML = names.map(n => {
    const p = S.params[n];
    const lo = p.min ?? 0, hi = p.max ?? 100;
    const step = (hi-lo) > 50 ? 1 : (hi-lo) > 5 ? 0.1 : 0.001;
    return `<div class="p">
      <label title="${lo} .. ${hi}">${n}</label>
      <input type="range" id="r_${n}" min="${lo}" max="${hi}" step="${step}"
             value="${p.value}" oninput="pset('${n}',this.value,1)">
      <input type="number" id="n_${n}" step="${step}"
             value="${p.value}" onchange="pset('${n}',this.value,0)"></div>`;
  }).join('');
}

let pending = {};
function pset(name, val, fromSlider){
  // Kéo thanh trượt sinh hàng chục sự kiện mỗi giây. Gửi hết thì tràn đường
  // truyền và đẩy telemetry ra khỏi băng thông, nên gộp lại 150ms một lần.
  document.getElementById((fromSlider?'n_':'r_')+name).value = val;
  pending[name] = val;
  clearTimeout(pending['_t']);
  pending['_t'] = setTimeout(() => {
    for (const k in pending) if (k !== '_t') cmd(k + ' ' + pending[k]);
    pending = {};
  }, 150);
}

function syncParams(){
  for (const n in (S.params||{})) {
    const r = document.getElementById('r_'+n), b = document.getElementById('n_'+n);
    if (!r || document.activeElement === r || document.activeElement === b) continue;
    if (n in pending) continue;
    r.value = S.params[n].value; b.value = S.params[n].value;
  }
}

// ── đồ thị ──────────────────────────────────────────────────────
function draw(){
  const cv = $('#chart'), dpr = devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== w*dpr) { cv.width = w*dpr; cv.height = h*dpr; }
  const g = cv.getContext('2d');
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  let m = 3;
  for (const [a,t] of hist) m = Math.max(m, Math.abs(a), Math.abs(t));
  m = Math.ceil(m);
  const y = v => h/2 - v/m * (h/2 - 8);

  g.strokeStyle = '#2a3140'; g.lineWidth = 1;
  g.beginPath(); g.moveTo(0,y(0)); g.lineTo(w,y(0)); g.stroke();
  g.strokeStyle = '#1e2635'; g.setLineDash([3,4]);
  for (const v of [1,-1]) { g.beginPath(); g.moveTo(0,y(v)); g.lineTo(w,y(v)); g.stroke(); }
  g.setLineDash([]);

  g.fillStyle = '#8b95a8'; g.font = '10px ui-monospace,monospace';
  g.fillText('+'+m+'°', 4, 12); g.fillText('-'+m+'°', 4, h-4);

  for (const [idx,col] of [[1,'#fbbf24'],[0,'#60a5fa']]) {
    g.strokeStyle = col; g.lineWidth = 1.5; g.beginPath();
    hist.forEach((p,i) => {
      const x = i/(200-1)*w;
      i ? g.lineTo(x,y(p[idx])) : g.moveTo(x,y(p[idx]));
    });
    g.stroke();
  }
}
</script></body></html>
"""
PAGE = PAGE.replace("%%PRIORITY%%", json.dumps(PRIORITY))


def main():
    ap = argparse.ArgumentParser(description="Giao diện điều khiển robot trong trình duyệt")
    ap.add_argument("mac", nargs="?", help="MAC của HC-05")
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()

    mac = choose_device(a.mac)
    if not mac:
        print("Không chọn được thiết bị.")
        return 1

    link = Link(mac)
    Handler.link = link
    threading.Thread(target=link.reader, daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://127.0.0.1:{a.port}/"
    print(f"Giao diện: {url}")
    print("Ctrl+C để dừng.")
    if not a.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\ndừng")
    return 0


if __name__ == "__main__":
    sys.exit(main())
