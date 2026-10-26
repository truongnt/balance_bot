#!/usr/bin/env python3
"""
Robot Balance Controller
========================
Kết nối Bluetooth HC-05 qua RFCOMM, hiển thị telemetry live, gửi lệnh.

Cài đặt (không cần pybluez):
    pip install rich prompt_toolkit

Sử dụng:
    python robot_control.py                   # scan paired BT devices trên Windows
    python robot_control.py AA:BB:CC:DD:EE:FF # chỉ định MAC thẳng

Lưu ý:
    HC-05 phải được PAIRED trong Windows Settings → Bluetooth trước.
    Khi script kết nối, Windows sẽ hiện "connected" — đây là bình thường.
    Mỗi lần close socket Windows sẽ hiện "disconnected" — cũng bình thường.
"""

import sys

# Fix UTF-8 output trên Windows ngay khi load module (trước rich/prompt_toolkit)
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import os
import re
import json
import time
import socket
import subprocess
import threading
import ctypes
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from rich.console import Console
from rich.table import Table
from rich.text import Text
from rich.panel import Panel
from rich import box

from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.styles import Style
from prompt_toolkit.formatted_text import HTML

# ─────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────
RFCOMM_PORT    = 1       # HC-05 luôn dùng RFCOMM channel 1
BT_RECV_SIZE   = 256
STATUS_EVERY   = 50      # ghi log mỗi N mẫu (~0.5 giây ở 100Hz)
# Telemetry chỉ vào log file, KHÔNG in console → prompt luôn visible

# Tên thiết bị HC-05 thường gặp (highlight khi hiện danh sách)
HC05_NAMES = {"hc-05", "hc05", "hc-06", "hc06", "linvor", "robot"}

# ─────────────────────────────────────────────────────────────────
# CẤU HÌNH — đọc từ configs/*.txt, KHÔNG chép tay vào đây
# ─────────────────────────────────────────────────────────────────
# Một nguồn sự thật duy nhất: sửa file .txt, không sửa code.
#   configs/balance_good.txt  — bộ đã đo: angle σ 0.272°, 99% trong ±1°
# Nạp lúc chạy bằng lệnh:  cfg balance_good
CONFIG_DIR     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs")
DEFAULT_CONFIG = "balance_good"


def load_config(name: str) -> list[str]:
    """Đọc configs/<name>.txt -> danh sách lệnh kèm newline.

    Bỏ qua dòng trống và dòng bắt đầu bằng '#'. Giữ nguyên thứ tự trong file:
    thứ tự có ý nghĩa (ví dụ 'S' phải nằm cuối để chốt mốc vị trí).
    """
    path = os.path.join(CONFIG_DIR, name + ".txt")
    out: list[str] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                out.append(line.upper() + "\n")
    return out


try:
    STARTUP_PARAMS = load_config(DEFAULT_CONFIG)
except OSError as e:
    # Thiếu file thì vẫn chạy được, chỉ là không tự gửi tham số
    print(f"[!] khong doc duoc config {DEFAULT_CONFIG}: {e}")
    STARTUP_PARAMS = []

console  = Console(highlight=False, legacy_windows=False)
_log_fh  = None   # transcript phiên (sự kiện + 1 dòng tóm tắt mỗi 50 mẫu)
_raw_fh  = None   # telemetry THÔ, mọi mẫu 100Hz — dùng để phân tích/vẽ đồ thị
_t0      = 0.0    # mốc thời gian, để cột t_ms trong file thô
_raw_cols = None  # bộ cột hiện tại, do Arduino báo qua "#COLS ..."


def open_log_file() -> tuple[str, str]:
    """Mo 2 file log trong thu muc logs/. Tra ve (transcript, raw)."""
    global _log_fh, _raw_fh, _t0, _raw_cols
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    fname = os.path.join(log_dir, f"robot_{stamp}.log")
    _log_fh = open(fname, "w", encoding="utf-8", buffering=1)  # line-buffered

    # File tho: moi mau mot dong. Header viet khi nhan duoc "#COLS" tu Arduino.
    rawname = os.path.join(log_dir, f"robot_{stamp}_raw.tsv")
    _raw_fh = open(rawname, "w", encoding="utf-8", buffering=4096)
    _raw_cols = None

    _t0 = time.perf_counter()
    return fname, rawname


def set_raw_cols(cols: list[str]):
    """Arduino bao doi bo cot. Lan dau -> viet header; sau do -> viet moc #COLS."""
    global _raw_cols
    if _raw_fh is None or cols == _raw_cols:
        return
    if _raw_cols is None:
        _raw_fh.write("t_ms\t" + "\t".join(cols) + "\n")
    else:
        # Đổi bộ cột giữa phiên → chèn mốc để script phân tích biết chỗ ngắt
        _raw_fh.write("#COLS t_ms " + " ".join(cols) + "\n")
    _raw_cols = cols


def log_raw(parts: list[str]):
    """Ghi 1 mau telemetry tho. Goi tu reader thread, phai nhanh."""
    if _raw_fh:
        t_ms = (time.perf_counter() - _t0) * 1000.0
        _raw_fh.write(f"{t_ms:.1f}\t" + "\t".join(parts) + "\n")


def safe_print(*args, **kwargs):
    """In ra terminal VÀ ghi vào log file."""
    console.print(*args, **kwargs)
    if _log_fh:
        text = " ".join(str(a) for a in args)
        _log_fh.write(Text.from_markup(text).plain + "\n")


def log_only(*args):
    """Chỉ ghi vào log file, KHÔNG in terminal — dùng cho telemetry dày."""
    if _log_fh:
        text = " ".join(str(a) for a in args)
        _log_fh.write(Text.from_markup(text).plain + "\n")


# ─────────────────────────────────────────────────────────────────
# TELEMETRY STATE
# ─────────────────────────────────────────────────────────────────
# Tên cột (từ VARS[] trong firmware) → thuộc tính của Telemetry.
# Cột nào không có ở đây vẫn được ghi vào file .tsv, chỉ là không hiện trên panel.
_TELEM_FIELDS = {
    "angle": "angle", "accel": "accel", "target": "target_angle",
    "gyro": "gyro_rate", "speed": "speed",
    "stepsL": "steps_l", "stepsR": "steps_r",
    "drive": "drive", "turn": "turn",
}
_INT_FIELDS = {"speed", "steps_l", "steps_r", "turn"}
# "ms" cố tình KHÔNG có trong _TELEM_FIELDS: nó chỉ cần vào file .tsv.

# Bộ cột mặc định — khớp log_mask mặc định (0xFF) của firmware.
# Dùng khi kết nối vào giữa phiên và chưa kịp nhận "#COLS".
DEFAULT_COLS = ["ms", "angle", "accel", "target", "gyro", "speed", "stepsL", "stepsR"]


@dataclass
class Telemetry:
    angle:        float = 0.0
    accel:        float = 0.0
    target_angle: float = 0.0
    gyro_rate:    float = 0.0
    speed:        int   = 0
    steps_l:      int   = 0
    steps_r:      int   = 0
    kp_angle:     float = 0.0
    kd_angle:     float = 0.0
    kp_vel:       float = 0.0
    ki_vel:       float = 0.0
    drive:        float = 0.0
    turn:         int   = 0
    state:        str   = "CONNECTING"
    last_update:  float = 0.0
    sample_count: int   = 0


# ─────────────────────────────────────────────────────────────────
# BLUETOOTH RFCOMM CONNECTION
# ─────────────────────────────────────────────────────────────────
class RobotBT:
    def __init__(self, mac: str):
        self.mac = mac.upper()
        self._sock: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.telem = Telemetry()
        self._lock = threading.Lock()
        self._pending_startup = False   # gửi startup params sau khi "Bat dau!" nhận được
        self._cols = list(DEFAULT_COLS) # cập nhật khi Arduino gửi "#COLS ..."
        # (BIAS_HINT da bo — xem ghi chu o STARTUP_PARAMS)

    def connect(self):
        console.print(f"[cyan]Đang kết nối tới [bold]{self.mac}[/bold] ...[/cyan]")
        # Dùng socket.AF_BLUETOOTH có sẵn trong Python 3.3+ (không cần pybluez)
        self._sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self._sock.settimeout(10)
        self._sock.connect((self.mac, RFCOMM_PORT))
        self._sock.settimeout(2)          # timeout cho recv sau khi kết nối
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True, name="bt-reader")
        self._thread.start()
        console.print(f"[green]✓ Kết nối thành công![/green]")

    def disconnect(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        console.print("[yellow]Đã ngắt kết nối Bluetooth.[/yellow]")

    @property
    def connected(self) -> bool:
        return self._sock is not None and self._running

    def send(self, cmd: str):
        if not self.connected:
            console.print("[red]Lỗi: chưa kết nối![/red]")
            return
        try:
            self._sock.sendall(cmd.encode("utf-8"))
        except OSError as e:
            console.print(f"[red]Lỗi gửi lệnh: {e}[/red]")

    # ── reader thread ────────────────────────────────────────────
    def _reader(self):
        buf = ""
        while self._running:
            try:
                data = self._sock.recv(BT_RECV_SIZE)
                if not data:
                    console.print("[yellow]Mất kết nối BT.[/yellow]")
                    self._running = False
                    break
                buf += data.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r ")
                    if line:
                        self._parse_line(line)
            except socket.timeout:
                continue
            except OSError as e:
                if self._running:
                    console.print(f"[red][BT ERR] {e}[/red]")
                self._running = False
                break

    def _parse_line(self, line: str):
        with self._lock:
            t = self.telem

            # ── Arduino báo bộ cột telemetry: "#COLS angle accel target ..."
            # Gửi lúc khởi động và mỗi lần log_mask đổi. Đây là NGUỒN DUY NHẤT
            # quyết định layout — Python không hardcode cột nào nữa.
            if line.startswith("#COLS"):
                self._cols = line.split()[1:]
                set_raw_cols(self._cols)
                safe_print(f"[dim blue]>> cột telemetry ({len(self._cols)}): "
                           f"{' '.join(self._cols)}[/dim blue]")
                return

            # Garbage: ký tự non-printable hoặc toàn binary
            printable = sum(1 for c in line if c.isprintable())
            if len(line) > 4 and printable / len(line) < 0.7:
                return   # >30% ký tự lạ → garbage, bỏ qua

            # ── Telemetry: số cột phải khớp với #COLS gần nhất ──
            parts = line.split("\t")
            if len(parts) > 1 and len(parts) == len(self._cols):
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    pass
                else:
                    for name, v in zip(self._cols, vals):
                        attr = _TELEM_FIELDS.get(name)
                        if attr:
                            setattr(t, attr, int(v) if attr in _INT_FIELDS else v)
                    t.last_update  = time.time()
                    t.sample_count += 1
                    log_raw(parts)      # ← mọi mẫu vào file .tsv riêng
                    if t.state not in ("FALLEN", "CALIBRATING"):
                        t.state = "BALANCING"
                    # In status 1 dòng mỗi STATUS_EVERY mẫu
                    if t.sample_count % STATUS_EVERY == 0:
                        self._print_status(t)
                    return

            # ── Sự kiện ──────────────────────────────────────────
            if "[!] FALLEN" in line:
                t.state = "FALLEN"
                safe_print(f"[bold red]{line}[/bold red]")
            elif "[+] RECOVERED" in line:
                t.state = "BALANCING"
                safe_print(f"[bold green]{line}[/bold green]")
            elif "> drive=" in line:
                m = re.search(r"drive=([-\d.]+)", line)
                if m: t.drive = float(m.group(1))
                safe_print(f"[cyan]{line}[/cyan]")
            elif "> turn=" in line:
                m = re.search(r"turn=([-\d]+)", line)
                if m: t.turn = int(m.group(1))
                safe_print(f"[cyan]{line}[/cyan]")
            elif "> STOP" in line:
                t.drive = 0; t.turn = 0
                safe_print(f"[yellow]{line}[/yellow]")
            elif _PARAM_LINE_RE.match(line):
                # Arduino in từng tham số một: "KP=80.0000 [0.00..500.00]"
                m    = _PARAM_LINE_RE.match(line)
                name = m.group(1)
                if name in _PANEL_FIELDS:
                    try:
                        setattr(t, _PANEL_FIELDS[name], float(m.group(2)))
                    except ValueError:
                        pass
                safe_print(f"[magenta]{line}[/magenta]")
            elif "Calib:" in line or "GIU robot" in line:
                t.state = "CALIBRATING"
                safe_print(f"[blue]{line}[/blue]")
            elif "Bat dau!" in line:
                t.state = "BALANCING"
                safe_print(f"[bold green]{line}[/bold green]")
                self._pending_startup = True   # trigger gửi params
            elif any(k in line for k in ("=== ROBOT", "MPU OK", "gyroBias", "upright_offset", "Goc ban dau")):
                safe_print(f"[dim blue]{line}[/dim blue]")
            elif line.startswith("> "):
                # Phản hồi tuning từ Arduino ("> kp=85.00", "> SAVED", "> bias=0.30", ...)
                safe_print(f"[bold magenta]{line}[/bold magenta]")

    @staticmethod
    def _print_status(t: "Telemetry"):
        sign = "+" if t.angle >= 0 else ""
        state_colors = {"BALANCING": "green", "FALLEN": "red", "CALIBRATING": "yellow"}
        sc = state_colors.get(t.state, "white")
        # drift = CF angle vs accel angle: >0.5° cảnh báo filter đang lệch
        drift = t.angle - t.accel
        drift_str = f" [yellow]drift{drift:+.1f}°[/yellow]" if abs(drift) > 0.5 else ""
        line = (
            f"[dim]#{t.sample_count:<5}[/dim] [{sc}]{t.state:<10}[/{sc}] "
            f"[white]{sign}{t.angle:5.2f}°[/white] "
            f"[dim]tgt[/dim][white]{t.target_angle:+5.2f}°[/white] "
            f"[dim]spd[/dim][white]{t.speed:+5d}[/white] "
            f"[dim]gyro[/dim][white]{t.gyro_rate:+5.1f}°/s[/white]"
            f"{drift_str}"
        )
        # Telemetry chỉ ghi log file — KHÔNG in console để giữ prompt visible
        log_only(line)


# ─────────────────────────────────────────────────────────────────
# BLUETOOTH DEVICE SCAN — dùng PowerShell, không cần pybluez
# ─────────────────────────────────────────────────────────────────
_PS_SCAN = r"""
$devs = Get-PnpDevice -Class Bluetooth -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -match 'DEV_' }
$result = @()
foreach ($d in $devs) {
    if ($d.InstanceId -match 'DEV_([0-9A-Fa-f]{12})') {
        $raw = $Matches[1].ToUpper()
        $mac = ($raw -split '(.{2})' | Where-Object { $_ }) -join ':'
        $result += [PSCustomObject]@{ mac=$mac; name=$d.FriendlyName }
    }
}
if ($result.Count -gt 0) { $result | ConvertTo-Json -Compress }
"""

def scan_paired_windows() -> list[tuple[str, str]]:
    """Lấy danh sách thiết bị BT đã paired trên Windows qua PowerShell."""
    try:
        r = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS_SCAN],
            capture_output=True, text=True, timeout=15,
        )
        out = r.stdout.strip()
        if not out:
            return []
        data = json.loads(out)
        if isinstance(data, dict):   # 1 thiết bị → dict thay vì list
            data = [data]
        return [(d["mac"], d.get("name") or "") for d in data if d.get("mac")]
    except Exception:
        return []

def _validate_mac(s: str) -> Optional[str]:
    mac = s.strip().upper().replace("-", ":")
    if re.fullmatch(r"([0-9A-F]{2}:){5}[0-9A-F]{2}", mac):
        return mac
    return None

def choose_device(mac_arg: Optional[str] = None) -> Optional[str]:
    """Trả về MAC address để kết nối."""

    # MAC truyền từ command line
    if mac_arg:
        mac = _validate_mac(mac_arg)
        if mac:
            return mac
        console.print(f"[red]MAC không hợp lệ: {mac_arg}[/red]")
        console.print("Định dạng đúng: AA:BB:CC:DD:EE:FF")
        return None

    # Scan paired devices qua PowerShell
    console.print("[cyan]Đang lấy danh sách thiết bị BT đã paired...[/cyan]")
    devices = scan_paired_windows()

    if devices:
        console.print(f"\n[bold cyan]─── Thiết bị Bluetooth đã paired ({len(devices)}) ───[/bold cyan]")
        for i, (mac, name) in enumerate(devices):
            tag = "[green]← HC-05?[/green]" if name.lower() in HC05_NAMES else ""
            console.print(f"  [yellow]{i+1}[/yellow]. [white]{mac}[/white]  {name:<24} {tag}")
        console.print()
    else:
        console.print("[yellow]Không tìm thấy thiết bị nào đã paired.[/yellow]")
        console.print("Hãy vào Windows Settings → Bluetooth → Add device → chọn HC-05 (PIN: 1234)\n")

    while True:
        try:
            prompt = "Chọn số thứ tự hoặc nhập MAC (AA:BB:CC:DD:EE:FF): " if devices \
                     else "Nhập MAC address của HC-05: "
            choice = input(prompt).strip()
            if not choice:
                continue
            if devices and choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(devices):
                    return devices[idx][0]
                console.print("[red]Số không hợp lệ.[/red]")
                continue
            mac = _validate_mac(choice)
            if mac:
                return mac
            console.print("[red]Định dạng MAC sai. Ví dụ: 98:DA:60:01:23:45[/red]")
        except (KeyboardInterrupt, EOFError):
            return None


# ─────────────────────────────────────────────────────────────────
# DISPLAY
# ─────────────────────────────────────────────────────────────────
STATE_STYLE = {
    "BALANCING":   "bold green",
    "FALLEN":      "bold red",
    "CALIBRATING": "bold yellow",
    "CONNECTING":  "dim",
}

def angle_bar(angle: float, width: int = 24) -> str:
    limit  = 30.0
    pct    = max(-1.0, min(1.0, angle / limit))
    center = width // 2
    pos    = center + int(pct * center)
    bar    = ["-"] * width
    bar[center] = "|"
    if 0 <= pos < width:
        bar[pos] = "█"
    return "".join(bar)

def render_telemetry(t: Telemetry) -> Panel:
    tbl = Table(box=box.SIMPLE, show_header=False, padding=(0, 1))
    tbl.add_column("k",  style="bold cyan", no_wrap=True)
    tbl.add_column("v",  style="white",     no_wrap=True)
    tbl.add_column("k2", style="bold cyan", no_wrap=True)
    tbl.add_column("v2", style="white",     no_wrap=True)

    age     = time.time() - t.last_update if t.last_update else 999
    age_str = f"{age:.1f}s ago" if age < 60 else "---"
    sign    = "+" if t.angle >= 0 else ""
    bar     = angle_bar(t.angle)
    state   = Text(t.state, style=STATE_STYLE.get(t.state, "white"))

    tbl.add_row("Góc (angle)",   Text(f"{sign}{t.angle:6.2f}°  [{bar}]"),
                "Target angle",  f"{t.target_angle:+.2f}°")
    tbl.add_row("Accel angle",   f"{t.accel:+.2f}°",
                "Gyro rate",     f"{t.gyro_rate:+.1f}°/s")
    tbl.add_row("Speed",         f"{t.speed:+6d} step/s",
                "Steps L / R",   f"{t.steps_l} / {t.steps_r}")
    tbl.add_row("BT drive",      f"{t.drive:+.2f}°",
                "BT turn",       f"{t.turn:+d} step/s")
    tbl.add_row("Kp / Kd angle", f"{t.kp_angle} / {t.kd_angle}",
                "Kpv / Kiv",     f"{t.kp_vel:.4f} / {t.ki_vel:.4f}")
    tbl.add_row("Trạng thái",    state,
                "Samples",       f"{t.sample_count}  ({age_str})")

    return Panel(tbl, title="[bold]TELEMETRY[/bold]", border_style="cyan")

def print_help():
    console.print("""
[bold cyan]╔══════════════════════════════════════════════════════╗
║              ROBOT BALANCE — HƯỚNG DẪN SỬ DỤNG      ║
╚══════════════════════════════════════════════════════╝[/bold cyan]

[bold yellow]ĐIỀU KHIỂN[/bold yellow]
  [yellow]f[/yellow] / [yellow]b[/yellow]          Tiến / Lùi  (+0.4° mỗi lần)
  [yellow]l[/yellow] / [yellow]r[/yellow]          Rẽ trái / Rẽ phải
  [yellow]s[/yellow]             Dừng (reset drive + turn về 0)
  [yellow]f3[/yellow] [yellow]b2[/yellow] [yellow]r4[/yellow] ...  Gửi lệnh N lần liên tiếp

[bold yellow]LÁI ROBOT — lệnh có thời lượng / quãng đường[/bold yellow]
  [yellow]fwd[/yellow] [yellow]back[/yellow] [yellow]left[/yellow] [yellow]right[/yellow] [dim][giây][/dim]   Chạy rồi TỰ DỪNG  [dim](mặc định 1 giây)[/dim]
  [yellow]go <drive> <turn>[/yellow] [dim][giây][/dim]  Đặt thẳng cả hai. Không ghi giây thì chạy tới khi gõ [/dim][yellow]s[/yellow]
  [yellow]move <cm>[/yellow]      Đi tới khi ĐỦ quãng đường — đếm bước thật, không canh giờ
  [yellow]spin <độ>[/yellow]      Quay tới khi ĐỦ góc — đếm chênh lệch bước hai bánh
  [yellow]seq a; b; c[/yellow]    Chạy chuỗi lệnh lần lượt
  [dim]vd:[/dim] [yellow]seq fwd 2; spin 90; fwd 2; spin 90[/yellow][dim]  — đi một hình vuông[/dim]
  [dim]→ [/dim][yellow]move[/yellow][dim] và [/dim][yellow]spin[/yellow][dim] là VÒNG KÍN: đọc stepsL/stepsR từ telemetry rồi dừng
     đúng lúc, nên chính xác hơn canh giờ. Robot ngã giữa chừng thì tự dừng.
  → [/dim][yellow]spin[/yellow][dim] dùng WHEEL_TRACK_MM = 140.6mm (đo 25/10 từ lệch bánh
     khoảng cách hai bánh rồi sửa trong robot_control.py cho chuẩn)[/dim]

[bold yellow]THÔNG TIN[/bold yellow]
  [yellow]t[/yellow]             Xem bảng telemetry
  [yellow]get[/yellow]           Xem tất cả tham số hiện tại (từ robot)
  [yellow]p[/yellow]             In PID gains nhanh

[bold yellow]LIVE TUNING — CÂN BẰNG[/bold yellow]
  [yellow]bias <val>[/yellow]    Đặt độ nghiêng bù CG, dương=về trước  [dim](mặc định: 0.0)[/dim]
  [yellow]biasadj <±>[/yellow]   Cộng thêm vào bias hiện tại
  [dim]→ Robot đổ về sau: tăng bias dần (bias 0.3 → 0.5 → ...)[/dim]

[bold yellow]LIVE TUNING — PID ANGLE (vòng trong, 100Hz)[/bold yellow]
  [yellow]kp <val>[/yellow]      Kp_angle — tăng nếu cân bằng yếu    [dim](mặc định: 80)[/dim]
  [yellow]kd <val>[/yellow]      Kd_angle — tăng nếu dao động nhiều  [dim](mặc định: 12)[/dim]
  [yellow]ki <val>[/yellow]      Ki_angle — thường để 0               [dim](mặc định: 0)[/dim]

[bold yellow]LIVE TUNING — PID VELOCITY (vòng ngoài, 10Hz)[/bold yellow]
  [yellow]kpv <val>[/yellow]     Kp_vel — tốc độ bù drift             [dim](mặc định: 0.008)[/dim]
  [yellow]kiv <val>[/yellow]     Ki_vel — bù drift dài hạn            [dim](mặc định: 0.001)[/dim]

[bold yellow]LIVE TUNING — BỘ LỌC[/bold yellow]
  [yellow]alpha <val>[/yellow]   CF filter (0.93–0.98), thấp=tin accel hơn  [dim](mặc định: 0.97)[/dim]
  [yellow]alpf <val>[/yellow]    Accel LPF (0.3–0.9), thấp=ít lọc hơn      [dim](mặc định: 0.70)[/dim]
  [yellow]vfa <val>[/yellow]     Velocity filter (0–0.9)                    [dim](mặc định: 0.60)[/dim]
  [dim]→ Filter drift (ngã mà angle thấy nhỏ): thử alpha 0.95[/dim]

[bold yellow]LIVE TUNING — MOTOR[/bold yellow]
  [yellow]spd <val>[/yellow]     Max tốc độ motor (step/s)   [dim](mặc định: 5500)[/dim]
  [yellow]acc <val>[/yellow]     Max gia tốc motor            [dim](mặc định: 1200)[/dim]
  [yellow]mto <val>[/yellow]     Max góc velocity loop (°)   [dim](mặc định: 3.0)[/dim]

[bold yellow]AN TOÀN / KIẾN TRÚC — trước đây phải nạp lại firmware[/bold yellow]
  [yellow]arch 0|1[/yellow]    Kiểu output: 0=tốc độ trực tiếp, 1=tích phân (B-ROBOT)
  [dim]→ ĐỔI arch là phải đổi gain luôn: arch 0 → kp 80 kd 12 | arch 1 → kp 8 kd 125[/dim]
  [yellow]dlpf <0-6>[/yellow]  Lọc phần cứng MPU  [dim](4=21Hz, 5=10Hz mặc định, 6=5Hz. Lọc mạnh = trễ nhiều)[/dim]
  [yellow]fall <độ>[/yellow]   Ngưỡng coi là ngã      [dim](45)[/dim]
  [yellow]rec[/yellow] / [yellow]recg[/yellow]  Ngưỡng phục hồi góc / gyro [dim](10 / 100)[/dim]
  [yellow]vdir -1|1[/yellow]   Đảo chiều vòng velocity nếu nó đẩy sai hướng
  [yellow]resync[/yellow] [yellow]resynck[/yellow]  Ngưỡng & độ mạnh kéo CF về accel [dim](3.0 / 0.04)[/dim]
  [yellow]awin <độ>[/yellow]   Cửa sổ anti-windup vòng velocity [dim](5)[/dim]
  [yellow]soft <n>[/yellow]    Số vòng ramp Kp sau khi phục hồi [dim](300 = 3 giây)[/dim]
  [yellow]cal[/yellow]         Calibrate lại (chỉ khi đang ngã) — khỏi phải power cycle

[bold yellow]CHỌN CỘT TELEMETRY[/bold yellow]
  [yellow]log[/yellow]           Liệt kê các cột, [x] = đang bật
  [yellow]log <tên>[/yellow]     Bật/tắt một cột  [dim](vd: log ctrlout, log ivel, log dt)[/dim]
  [yellow]logall[/yellow] / [yellow]lognone[/yellow]  Bật hết / tắt hết
  [yellow]logdiv <n>[/yellow]    Chia tần số: 1=100Hz, 2=50Hz, 4=25Hz
  [dim]→ Cột tắt sẵn, bật khi cần soi: pidout ctrlout ivel iang velf velerr dt[/dim]
  [dim]→ Băng thông 115200 ≈ 11.5 KB/s. Quá 10 cột @100Hz thì đặt [/dim][yellow]logdiv 2[/yellow]
  [dim]→ Arduino tự gửi "#COLS ..." mỗi lần đổi, PC tự đổi header file .tsv theo[/dim]

[bold yellow]CƠ CHẾ[/bold yellow]
  [dim]Mọi tham số đều theo dạng [/dim][yellow]<tên> <giá trị>[/yellow][dim]. Gõ [/dim][yellow]<tên>[/yellow][dim] không kèm số
  → đọc lại giá trị hiện tại. Gõ [/dim][yellow]p[/yellow][dim] → dump cả bảng kèm khoảng hợp lệ.
  Python không giữ danh sách tham số: thêm 1 dòng vào PARAMS[] trong
  firmware là dùng được ngay, file này không phải sửa.[/dim]

[bold yellow]FILE CẤU HÌNH[/bold yellow]
  [yellow]cfg[/yellow]           Liệt kê các file trong [dim]configs/[/dim]
  [yellow]cfg <tên>[/yellow]     Nạp [dim]configs/<tên>.txt[/dim] — gửi từng lệnh theo thứ tự trong file
  [dim]→ [/dim][yellow]balance_good[/yellow][dim] là bộ đã đo: angle σ 0.272°, 99% thời gian trong ±1°[/dim]
  [dim]→ Bộ này cũng TỰ GỬI mỗi khi Arduino báo "Bat dau!"[/dim]
  [dim]→ Sửa tham số thì sửa file .txt, đừng sửa code[/dim]

[bold yellow]LƯU / PHỤC HỒI[/bold yellow]
  [yellow]save[/yellow]          Lưu tất cả params vào EEPROM (nhớ qua power cycle)
  [yellow]load[/yellow]          Load lại từ EEPROM
  [yellow]reset[/yellow]         Reset về giá trị mặc định (chưa save)

  [yellow]help[/yellow]  Hiện lại menu này     [yellow]quit[/yellow]  Thoát
""")


# ─────────────────────────────────────────────────────────────────
# COMMAND PROCESSOR
# ─────────────────────────────────────────────────────────────────
SINGLE_CMDS = {"f": "F\n", "b": "B\n", "l": "L\n", "r": "R\n", "s": "S\n", "p": "P\n"}

# Lệnh hành động — Arduino xử lý riêng, KHÔNG nằm trong bảng tham số
ACTION_CMDS = {"save", "load", "reset", "cal", "calup", "get", "biasadj"}

# ─────────────────────────────────────────────────────────────────
# LÁI ROBOT
# ─────────────────────────────────────────────────────────────────
# Khoảng cách giữa hai bánh, mm. Ước từ ảnh — ĐO LẠI rồi sửa số này thì
# lệnh 'spin' mới chính xác. Dùng để quy đổi chênh lệch bước sang góc quay.
# ĐO 25/10/2026: TURN 200 trong 3s cho lệch bánh 220.9mm, quan sát robot quay
# đúng 1/4 vòng → track = 220.9 / (pi/2) = 140.6mm. Trước đây để 120mm, chỉ là
# ước lượng từ ảnh, nên 'spin <deg>' quay thiếu ~17%.
WHEEL_TRACK_MM = 140.6
MM_PER_STEP    = 0.177      # 90mm × π / 1600 bước (1/8 vi bước)

DRIVE_LEVEL = 1.0           # độ nghiêng mặc định cho fwd/back
TURN_LEVEL  = 150           # step/s mặc định cho left/right
MOVE_TIMEOUT = 20.0         # giây, chặn trên cho move/spin


def _send_motion(robot, drive=None, turn=None):
    if drive is not None:
        robot.send(f"DRIVE {drive:.2f}\n"); time.sleep(0.08)
    if turn is not None:
        robot.send(f"TURN {int(turn)}\n");  time.sleep(0.08)


def _stop(robot):
    robot.send("S\n")


def _timed(robot, drive, turn, secs, label):
    """Lái trong N giây rồi dừng. Dừng sớm nếu robot ngã."""
    safe_print(f"[dim yellow]>> {label} trong {secs:.1f}s[/dim yellow]")
    _send_motion(robot, drive, turn)
    t0 = time.time()
    while time.time() - t0 < secs:
        time.sleep(0.05)
        with robot._lock:
            if robot.telem.state == "FALLEN":
                _stop(robot)
                console.print("[red]robot ngã — dừng[/red]")
                return
    _stop(robot)
    safe_print("[dim yellow]>> dừng[/dim yellow]")


def _move_until(robot, drive, turn, target, metric, label):
    """Lái cho tới khi đạt quãng đường/góc mục tiêu, đo bằng telemetry.

    metric('avg')  -> (stepsL+stepsR)/2, quãng đường tịnh tiến
    metric('diff') -> stepsR-stepsL,     lượng quay
    Đây là vòng kín dùng chính bộ đếm bước, nên chính xác hơn canh giờ nhiều.
    """
    with robot._lock:
        t = robot.telem
        base = (t.steps_l + t.steps_r) / 2 if metric == "avg" else (t.steps_r - t.steps_l)
    _send_motion(robot, drive, turn)
    t0 = time.time()
    while time.time() - t0 < MOVE_TIMEOUT:
        time.sleep(0.03)
        with robot._lock:
            t = robot.telem
            if t.state == "FALLEN":
                _stop(robot); console.print("[red]robot ngã — dừng[/red]"); return
            cur = (t.steps_l + t.steps_r) / 2 if metric == "avg" else (t.steps_r - t.steps_l)
        if abs(cur - base) >= abs(target):
            _stop(robot)
            safe_print(f"[dim yellow]>> {label}: xong ({abs(cur-base):.0f} bước)[/dim yellow]")
            return
    _stop(robot)
    console.print(f"[yellow]{label}: hết {MOVE_TIMEOUT:.0f}s mà chưa tới — dừng[/yellow]")

# Live tuning: bias +0.1 / bias -0.2 / kp 85 / kd 14
# Tuning CHUNG: "<tên> <giá trị>" hoặc "<tên>" (đọc lại giá trị hiện tại).
# Python KHÔNG giữ danh sách tham số — cứ viết hoa rồi gửi thẳng. Arduino tra
# bảng PARAMS[] của nó; tên lạ thì nó trả về "? TÊN". Thêm tham số mới vào
# firmware là dùng được ngay, không phải sửa file này.
_TUNE_RE  = re.compile(r"^([a-z][a-z0-9_]{0,8})(?:\s+([-+]?\d*\.?\d+))?$")

# Dòng Arduino in ra khi dump tham số: "KP=80.0000 [0.00..500.00]"
_PARAM_LINE_RE = re.compile(r"^([A-Z][A-Z0-9_]*)=(-?[\d.]+)")
_PANEL_FIELDS  = {"KP": "kp_angle", "KD": "kd_angle",
                  "KPV": "kp_vel",  "KIV": "ki_vel"}

def process_command(robot: RobotBT, user_input: str) -> bool:
    cmd = user_input.strip()
    if not cmd:
        return True
    lower = cmd.lower()

    if lower in ("quit", "exit", "q"):
        return False
    if lower in ("help", "?", "h"):
        print_help()
        return True
    if lower in ("t", "telem", "telemetry", "status"):
        with robot._lock:
            console.print(render_telemetry(robot.telem))
        return True
    if lower.startswith("raw "):
        payload = cmd[4:]
        robot.send(payload)
        console.print(f"[dim]→ raw: {repr(payload)}[/dim]")
        return True

    if lower in ("save",):
        robot.send("SAVE\n")
        safe_print(f"[dim yellow]>> SAVE (robot có thể ngã — block ~150ms)[/dim yellow]")
        return True
    if lower in ("load",):
        robot.send("LOAD\n")
        safe_print(f"[dim yellow]>> LOAD[/dim yellow]")
        return True
    if lower in ("reset",):
        robot.send("RESET\n")
        safe_print(f"[dim yellow]>> RESET[/dim yellow]")
        return True
    if lower in ("cal", "calib"):
        robot.send("CAL\n")
        safe_print(f"[dim yellow]>> CAL (chỉ chạy khi robot đang ngã — block 4s)[/dim yellow]")
        return True
    if lower in ("get", "params"):
        robot.send("GET\n")
        return True
    if lower in SINGLE_CMDS:
        robot.send(SINGLE_CMDS[lower])
        return True

    m = re.fullmatch(r"([fblrsp])(\d+)", lower)
    if m:
        key, n = m.group(1), min(int(m.group(2)), 20)
        bt_cmd = SINGLE_CMDS[key]
        for _ in range(n):
            robot.send(bt_cmd)
            time.sleep(0.05)
        console.print(f"[dim]→ '{bt_cmd}' × {n}[/dim]")
        return True

    # ── LÁI ROBOT ──────────────────────────────────────────────────
    # go <drive> <turn> [giây]  — đặt thẳng cả hai, kèm tự dừng
    m = re.fullmatch(r"go\s+([-+]?[\d.]+)\s+([-+]?\d+)(?:\s+([\d.]+))?", lower)
    if m:
        d, t_, secs = float(m.group(1)), int(m.group(2)), m.group(3)
        if secs:
            _timed(robot, d, t_, float(secs), f"go {d} {t_}")
        else:
            _send_motion(robot, d, t_)
            safe_print(f"[dim yellow]>> drive={d} turn={t_} (gõ 's' để dừng)[/dim yellow]")
        return True

    # fwd/back/left/right [giây] — mặc định 1 giây
    m = re.fullmatch(r"(fwd|back|left|right)(?:\s+([\d.]+))?", lower)
    if m:
        what, secs = m.group(1), float(m.group(2) or 1.0)
        d, t_ = {"fwd":   ( DRIVE_LEVEL, 0),
                 "back":  (-DRIVE_LEVEL, 0),
                 "left":  (0, -TURN_LEVEL),
                 "right": (0,  TURN_LEVEL)}[what]
        _timed(robot, d, t_, secs, what)
        return True

    # move <cm> — vòng kín, đếm bước tới khi đủ quãng đường
    m = re.fullmatch(r"move\s+([-+]?[\d.]+)", lower)
    if m:
        cm = float(m.group(1))
        steps = cm * 10.0 / MM_PER_STEP
        _move_until(robot, DRIVE_LEVEL if cm > 0 else -DRIVE_LEVEL, 0,
                    steps, "avg", f"move {cm}cm")
        return True

    # spin <độ> — vòng kín, đếm chênh lệch bước hai bánh
    m = re.fullmatch(r"spin\s+([-+]?[\d.]+)", lower)
    if m:
        deg = float(m.group(1))
        # cung mỗi bánh = góc(rad) × track/2; chênh lệch = 2 lần cung đó
        arc_mm = abs(deg) * 3.14159 / 180.0 * WHEEL_TRACK_MM
        steps  = arc_mm / MM_PER_STEP
        _move_until(robot, 0, TURN_LEVEL if deg > 0 else -TURN_LEVEL,
                    steps, "diff", f"spin {deg}do")
        return True

    # seq a; b; c — chạy lần lượt
    if lower.startswith("seq "):
        for part in cmd[4:].split(";"):
            part = part.strip()
            if not part:
                continue
            console.print(f"[dim]→ {part}[/dim]")
            if not process_command(robot, part):
                return False
        return True

    # "cfg" liệt kê các file cấu hình, "cfg <tên>" nạp một file
    if lower == "cfg" or lower.startswith("cfg "):
        name = cmd[4:].strip()
        if not name:
            try:
                names = sorted(f[:-4] for f in os.listdir(CONFIG_DIR) if f.endswith(".txt"))
            except OSError:
                names = []
            console.print(f"[cyan]configs/: {', '.join(names) or '(trống)'}[/cyan]")
            return True
        try:
            cmds = load_config(name)
        except OSError as e:
            console.print(f"[red]Không đọc được '{name}': {e}[/red]")
            return True
        safe_print(f"[dim yellow]>> nạp configs/{name}.txt — {len(cmds)} lệnh[/dim yellow]")
        for c in cmds:
            robot.send(c)
            time.sleep(0.12)      # Arduino cần thời gian xử lý từng dòng
        safe_print(f"[dim yellow]>> xong. Gõ 'p' để kiểm tra.[/dim yellow]")
        return True

    # "log gyro" — bật/tắt 1 cột telemetry. Phải xử lý riêng vì _TUNE_RE
    # chỉ nhận số làm đối số. ("log", "logall", "lognone", "cols" đi đường chung.)
    m = re.fullmatch(r"log\s+([a-z][a-z0-9_]*)", lower)
    if m:
        payload = f"LOG {m.group(1).upper()}\n"
        robot.send(payload)
        safe_print(f"[dim yellow]>> {payload.strip()}[/dim yellow]")
        return True

    # ── Tuning CHUNG — luôn đặt CUỐI, sau mọi lệnh hành động ở trên ──
    # Không có bảng tham số ở phía Python: viết hoa rồi gửi thẳng.
    # Arduino tra bảng PARAMS[] của nó và trả "? TÊN" nếu không biết.
    m = _TUNE_RE.match(lower)
    if m:
        key, val = m.group(1), m.group(2)
        body    = f"{key.upper()} {val}" if val is not None else key.upper()
        payload = body + "\n"
        robot.send(payload)
        safe_print(f"[dim yellow]>> {payload.strip()}[/dim yellow]")
        return True

    console.print(f"[red]Lệnh không rõ: '{cmd}'. Gõ 'help' để xem hướng dẫn.[/red]")
    return True




# ─────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────
def _enable_windows_ansi():
    """Bật ANSI VT processing cho cmd.exe / PowerShell cũ trên Windows 10+."""
    try:
        k32 = ctypes.windll.kernel32
        handle = k32.GetStdHandle(-11)          # STD_OUTPUT_HANDLE
        mode   = ctypes.c_ulong()
        k32.GetConsoleMode(handle, ctypes.byref(mode))
        k32.SetConsoleMode(handle, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    if sys.platform == "win32":
        _enable_windows_ansi()

    console.print("[bold cyan]╔══════════════════════════════════╗[/bold cyan]")
    console.print("[bold cyan]║  Robot Balance BT Controller     ║[/bold cyan]")
    console.print("[bold cyan]╚══════════════════════════════════╝[/bold cyan]\n")

    mac_arg = sys.argv[1] if len(sys.argv) >= 2 else None
    mac = choose_device(mac_arg)
    if not mac:
        console.print("[red]Thoát.[/red]")
        sys.exit(1)

    robot = RobotBT(mac)
    try:
        robot.connect()
    except OSError as e:
        console.print(f"[red]Không thể kết nối tới {mac}: {e}[/red]")
        console.print("Kiểm tra: HC-05 đã ghép nối (paired) chưa? Đang bật nguồn?")
        sys.exit(1)

    log_path, raw_path = open_log_file()
    console.print(f"[dim]Transcript : [white]{log_path}[/white][/dim]")
    console.print(f"[dim]Telemetry  : [white]{raw_path}[/white]  (100Hz, mọi mẫu)[/dim]")
    console.print("[dim]Startup params sẽ tự gửi sau khi Arduino báo 'Bat dau!'[/dim]")

    print_help()
    console.print("[dim]Log từ robot xuất hiện tự động. Gõ lệnh bên dưới:[/dim]\n")

    def _send_startup():
        if not STARTUP_PARAMS:
            return
        time.sleep(0.3)
        safe_print("[dim yellow]>> Gửi startup params...[/dim yellow]")
        for cmd in STARTUP_PARAMS:
            robot.send(cmd)
            safe_print(f"[dim yellow]>> {cmd.strip()}[/dim yellow]")
            time.sleep(0.12)

    session = PromptSession(style=Style.from_dict({"prompt": "bold ansicyan"}))
    _ctrl_c = 0
    while robot.connected:
        # Gửi startup params mỗi khi Arduino reboot ("Bat dau!" nhận được)
        if robot._pending_startup:
            robot._pending_startup = False
            threading.Thread(target=_send_startup, daemon=True).start()

        try:
            user_input = session.prompt(
                HTML("<ansigreen><b>robot</b></ansigreen><ansicyan>></ansicyan> "),
                timeout=1.0,
            )
            _ctrl_c = 0
        except KeyboardInterrupt:
            _ctrl_c += 1
            if _ctrl_c >= 2:
                console.print("\n[bold red]Ctrl+C × 2 — thoát.[/bold red]")
                break
            console.print("\n[yellow]Ctrl+C — bấm lần nữa để thoát, hoặc gõ 'quit'.[/yellow]")
            continue
        except EOFError:
            break
        except Exception:
            # timeout — không reset counter, chỉ kiểm tra pending_startup rồi prompt lại
            continue
        if not process_command(robot, user_input):
            break

    robot.disconnect()
    if _log_fh:
        _log_fh.close()
        console.print(f"[dim]Transcript đã lưu: [white]{log_path}[/white][/dim]")
    if _raw_fh:
        _raw_fh.flush()
        _raw_fh.close()
        n = robot.telem.sample_count
        console.print(f"[dim]Telemetry đã lưu : [white]{raw_path}[/white]  ({n} mẫu)[/dim]")
    console.print("[bold]Tạm biệt![/bold]")


if __name__ == "__main__":
    main()
