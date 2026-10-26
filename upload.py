#!/usr/bin/env python3
"""
Nạp firmware một cách xác định, thay cho `arduino-cli upload -p COM?` trần.

    python upload.py              # tự dò board rồi nạp
    python upload.py --check      # chỉ kiểm tra cổng, không nạp

Vì sao cần file này thay vì gõ thẳng arduino-cli:

Máy này có BẢY cổng COM, sáu trong số đó là cổng ảo Bluetooth, và một cổng ảo
Bluetooth "incoming" đang mang **đúng cùng tên COM7** với board:

    COM7  USB Serial Device          USB VID:PID=0843:5740 SER=FX2348N   <- board
    COM7  Standard Serial over BT    BTHENUM\\...LOCALMFG&0000...         <- rác

Mở tên "COM7" là hên xui trúng cái nào. Trúng cổng Bluetooth thì open() CHẶN VÔ
HẠN — không lỗi, không timeout, avrdude đứng im mãi. Ngày 26/10 mất nhiều lần
thử vì chuyện này, trong khi sáng cùng ngày nó lại nạp được, thuần may mắn.

Nên ở đây:
  * dò board theo VID:PID, không bao giờ tin một số COM nhớ sẵn;
  * cảnh báo khi tên cổng bị trùng, vì đó là nguyên nhân gốc;
  * mỗi lần nạp có HẠN GIỜ CỨNG rồi giết tiến trình, nên treo không thành đứng im;
  * thử lại vài lần, vì việc phân giải tên trùng là ngẫu nhiên.

Cách sửa triệt để (người dùng làm, không cần quyền admin): Settings → Bluetooth
& devices → More Bluetooth settings → tab COM Ports → xoá cổng COM7 loại
"Incoming". Đừng xoá COM4, đó là cổng trỏ tới HC-05 của robot.
"""
import argparse
import os
import subprocess
import sys
import time

import serial.tools.list_ports as list_ports

CLI = r"C:\Program Files\Arduino CLI\arduino-cli.exe"
SKETCH = os.path.dirname(os.path.abspath(__file__))
FQBN = "arduino:avr:uno"
BOARD_VIDPID = "0843:5740"        # clone dùng USB CDC, KHÔNG phải CH340
TIMEOUT = 60                      # giây cho mỗi lần nạp


def find_board():
    """Trả về (cổng, có_bị_trùng_tên, danh_sách_tất_cả)."""
    ports = list(list_ports.comports())
    board = [p for p in ports if BOARD_VIDPID in (p.hwid or "").upper().replace("VID:PID=", "")
             or BOARD_VIDPID.upper() in (p.hwid or "").upper()]
    if not board:
        return None, False, ports
    dev = board[0].device
    dup = sum(1 for p in ports if p.device == dev) > 1
    return dev, dup, ports


def show(ports, board_dev):
    print(f"{len(ports)} cổng COM:")
    for p in sorted(ports, key=lambda x: (len(x.device), x.device)):
        mark = "  <-- BOARD" if (p.device == board_dev and "VID" in (p.hwid or "").upper()
                                 and BOARD_VIDPID.upper() in (p.hwid or "").upper()) else ""
        print(f"   {p.device:7} {str(p.description)[:40]:42}{mark}")


def upload(port, attempt):
    """Một lần nạp, có hạn giờ cứng. Treo thì giết, không chờ mãi."""
    print(f"\n--- lần {attempt}: nạp vào {port} (hạn {TIMEOUT}s) ---", flush=True)
    p = subprocess.Popen([CLI, "upload", "-p", port, "--fqbn", FQBN, SKETCH],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        out, _ = p.communicate(timeout=TIMEOUT)
        for ln in (out or "").strip().splitlines()[-4:]:
            print("   |", ln[:100])
        return p.returncode == 0
    except subprocess.TimeoutExpired:
        p.kill()
        try:
            p.communicate(timeout=5)
        except Exception:
            pass
        subprocess.run(["taskkill", "/F", "/IM", "avrdude.exe"],
                       capture_output=True)
        print("   TREO -> đã giết. Gần như chắc chắn đã mở trúng cổng Bluetooth trùng tên.")
        return False


def main():
    ap = argparse.ArgumentParser(description="Nạp firmware, tự dò cổng theo VID:PID")
    ap.add_argument("--check", action="store_true", help="chỉ liệt kê, không nạp")
    ap.add_argument("--tries", type=int, default=4)
    a = ap.parse_args()

    port, dup, ports = find_board()
    show(ports, port)
    if not port:
        print(f"\nKHÔNG THẤY BOARD (VID:PID {BOARD_VIDPID}). Cắm USB chưa?")
        return 1
    print(f"\nboard ở {port}")
    if dup:
        print(f"[!] {port} trùng tên với một cổng Bluetooth.")
        print("    Nếu cổng Bluetooth đó đang BẬT thì avrdude sẽ treo cứng khi mở.")
        print("    Nếu đã disable rồi thì không sao — danh sách này đọc từ registry")
        print("    nên vẫn hiện cả mục đã tắt.")
        print("    Cách đã gỡ được (26/10/2026, không cần quyền admin):")
        print("      1. Device Manager → Ports → chuột phải cổng Bluetooth trùng số")
        print("         → Disable device   (ĐỪNG đụng cổng trỏ tới HC-05 của robot)")
        print("      2. RÚT cáp USB, đợi 2 giây, CẮM LẠI")
        print("    Chỉ disable mà không cắm lại thì vẫn lỗi 'unable to open port':")
        print("    thiết bị đã tắt vẫn giữ liên kết tên cho tới khi board liệt kê lại.")
    if a.check:
        return 0

    for i in range(1, a.tries + 1):
        if upload(port, i):
            print(f"\nNẠP XONG sau {i} lần.")
            return 0
        if i < a.tries:
            time.sleep(2)
    print(f"\nThất bại sau {a.tries} lần. Xoá cổng Bluetooth trùng tên rồi chạy lại.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
