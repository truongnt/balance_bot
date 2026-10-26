#!/usr/bin/env python3
"""
Giữ kết nối với robot, TỰ NẠP LẠI cấu hình mỗi khi Arduino khởi động lại,
và ghi telemetry liên tục. Không cần gõ lệnh.

    python robot_auto.py                      # chạy mãi, Ctrl+C để dừng
    python robot_auto.py 300                  # chạy 300 giây rồi thoát
    python robot_auto.py 300 balance_good     # chỉ định file cấu hình

Vì sao cần: mỗi lần Arduino reboot (mất nguồn, chạm chân, nạp code) là toàn bộ
tham số về mặc định firmware. Script này thấy dòng "Bat dau!" là tự gửi lại
configs/<tên>.txt, nên robot luôn chạy đúng bộ tham số mà không phải nhớ gì.

Nó cũng cảnh báo khi calibration hỏng: upright_offset bình thường nằm trong
khoảng 0–15°. Vượt quá nghĩa là robot bị calibrate lúc đang nằm, và nó sẽ coi
tư thế nằm là "thẳng đứng" — phải dựng thẳng rồi chạy lại 'cal'.
"""
import os
import socket
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_control import CONFIG_DIR, load_config  # noqa: E402

MAC      = "00:21:06:08:1C:D4"
LOG_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
SECS     = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")
CFG_NAME = sys.argv[2] if len(sys.argv) > 2 else "balance_good"

UPRIGHT_SANE_MAX = 15.0     # độ; lớn hơn = calibrate lúc robot đang nằm


def now() -> str:
    return time.strftime("%H:%M:%S")


def connect(tries: int = 8):
    for a in range(1, tries + 1):
        try:
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            s.settimeout(10)
            s.connect((MAC, 1))
            s.settimeout(0.3)
            return s
        except Exception as e:
            print(f"[{now()}] ket noi thu {a}: {type(e).__name__}", flush=True)
            time.sleep(2)
    return None


class Session:
    """Một phiên ghi: header theo #COLS, đổi cột giữa chừng thì chèn mốc."""

    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8", buffering=1)
        self.cols = ["ms", "angle", "accel", "target", "gyro", "speed", "stepsL", "stepsR"]
        self.header = False
        self.n = 0

    def set_cols(self, cols):
        if cols == self.cols:
            return
        self.cols = cols
        print(f"[{now()}] #COLS -> {len(cols)}: {' '.join(cols)}", flush=True)
        if self.header:
            self.f.write("#COLS " + " ".join(cols) + "\n")

    def row(self, ln, parts):
        if not self.header:
            self.f.write("\t".join(self.cols) + "\n")
            self.header = True
        self.f.write(ln + "\n")
        self.n += 1

    def close(self):
        self.f.close()


def main() -> int:
    try:
        cfg = load_config(CFG_NAME)
    except OSError as e:
        print(f"Khong doc duoc configs/{CFG_NAME}.txt: {e}")
        return 1
    print(f"cau hinh: configs/{CFG_NAME}.txt ({len(cfg)} lenh)")

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"robot_{datetime.now():%Y%m%d_%H%M%S}_auto_raw.tsv")
    ses = Session(path)
    print(f"log: {path}\n")

    t_end   = time.time() + SECS
    reloads = 0
    drops   = 0
    pending = False          # thấy "Bat dau!" -> nạp lại ở vòng kế
    last_report = time.time()

    try:
        while time.time() < t_end:
            s = connect()
            if s is None:
                print("khong noi lai duoc, dung lai")
                break
            print(f"[{now()}] da noi (rot {drops} lan, nap lai {reloads} lan)", flush=True)
            # Nạp ngay khi vừa nối: không biết robot đã reboot lúc mình offline hay chưa
            pending = True
            buf = ""
            try:
                while time.time() < t_end:
                    if pending:
                        pending = False
                        for c in cfg:
                            s.sendall(c.encode())
                            time.sleep(0.12)
                        reloads += 1
                        print(f"[{now()}] >> da nap lai cau hinh ({reloads})", flush=True)

                    try:
                        d = s.recv(1024)
                        if not d:
                            raise OSError("peer closed")
                        buf += d.decode("utf-8", errors="replace")
                    except socket.timeout:
                        continue

                    while "\n" in buf:
                        ln, buf = buf.split("\n", 1)
                        ln = ln.strip("\r ")
                        if not ln:
                            continue
                        if ln.startswith("#COLS"):
                            ses.set_cols(ln.split()[1:])
                            continue
                        parts = ln.split("\t")
                        if len(parts) == len(ses.cols) and len(parts) >= 4:
                            try:
                                [float(x) for x in parts]
                            except ValueError:
                                pass
                            else:
                                ses.row(ln, parts)
                                continue
                        # dòng chữ
                        if "Bat dau" in ln:
                            print(f"[{now()}] !! ARDUINO KHOI DONG LAI", flush=True)
                            pending = True
                        elif "upright_offset" in ln:
                            try:
                                v = float(ln.split("=")[1])
                            except (IndexError, ValueError):
                                v = 0.0
                            bad = abs(v) > UPRIGHT_SANE_MAX
                            print(f"[{now()}] {ln}" + ("   <<< HONG: calibrate luc dang NAM."
                                                       " Dung thang roi chay 'cal'" if bad else ""),
                                  flush=True)
                        elif any(k in ln for k in ("FALLEN", "RECOVERED", "gyroBias", "? ")):
                            print(f"[{now()}] {ln[:70]}", flush=True)

                    if time.time() - last_report > 30:
                        last_report = time.time()
                        print(f"[{now()}] {ses.n} mau", flush=True)
            except OSError as e:
                drops += 1
                print(f"[{now()}] rot link ({type(e).__name__})", flush=True)
            finally:
                try:
                    s.close()
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\ndung theo yeu cau")
    finally:
        ses.close()

    print(f"\n{ses.n} mau -> {path}")
    print(f"nap lai cau hinh {reloads} lan | rot link {drops} lan")
    return 0


if __name__ == "__main__":
    sys.exit(main())
