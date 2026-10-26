#!/usr/bin/env python3
"""
Phân tích file telemetry thô (logs/*_raw.tsv) do robot_control.py ghi.

    python analyze_log.py logs\robot_20261021_162444_raw.tsv
    python analyze_log.py                    # tự lấy file _raw.tsv mới nhất

In ra: thống kê từng cột, phổ tần số của angle/accel/gyro, và drift CF-vs-accel.
Phổ tần số là thứ để trả lời "robot rung ở tần số nào" — câu mà log tóm tắt
0.5 giây/mẫu không bao giờ trả lời được (aliasing).
"""
import sys, os, glob
import numpy as np

LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def load(path):
    """Đọc .tsv. Bộ cột có thể đổi giữa phiên qua dòng mốc '#COLS ...'.

    Mỗi lần đổi là một khối riêng với header riêng. Trả về khối DÀI NHẤT —
    đó gần như luôn là phần đáng phân tích, còn khối đầu thường chỉ là vài
    trăm mẫu trước khi cấu hình kịp áp dụng.
    """
    with open(path, encoding="utf-8") as f:
        lines = [l.rstrip("\n") for l in f if l.strip()]

    blocks = []                       # [(hdr, rows), ...]
    hdr, rows = lines[0].split("\t"), []
    for l in lines[1:]:
        if l.startswith("#COLS"):
            if rows:
                blocks.append((hdr, rows))
            hdr, rows = l.split()[1:], []
            continue
        if l.startswith("#"):
            continue
        p = l.split("\t")
        if len(p) != len(hdr):
            continue
        try:
            rows.append([float(x) for x in p])
        except ValueError:
            continue
    if rows:
        blocks.append((hdr, rows))
    if not blocks:
        return [], np.empty((0, 0))

    if len(blocks) > 1:
        print("  [!] bộ cột đổi giữa phiên — các khối: "
              + ", ".join(f"{len(r)} mẫu/{len(h)} cột" for h, r in blocks))
    hdr, rows = max(blocks, key=lambda b: len(b[1]))
    return hdr, np.array(rows, dtype=float)


def spectrum(x, fs, hp=0.5):
    """Phổ biên độ một chiều, bỏ thành phần dưới hp Hz (DC + trôi chậm)."""
    x = x - x.mean()
    if x.std() < 1e-9:
        return None, None
    sp = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), 1.0 / fs)
    sp[fr < hp] = 0
    return fr, sp / len(x) * 4.0      # chuẩn hoá về biên độ xấp xỉ


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        files = sorted(glob.glob(os.path.join(LOGDIR, "*_raw.tsv")))
        if not files:
            print("Không có file *_raw.tsv nào trong logs/"); return 1
        path = files[-1]

    hdr, D = load(path)
    if D.size == 0:
        print("File rỗng hoặc sai định dạng."); return 1

    T, A, cols = D[:, 0], D[:, 1:], hdr[1:]

    # Cột thời gian nhảy LÙI = Arduino đã reset giữa chừng. Phải cắt thành đoạn,
    # nếu không thì mọi tính toán tần số đều vô nghĩa (từng ra "-5.4 giây, 0 Hz").
    breaks = np.where(np.diff(T) < 0)[0]
    print(f"\n=== {os.path.basename(path)}")
    if len(breaks):
        groups = np.split(np.arange(len(T)), breaks + 1)
        print(f"[!] {len(breaks)} lần ARDUINO RESET giữa phiên — cắt thành {len(groups)} đoạn:")
        for i, g in enumerate(groups):
            if len(g) < 2:
                continue
            print(f"    đoạn {i}: {len(g):5d} mẫu, {(T[g[-1]]-T[g[0]])/1000:5.1f}s")
        g = max(groups, key=len)
        print(f"    → phân tích đoạn dài nhất ({len(g)} mẫu)\n")
        T, A = T[g], A[g]

    dur = (T[-1] - T[0]) / 1000.0
    fs  = len(T) / dur if dur > 0 else 0.0
    if fs <= 0:
        print("Không đủ dữ liệu liên tục để phân tích."); return 1
    print(f"{len(T)} mẫu · {dur:.1f} giây · {fs:.1f} Hz\n")

    # jitter chu kỳ — nếu lớn thì vòng điều khiển đang bị chặn ở đâu đó
    dt = np.diff(T)
    print(f"chu kỳ mẫu: trung bình {dt.mean():.2f} ms, "
          f"độ lệch {dt.std():.2f} ms, max {dt.max():.1f} ms")
    if dt.max() > 30:
        print(f"  [!] có khoảng trống {dt.max():.0f} ms — mất gói BT hoặc loop bị chặn")

    print(f"\n{'cột':9}{'t.bình':>11}{'độ lệch':>11}{'min':>10}{'max':>10}{'biên độ':>10}")
    for i, c in enumerate(cols):
        x = A[:, i]
        print(f"{c:9}{x.mean():11.3f}{x.std():11.4f}{x.min():10.2f}{x.max():10.2f}{np.ptp(x):10.2f}")

    print(f"\n=== PHỔ TẦN SỐ (Nyquist {fs/2:.0f} Hz) ===")
    for name in ("angle", "accel", "gyro", "speed", "ctrlout"):
        if name not in cols:
            continue
        fr, sp = spectrum(A[:, cols.index(name)], fs)
        if fr is None:
            print(f"{name:8}: phẳng (không đổi)"); continue
        top = np.argsort(sp)[-3:][::-1]
        print(f"{name:8}: " + ",  ".join(f"{fr[k]:5.1f} Hz (bđ {sp[k]:.4f})" for k in top))

    if "angle" in cols and "accel" in cols:
        d = A[:, cols.index("angle")] - A[:, cols.index("accel")]
        print(f"\ndrift (angle − accel): trung bình {d.mean():+.3f}°  "
              f"độ lệch {d.std():.3f}°  max |{np.abs(d).max():.2f}|°")
        if np.abs(d).max() > 3:
            print("  [!] vượt ngưỡng resync 3° — complementary filter đang phân kỳ")

    for c in ("stepsL", "stepsR"):
        if c in cols:
            x = A[:, cols.index(c)]
            print(f"{c}: {x[0]:.0f} → {x[-1]:.0f}  (chạy {abs(x[-1]-x[0]):.0f} bước)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
