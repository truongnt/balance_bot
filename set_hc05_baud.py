"""
Đổi baud rate HC-05 sang 115200.
Chạy khi HC-05 đang ở AT mode (đèn nhấp chậm 1s/lần).

Usage:
    python set_hc05_baud.py        # tự tìm cổng COM
    python set_hc05_baud.py COM8   # chỉ định cổng
"""
import sys
import time
import serial
import serial.tools.list_ports

NEW_BAUD  = 115200

def find_port():
    ports = list(serial.tools.list_ports.comports())
    usb = [p for p in ports if "USB" in (p.description or "").upper()
                             or "CH340" in (p.description or "").upper()
                             or "CP210" in (p.description or "").upper()]
    if usb:
        return usb[0].device
    if ports:
        return ports[0].device
    return None

def open_no_reset(port, baud):
    """Mở serial mà không trigger Arduino reset (tắt DTR trước khi open)."""
    s = serial.Serial()
    s.port    = port
    s.baudrate = baud
    s.timeout  = 2
    s.dtr      = False   # giữ DTR thấp — không reset Arduino
    s.rts      = False
    s.open()
    return s

def try_at(s):
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"AT\r\n")
    time.sleep(0.8)
    resp = s.read_all().decode(errors="replace").strip()
    return resp

def run(port):
    for baud in [38400, 9600]:
        print(f"Try {port} @ {baud} baud...", end=" ", flush=True)
        with open_no_reset(port, baud) as s:
            resp = try_at(s)
            if "OK" in resp:
                print("OK!")
                return run_at(port, baud)
            print(f"no response ({repr(resp)})")
    print("\nHC-05 not in AT mode.")
    print("Try: unplug USB -> hold KEY -> plug USB back -> run script again")
    return False

def run_at(port, AT_BAUD):
    with open_no_reset(port, AT_BAUD) as s:
        time.sleep(0.3)
        # Xem baud rate hiện tại
        s.write(b"AT+UART?\r\n")
        time.sleep(0.5)
        resp = s.read_all().decode(errors="replace").strip()
        print(f"Current baud: {resp}")

        cmd = f"AT+UART={NEW_BAUD},0,0\r\n".encode()
        s.write(cmd)
        time.sleep(0.5)
        resp = s.read_all().decode(errors="replace").strip()
        print(f"Set {NEW_BAUD}: {resp}")

        if "OK" in resp:
            print(f"\nDone! Next steps:")
            print(f"  1. Power cycle HC-05 normally (no KEY)")
            print(f"  2. Upload Arduino firmware (Serial.begin(115200))")
            return True
        else:
            print("Failed. Try again.")
            return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("COM port not found. Plug in Arduino USB.")
        sys.exit(1)
    run(port)
