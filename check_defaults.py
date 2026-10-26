# -*- coding: utf-8 -*-
"""Doi chieu cot 'default' trong bang PARAMS voi gia tri khoi tao cua bien toan cuc.

Vi sao can: mac dinh nam o HAI cho. resetParams() va lenh 'reset' dung cot trong
bang; con khi EEPROM trong (magic sai) thi firmware KHONG goi resetParams(), bien
giu nguyen gia tri khoi tao luc khai bao. Hai cho lech nhau thi 'reset' va 'boot
khong co EEPROM' cho ra hai cau hinh khac nhau — dung cai bay da lam UPOFF=0
thay vi 2.0 sau lan nap 26/10.
"""
import re, io, sys

P = r"d:\Project\robot_balance\robot_balance.ino"
src = io.open(P, encoding="utf-8").read()

tbl = re.search(r"const ParamDef PARAMS\[\] PROGMEM = \{(.*?)\n\};", src, re.S)
if not tbl:
    sys.exit("khong tim thay bang PARAMS")

rows = re.findall(
    r'\{\s*"([A-Z0-9_]+)"\s*,\s*&(\w+)\s*,\s*PT_\w+\s*,\s*[-\d.ef]+\s*,\s*[-\d.ef]+\s*,\s*([-\d.ef]+)\s*\}',
    tbl.group(1))

print(f"{len(rows)} tham so trong bang\n")
print(f"{'TEN':9}{'BIEN':20}{'bang':>12}{'khoi tao':>12}   ")
bad = []
for name, var, dflt in rows:
    m = re.search(r"^\s*(?:volatile\s+)?(?:float|int16_t|int8_t|uint8_t|uint16_t|uint32_t)\s+"
                  + re.escape(var) + r"\s*=\s*(0[xX][0-9a-fA-F]+|[-\d.ef]+)\s*;", src, re.M)
    if not m:
        print(f"{name:9}{var:20}{dflt:>12}{'?':>12}   (khong doc duoc khoi tao)")
        continue
    lit = m.group(1)
    b = float(int(lit, 16)) if lit[:2].lower() == "0x" else float(lit.rstrip("f"))
    a = float(dflt.rstrip("f"))
    flag = "" if abs(a - b) < 1e-9 else "   <<< LECH"
    if flag:
        bad.append((name, var, a, b))
    print(f"{name:9}{var:20}{a:>12g}{b:>12g}{flag}")

print()
if bad:
    print(f"{len(bad)} cho lech:")
    for n, v, a, b in bad:
        print(f"  {n}: bang={a}  khoi tao={b}  -> sua khoi tao thanh {a}")
else:
    print("Khop het.")
