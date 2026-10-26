"""Mo phong dung may trang thai handleBTCommand() de kiem tra truoc khi nap.
Khong thay the test tren phan cung, nhung bat duoc loi logic ma neu khong thi
phai cam USB moi biet."""
BT_IDLE_MS = 60
JOY = set("FBLRSGIHJ0123456789")


class Parser:
    def __init__(self):
        self.buf = ""
        self.last_rx = 0
        self.out = []

    def rx(self, c, t):
        self.last_rx = t
        if c in "\n\r":
            if self.buf:
                self.out.append(self.buf); self.buf = ""
        elif len(self.buf) == 1 and c == self.buf[0] and c in JOY:
            self.out.append(self.buf); self.buf = c
        elif len(self.buf) < 31:
            self.buf += c
        else:
            self.buf = ""

    def tick(self, t):
        if self.buf and (t - self.last_rx) > BT_IDLE_MS:
            self.out.append(self.buf); self.buf = ""


def run(events):
    """events: [(text, gap_ms_giua_cac_ky_tu, gap_sau_cum)]"""
    p = Parser(); t = 0
    for text, gap, after in events:
        for c in text:
            p.rx(c, t); t += gap
            p.tick(t)
        t += after
        p.tick(t)
    t += 500; p.tick(t)
    return p.out


CASES = [
    ("PC gui 'BIAS 1.4' tron goi",      [("BIAS 1.4\n", 0, 0)],              ["BIAS 1.4"]),
    ("PC gui 'SAVE'",                   [("SAVE\n", 0, 0)],                  ["SAVE"]),
    ("PC gui 'LOGMASK 6143'",           [("LOGMASK 6143\n", 0, 0)],          ["LOGMASK 6143"]),
    ("App bam 'F' mot lan",             [("F", 0, 200)],                     ["F"]),
    ("App giu 'F', lap 50ms (nhanh hon nguong)",
                                        [("FFFF", 50, 300)],                 ["F","F","F","F"]),
    ("App giu 'F', lap 100ms (cham hon nguong)",
                                        [("FFFF", 100, 300)],                ["F","F","F","F"]),
    ("App: tien roi stop",              [("F", 0, 200), ("S", 0, 200)],      ["F","S"]),
    ("App nut cheo 'G' lap",            [("GG", 40, 200)],                   ["G","G"]),
    ("App thanh truot '5' lap",         [("55", 40, 200)],                   ["5","5"]),
    ("PC gui 'BIAS' KHONG newline",     [("BIAS 1.4", 0, 200)],              ["BIAS 1.4"]),
    ("PC gui lien tiep 2 lenh",         [("KP 45\nKD 2.5\n", 0, 0)],         ["KP 45","KD 2.5"]),
    ("App 'R' roi PC 'RESET'",          [("R", 0, 200), ("RESET\n", 0, 0)],  ["R","RESET"]),
]

bad = 0
for name, ev, want in CASES:
    got = run(ev)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'LOI'}  {name:46} -> {got}" + ("" if ok else f"   MONG DOI {want}"))
print(f"\n{len(CASES)-bad}/{len(CASES)} dat")
