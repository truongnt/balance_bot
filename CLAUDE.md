# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Robot Balance – Project Overview

Self-balancing two-wheeled robot. Arduino firmware (`robot_balance.ino`) + a Windows
Python Bluetooth console (`robot_control.py`) used for live telemetry and live PID tuning.
Architecture follows B-ROBOT EVO2: Timer1 ISR step generation + cascaded PID.

## Hardware

| Component | Detail |
|---|---|
| MCU | Arduino Uno (ATmega328P @ 16MHz) |
| IMU | MPU-6050 (I²C, A4/A5, addr 0x68, DLPF 10Hz, gyro ±250°/s) |
| Motors | 2× NEMA17 stepper, 1/8 microstep (A4988 on CNC Shield) |
| Motor L (X axis) | STEP=D2, DIR=D5 |
| Motor R (Y axis) | STEP=D3, DIR=D6 |
| Enable | D8 (LOW = ON) |
| Bluetooth | HC-05 on hardware Serial (D0/D1), baud **115200** |

Pins are driven via direct PORTD/PORTB bit manipulation (macros at the top of the `.ino`),
not `digitalWrite` — the ISR cannot afford the overhead.

**HC-05 gotcha**: D0/D1 is shared between HC-05 and the ATmega16U2 USB-Serial bridge. When
USB is plugged in, D0 is driven by both — you cannot read HC-05 AT responses via USB
passthrough. Use `hc05_config\hc05_config.ino` (Arduino blindly sends AT command without
reading the response) to change HC-05 baud rate.

**Arduino reset gotcha**: Opening a serial port from Python triggers DTR → Arduino resets.
Always open with `dtr=False` before calling `s.open()` (see `open_no_reset()` in
`set_hc05_baud.py`).

**Motor direction**: motor R's DIR logic is inverted in `setSingleMotor()` because the two
motors are mounted mirrored. Don't "fix" this asymmetry.

## Files

```
robot_balance.ino       — Arduino firmware (main file)
robot_control.py        — PC controller: BT connect, live telemetry, command input
robot_ui.py             — Browser control panel on localhost (stdlib + SSE)
set_hc05_baud.py        — One-shot script: change HC-05 baud rate via AT mode
hc05_config\            — Arduino sketch to blindly send AT+UART=115200
passthrough\            — Minimal Serial passthrough sketch (rarely needed)
test_motor\             — Motor test sketch
test_imu\               — IMU test sketch
configs\                — Tuned parameter sets, one command per line (see below)
logs\                   — Session logs written by robot_control.py
analyze_log.py          — Stats + amplitude spectrum for logs\*_raw.tsv
test_bt_parser.py       — Host-side simulation of the BT command state machine
check_defaults.py       — Checks PARAMS table defaults against variable initialisers
_backup_v*/             — Firmware snapshots, saved as .ino.txt so arduino-cli ignores them
HUONG_DAN.md            — Vietnamese wiring/power/PID guide (see caveat below)
```

**`HUONG_DAN.md` is partly stale**: its wiring, power, microstep-jumper and A4988 Vref
sections are still correct and are the only place that information lives. Its PID section
describes v1 (single loop, `Kp=18 Ki=0.8 Kd=1.2`, a hardcoded `TARGET_ANGLE`, 45° fall
threshold) and no longer matches the firmware. Don't follow it for tuning; don't cite its
PID numbers.

## Firmware Architecture (`robot_balance.ino`)

### Control loop (100Hz, `LOOP_PERIOD_US = 10000`)
`loop()` is non-blocking: it handles BT input and one deferred-print step every pass, then
returns early unless 10ms have elapsed. On a tick:

1. Read MPU-6050 raw accel + gyro
2. Accel LPF (`accel_lpf`) → `accel_angle`; complementary filter (`cf_alpha`) → `angle`
3. **Soft resync**: if `|accel_angle - angle| > 3°`, pull `angle` 4% toward accel each tick.
   Kills long-term gyro drift without making the estimate sensitive to motor vibration.
4. Safety / fall check (below)
5. **Outer PID** (velocity loop, 10Hz, every `VEL_LOOP_DIVIDER` = 10 iterations):
   step-count delta → `target_angle`
6. **Inner PID** (angle loop, 100Hz): `target_angle - angle` → `pid_out` (step rate)
7. `setMotorSpeed(pid_out)` with acceleration ramping
8. Telemetry print, every tick (100Hz)

### Cascaded PID details
- **Inner (angle)**: `kp_eff*err + Ki_angle*I + Kd_angle*(-gyro_rate)`. The D term uses the
  raw gyro rate directly rather than differentiating the angle error — far less noise.
  `pidI_angle` clamped ±200. Output clamped to ±`max_steps_per_sec`.
- **Outer (velocity)**: velocity = Δ(average step count) per 100ms, low-passed by
  `vel_filter_alpha`. `vel_err = -vel_filtered * vel_dir` (`vel_dir` is `-1`; flip it live with `vdir 1` if the
  robot runs away instead of correcting). Produces
  `target_angle = target_bias + Kp_vel*vel_err + Ki_vel*pidI_vel + bt_drive`, clamped to
  ±(`max_target_offset` + 2).
- **Velocity anti-windup**: `pidI_vel` only accumulates when `|angle| < 5°` **and**
  `|bt_drive| < 0.01` — otherwise the integral fights the drive command. Clamped ±500.
  Every command that changes the setpoint (`F B S BIAS BIASADJ`) zeroes `pidI_vel`.

### Motor step generation
Timer1 (**prescaler 64 → 250kHz, 4µs/tick**) runs two independent compare channels: OCR1A
for motor L, OCR1B for motor R. Each ISR emits one step pulse and reschedules itself at
`OCR + period`. `period = 250000 / steps_per_sec`. `ZERO_SPEED` (32000) is the "stopped"
sentinel. No blocking in the main loop.

**The prescaler was 8 (2MHz) until 2026-10-25.** Because the longest usable period is
`ZERO_SPEED` ticks, a 0.5µs tick put the slowest commandable speed at 62 step/s — an 11mm/s
deadband that left the motors fully stopped 28% of the time while |angle| < 2°. At 4µs/tick
the floor drops to **7.8 step/s**. The cost is step-timing resolution at speed: 3000 step/s
is 83 ticks (1.2% quantisation), which is immaterial.

**The slip guard runs only when actually stepping.** The `(int16_t)(next - TCNT1) < 4` test
misreads any period above 32767 as negative, so the stopped case returns early
(`OCR1A = TCNT1 + ZERO_SPEED`) before reaching it. Keep that ordering if you touch the ISR.

`setMotorSpeed()` ramps `current_speed` toward the target by at most `max_accel` per tick,
then applies `bt_turn` as a differential (`L = speed - turn`, `R = speed + turn`).
Ramping limits peak current and prevents step loss.

### Deferred print
Blocking `Serial.print` of the full param set in a 100Hz loop overruns the TX buffer.
Solution: the `deferred_print` state machine (global `uint8_t`, 0=idle, 1–8=in-progress).
`doPrintStep()` prints one field per **tick**; a full dump takes 8 ticks (~80ms).
`printParams()` just sets the flag — never make it print inline.

**The call must stay below the `if (elapsed < LOOP_PERIOD_US) return;` gate.** It originally
sat above it, and since `loop()` spins thousands of times per 10ms, all 6 lines flushed
within microseconds — ~240 bytes into a 64-byte TX buffer, blocking ~15ms and stalling the
control loop every time someone pressed `p`. Exactly what the state machine existed to
prevent.

Per-tick telemetry is fine: ~43 bytes × 100Hz = 4.3KB/s against 11.5KB/s at 115200 baud.
This is why the baud change to 115200 was necessary; at 9600 it overflowed.

### Bluetooth command parsing
All commands are **line-buffered** — `handleBTCommand()` accumulates chars (32-byte buffer)
until `\n`/`\r`, then calls `parseCommand()`, which uppercases in place and splits on the
first space. Single-char parsing was removed because 'B','S','L','R' inside command names
(BIAS/SAVE/LOAD/RESET) collided with the motion commands.

**Phone apps send bare characters with no newline**, which cannot be reconciled with that
by content alone: `B` is both *backward* and the first letter of `BIAS`. Two rules added
2026-10-25 resolve it by **timing and repetition** instead, so both talkers work with no
mode switch and no change on the Python side:

1. **Idle flush.** If the buffer is non-empty and nothing has arrived for `BTIDLE` ms
   (default 60), parse what is there. The PC sends a whole line in one `sendall`, so at
   115200 baud the gaps *inside* a word are under 1 ms; a phone sends one character and
   then goes quiet for hundreds. Worst case, a BT packet split mid-word, is one `? XY`
   line, which is harmless.
2. **Repeat flush.** If the buffer holds exactly one joystick character and the *same*
   character arrives again, flush the first immediately. Apps repeat while a button is
   held, sometimes faster than `BTIDLE`, which would otherwise glue `FF` together. The
   rule requires the two characters to be **identical**, so `BIAS` (`B` then `I`, both
   joystick characters but different ones) is never split.

`BTIDLE` is a live parameter because 60 ms is an estimate: raise it if an app sends slowly
or packets arrive fragmented, lower it for a snappier joystick.

`test_bt_parser.py` simulates this state machine on the host and is the cheap way to check
a change without a USB upload. It covers `BIAS 1.4` with and without a trailing newline,
back-to-back `KP 45` + `KD 2.5`, `F` held at 50 ms and at 100 ms repeat rates, and an app
`R` immediately followed by a PC `RESET`. 12/12 at the time of the change.

```powershell
python test_bt_parser.py
```

### Phone control via an Arduino-BT-style app
Pair the phone with the HC-05 first (PIN is usually `1234`). The firmware answers the
standard character set those apps emit:

| char | action |
|---|---|
| `F` `B` | drive forward / back, plus or minus `DRVSTEP` per press |
| `L` `R` | turn left / right, plus or minus `TRNSTEP` per press |
| `G` `I` `H` `J` | diagonals: forward-left, forward-right, back-left, back-right |
| `S` | stop both, clear the velocity integral |
| `0`-`9` | speed slider, maps to `app_speed` 0.1-1.0 |

`app_speed` scales the **ceiling** the joystick can reach. It deliberately does not write
`DRVMAX`/`TRNMAX`: those are EEPROM-backed tunables and a drive command has no business
modifying them.

Two things to know before connecting a phone:

- **The HC-05 accepts one connection at a time.** The phone and `robot_control.py` cannot
  both be attached, so disconnect one first.
- **Turn the telemetry down.** 100 Hz floods an app's terminal view and eats the uplink.
  From the PC, set `logdiv 20` (5 Hz) or `lognone`, then `save` while the robot is fallen,
  before switching over.

### Fall / recover
- Fall triggers on **either** filtered `angle` or raw `accel_angle` exceeding
  `fall_angle` (45°, `fall` cmd). The accel check exists so a drifted complementary filter can't bypass
  the safety cut; when accel alone trips it, the firmware resyncs `angle = accel_angle` and
  logs `[!] FALLEN (filter drift! resync angle)`.
- On fall: motors stopped and disabled, both integrators and `target_angle` zeroed.
- Recovery needs `|angle| < recover_angle` (10°, `rec` cmd) **and** `|gyro_rate| < recover_gyro` (`recg` cmd)
  (100°/s). Step counters reset, then **soft-start**: `Kp_angle` ramps 50% → 100% over
  `SOFT_START_LOOPS` (300 ticks = 3s) to avoid post-recovery overshoot.

### Startup sequence
`loadParams()` from EEPROM → pin setup → `initMPU()` → `calibrate()` (2.5s settle + 1000
samples, robot must be held upright) → print initial angle → `initTimer1()` → enable motors
→ print `Bat dau!`. Python keys off these strings, so changing them breaks the controller —
notably `Bat dau!` (triggers startup params), `[!] FALLEN`, `[+] RECOVERED`, `Calib:`,
`Kp=`, and the `> ` reply prefix.

Calibration measures `gyroBias` and `upright_offset` (the accel angle while held upright),
so there is no hardcoded target angle. Residual CG offset is handled by `target_bias`.

**`upright_offset` is not repeatable**: across three logged boots it read 4.51° / 2.32° /
5.47°. Whatever angle the robot is held at during calibration *becomes* its definition of
vertical, so a 3° spread in how you hold it is a 3° error in the setpoint. Hold it against
something fixed.

**`BIAS_HINT` was removed** (it used to print right after calibration and Python auto-applied
it as `BIAS`). It computed `one accel sample − the mean of the 1000 samples just taken`,
whose expected value is zero: it measured sensor noise, not CG offset. Logged values were
−0.65 / 0.33 / −1.32, i.e. a random ±1° lean injected on every boot. Use `bias <val>` by hand.

### SAVE / LOAD / EEPROM
`Params` struct at address 0, guarded by `EEPROM_MAGIC` 0xAB43. **Changing the field order
or adding a field silently invalidates saved data** unless you bump the magic — a stale
struct would be reinterpreted field-by-field. `loadParams()` returns false on magic
mismatch and the firmware falls back to compile-time defaults.

`EEPROM.put()` blocks ~150ms, which drops the robot. **SAVE is restricted to
`fallen == true`** (firmware enforces this; it prints a hint otherwise). Intentionally tip
the robot over, then type `save`. LOAD and RESET work anytime.

## Parameter table — how to add a tunable

Every tunable lives in one `PROGMEM` table in the `.ino`. **Adding a parameter is one line.**
There is no Python-side list to keep in sync.

```c
const ParamDef PARAMS[] PROGMEM = {
//  name       variable            type      min       max      default
  { "KP",      &Kp_angle,          PT_F32,    0.0f,   500.0f,    80.0f  },
  { "ACC",     &max_accel,         PT_I16,   50.0f,  3000.0f,   350.0f  },
  { "ARCH",    &integrate_out,     PT_U8,     0.0f,     1.0f,     0.0f  },
  ...
};
```

One line gets you: the BT command, range clamping, EEPROM save/load, `reset` to default,
and a line in the `p` dump. Types are `PT_F32 / PT_I16 / PT_I8 / PT_U8`; everything is
carried over the wire and in EEPROM as a float and cast on store.

- **Side effects** (e.g. `DLPF` must be pushed to the MPU, `ARCH` must clear controller
  state) go in `onParamSet()`, matched by name.
- **Relative commands** like `BIASADJ` are not table entries — they stay as explicit
  branches in `parseCommand()` ahead of the table lookup.
- `parseCommand()` handles action commands (`F B L R S P GET BIASADJ SAVE LOAD RESET CAL`)
  first, then falls through to `findParam()`. An unknown name returns `? NAME`.
- Sending a bare name with no value **reads it back** — `KP` → `> KP=80.0000`.

**Python needs no changes for a new parameter.** `_TUNE_RE` is generic
(`^([a-z][a-z0-9_]{0,8})(?:\s+(number))?$`), uppercases the word and forwards it. It must
stay **last** in `process_command()`, after every action command, or it will swallow
`save`, `f`, `p` etc. The firmware is the single source of truth for what exists.

### Defaults live in two places, and they must agree
A parameter's default is written twice: the `default` column of the `PARAMS` row, and the
**initialiser of the global variable itself**. `resetParams()` and the `reset` command use
the table column. But when the EEPROM magic does not match, the firmware does *not* call
`resetParams()` — the variables simply keep their declared initialisers. So the two paths,
"`reset`" and "boot with no valid EEPROM", can hand you different configurations.

This bit on 2026-10-26: `UPOFF`'s table default was changed to 2.0 but the declaration was
left at `= 0`, so a fresh board booted with `UPOFF=0` while `reset` produced 2.0.

`check_defaults.py` parses both and compares them. Run it after touching any default:

```powershell
python check_defaults.py
```

It understands hex initialisers (`dlpf_cfg = 0x05`, `log_mask = 0x00FF`), because a row it
cannot parse is a row it silently stops checking.

### EEPROM
Layout is `[magic u16][float × NUM_PARAMS]` in table order. `EEPROM_MAGIC` is
`0xAB00 | NUM_PARAMS`, so adding or removing a parameter self-invalidates old data and the
firmware falls back to defaults. **Reordering rows while keeping the count does not
invalidate** — after a reorder, `reset` then `save`.

`EEPROM.put()` blocks ~150ms, so SAVE is still gated on `fallen == true`.

### Printing
`doPrintStep()` walks the table one row per tick: `KP=80.0000 [0.00..500.00]`. A full dump
is `NUM_PARAMS` ticks (~220ms), non-blocking. Python's `_PARAM_LINE_RE` parses these back
and mirrors the few it shows in the telemetry panel (`_PANEL_FIELDS`).

## Key Parameters

| Parameter | Default | Command | Accepted range | Note |
|---|---|---|---|---|
| `Kp_angle` | 80 | `kp 80` | — | Inner P. **`arch 1` needs ~45**, not 80 — different units entirely |
| `Ki_angle` | 0 | `ki 0` | — | Inner I |
| `Kd_angle` | 12 | `kd 12` | — | Inner D. **`arch 1` needs ~2.5**, not 12 |
| `Kp_vel` | 0.008 | `kpv 0.008` | — | Outer P — keep low, diverges fast |
| `Ki_vel` | 0.001 | `kiv 0.001` | — | Outer I |
| `target_bias` | 0 | `bias 1.4` / `biasadj +0.3` | — | CG offset (degrees forward lean) |
| `cf_alpha` | 0.97 | `alpha 0.95` | 0.80–0.99 | Complementary filter |
| `accel_lpf` | 0.85 | `alpf 0.85` | 0.30–0.95 | Accel noise filter (0.85 drifts less than 0.7) |
| `vel_filter_alpha` | 0.6 | `vfa 0.6` | 0.0–0.95 | Velocity filter |
| `max_target_offset` | 3.0° | `mto 3.0` | 1.0–8.0 | Max angle the velocity loop can request |
| `max_steps_per_sec` | 7000 | `spd 7000` | 500–15000 | Speed limit (7000 step/s ≈ 1.24 m/s on 90mm wheels). Run `arch 1` at **3000** — it caps the current draw if the integrator ever runs away |
| `max_accel` | 350 | `acc 350` | 200–3000 | Accel ramp, step/s per 10ms tick. **350 ≈ 6.2 m/s²; matches B-ROBOT EVO2 (MAX_ACCEL 14 × 25) and the motors' real torque. The old 1500 = 26.5 m/s², ~6× beyond what the steppers can deliver → lost steps.** |
| `drive_vel_gain` | 20 | `drvvel 20` | −200–200 | **Drive scaling**: `DRIVE 1.0` = 20 steps/100ms = 200 step/s = 3.5 cm/s. Negative flips forward/back, so the direction costs a BT command rather than an upload |
| `vel_cmd_max` | 40 | `vmax 40` | 10–2000 | Ceiling on `vel_setpoint`, both the position loop's and the drive command's. 200 let the position loop demand 872 step/s and topple the robot |
| `upright_offset` | 2.0 | `upoff 2.0` | −90–90 | MPU mounting angle. Mechanical constant: measure once with `calup`, or read it off `target` while balancing, then `save` |
| `bt_idle_ms` | 60 | `btidle 60` | 10–250 | Quiet gap that ends a command, for phone apps that send bare characters |

Values outside the range are clamped by `constrain()`, not rejected. `reset` restores
exactly this column. The in-app help text in `robot_control.py` still shows a couple of
superseded defaults (`alpf 0.70`, `acc 1200`) — this table and the firmware are authoritative.

**The firmware defaults are now the measured-good set** (2026-10-25): `ARCH 1`, `KP 45`,
`KD 2.5`, `VDIR 1`, `SPD 3000`, `VFA 0.7`, `ACC 350`. The robot balances straight out of
reset with no PC and no EEPROM. The `Default` column above is the pre-2026-10-22 history
and is kept only to explain the old notes; trust `configs\balance_good.txt`.

Compile-time only, everything else is a live BT command: `MICROSTEP` (declared, never
used), `LOOP_PERIOD_US`, `VEL_LOOP_DIVIDER`, `ZERO_SPEED`, `MIN_STEP_RATE`,
`SOFT_START_LOOPS`, the pin macros.

## PC Controller (`robot_control.py`)

Windows-only. Connects over **RFCOMM via `socket.AF_BLUETOOTH`** (stdlib, Python 3.3+) —
not a COM port, no pybluez. The HC-05 must already be paired in Windows Bluetooth settings.
Device discovery shells out to PowerShell `Get-PnpDevice -Class Bluetooth` to list paired
MACs.

```powershell
pip install rich prompt_toolkit        # pyserial only needed for set_hc05_baud.py
python robot_control.py                # scan + pick from paired devices
python robot_control.py AA:BB:CC:DD:EE:FF
```

A background reader thread parses lines into a `Telemetry` dataclass under a lock; the main
thread runs a `prompt_toolkit` session with a 1s timeout so it can poll `_pending_startup`.

### Config files — `configs\*.txt`
Tuned parameter sets live in `configs\`, **not** in the Python source. One command per
line, `#` comments, order preserved (it matters — `S` goes last, to clear the integrators
and latch `pos_target` after everything else is set).

`configs\balance_good.txt` is the measured-good set and is what `STARTUP_PARAMS` loads at
import, so it is sent automatically whenever the Arduino prints `"Bat dau!"`. Change
parameters by editing that file; do not hand-edit the Python list.

```
cfg                  # list configs\*.txt
cfg balance_good     # send it now, 120ms between lines
```

A missing file is not fatal — `load_config()` raises, the module prints a warning and
`STARTUP_PARAMS` falls back to empty.

### Commands (type at prompt)
| Input | Sends | Effect |
|---|---|---|
| `f` / `b` | `F\n` / `B\n` | Drive forward/back (±0.4° per press, clamped ±1.5°) |
| `l` / `r` | `L\n` / `R\n` | Turn left/right (±100 step/s per press, clamped ±300) |
| `s` | `S\n` | Stop drive + turn, zero velocity integral |
| `f3`, `b2`, `r4` … | repeats | Same command N times (max 20), 50ms apart |
| `p` / `get` | `P\n` / `GET\n` | Print all params (deferred, safe) |
| `t` | — | Render the local telemetry panel (no BT traffic) |
| `bias 1.4` | `BIAS 1.4\n` | Set CG offset |
| `biasadj -0.2` | `BIASADJ -0.2\n` | Adjust CG offset relatively |
| `kp 85`, `kd 14`, `kpv 0.01`, … | `KP 85\n` … | Any key in `_TUNE_RE` |
| `arch 0` / `arch 1` | `ARCH n\n` | **Output structure**: 0 = speed direct, 1 = integrating (B-ROBOT). Resets controller state; needs a different gain set — see below |
| `dlpf 5` | `DLPF 5\n` | MPU CONFIG reg 0x1A, 0–6 (4=21Hz, 5=10Hz, 6=5Hz). Applied immediately |
| `fall 45` | `FALL 45\n` | Fall threshold |
| `rec 10` / `recg 100` | `REC`/`RECG` | Recovery thresholds (angle, gyro) |
| `vdir -1` | `VDIR -1\n` | Flip velocity-loop sign |
| `cal` | `CAL\n` | Re-run calibration without a power cycle (only while fallen, blocks 4s) |
| `save` | `SAVE\n` | Save to EEPROM (only while fallen) |
| `load` | `LOAD\n` | Load from EEPROM |
| `reset` | `RESET\n` | Reset params to firmware defaults |
| `raw <str>` | `<str>` | Send raw string, no newline added |
| `help` | — | In-app help |
| `q` / `quit` | — | Exit (or Ctrl+C twice) |

Adding a tuning command means touching three places: the `parseCommand()` branch in the
`.ino`, `_TUNE_RE` + `cmd_map` in `process_command()`, and the help text.

### Browser control panel (`robot_ui.py`)

```powershell
python robot_ui.py                      # http://127.0.0.1:8777/
python robot_ui.py <MAC> --port 8800 --no-browser
```

Python holds the RFCOMM socket and serves a page on localhost; a browser cannot open
Bluetooth itself, so a static page was never an option. Server side is standard library
only — `ThreadingHTTPServer` plus server-sent events at 20 Hz. `GET /state` returns the
same snapshot as one SSE frame, which makes it easy to check from the shell:

```powershell
curl -s http://127.0.0.1:8777/state
```

**It holds no list of parameters or columns.** On connect it sends `COLS` then `P`, parses
`NAME=value [min..max]` and builds one slider per parameter bounded by the firmware's own
range; telemetry columns come from `#COLS`. Same contract as `robot_control.py`: adding a
`PARAMS` row needs no change here. `PRIORITY` only decides which names sort to the top, and
an unlisted name still appears.

Three things that are easy to get wrong, all of them found by running it:

- **State cannot be inferred from telemetry arriving.** That was true until 2026-10-26,
  when the fall branch started emitting telemetry too. The panel reads `|angle|` against
  the `FALL` parameter it already has instead.
- **Corrupt BT frames must be filtered before they reach the log.** A broken packet
  arrives as hundreds of NUL bytes; logged verbatim it inflated every SSE frame to tens of
  KB, twenty times a second. `Link.clean()` drops any line that is more than 40%
  unprintable, and split telemetry rows are counted as `junk` rather than logged. The frame
  went from ~40 KB to 2.3 KB.
- **Slider drags are coalesced over 150 ms.** Sending every `input` event floods the uplink
  and pushes telemetry out of the 115200-baud budget.

**Turning is a rate command, so the panel pulses it.** `L`/`R` set a turn *rate* that
holds until something changes it, so one keypress spins the robot until you stop it —
`TURN 200` held for 3 s was a full quarter turn. The pad sends `TURN n`, then `TURN 0`
after an adjustable interval, which makes one press a repeatable turn *amount*. 400 ms at
`TRNSTEP 100` is roughly 6 degrees, from the measured 220.9 mm of wheel differential per
quarter turn at a 140.6 mm track. Drive is deliberately left latching, because driving is
something you want to hold.

**The stop button does not send `S`.** It sends `DRIVE 0` + `TURN 0`. See the note under
the parameter table about `S` clearing the level trim; the panel's version works correctly
on firmware from either side of that fix.

**One connection at a time.** The HC-05 accepts a single RFCOMM link, so `robot_ui.py`,
`robot_control.py`, `robot_auto.py` and a phone are mutually exclusive. Two of them running
at once is not a clean failure: both appear to work while the stream is split between them
and arrives corrupted. If the log fills with junk-line counts, check for a second client
before suspecting the radio.

### Log files
Each session writes **two** files into `logs\`:

| File | Content |
|---|---|
| `robot_YYYYMMDD_HHMMSS.log` | Session transcript: firmware events, command echoes, and one condensed status line every `STATUS_EVERY` (50) samples ≈ 0.5s |
| `robot_YYYYMMDD_HHMMSS_raw.tsv` | **Every** telemetry sample at 100Hz, with a `t_ms` column. This is the file to analyse. |

Transcript status line:

```
#150   BALANCING  -0.33° tgt+0.00° spd  +38 gyro -1.0°/s  drift+0.7°
```

`drift` (filtered angle − accel angle) is only printed when it exceeds 0.5° — it is the
early warning for complementary-filter divergence.

The `.tsv` is written by `log_raw()` from the reader thread (4KB buffered, flushed on exit).
At 0.5s per status line the transcript aliases anything above 1Hz, which made the earlier
±90°/s oscillations impossible to characterise — that is why the raw file exists.

**`stepsL`/`stepsR` count commanded steps, not actual ones.** If the motors skip, these
counters — and therefore the whole velocity loop — are reading fiction. See **Motor torque
budget** below.

### Telemetry table — choosing what gets logged
Which fields stream is **runtime-configurable**, same pattern as the parameter table.
Adding a loggable variable is one line in `VARS[]`:

```c
const VarDef VARS[] PROGMEM = {
//  name       variable          type      decimals
  { "angle",   &angle,           VT_F32,   2 },
  { "stepsL",  (void*)&stepsL,   VT_I32V,  0 },   // volatile → read atomically
  { "ctrlout", &control_output,  VT_F32,   0 },
  ...
};
```

`log_mask` is a bitfield over the table, one bit per row, default `0xFF` (`ms` + the classic
seven). Types: `VT_F32 / VT_I16 / VT_I32 / VT_I32V`. Anything the loop computes must be a
**global** for the table to point at it — `gyro_rate`, `accel_angle`, `vel_filtered`,
`vel_err`, `pid_out`, `dt` and `loop_ms` were promoted out of `loop()` for exactly this.

**`log_mask` must stay wider than `NUM_VARS`.** It was `uint16_t` when the table grew to 17
rows; `(uint32_t)1 << 16` silently became 0, so the 17th row (`dt`) could never be toggled —
`log dt` returned an unchanged `#COLS` with no error. It is `uint32_t` now, good to 32 rows.
Past 32, widen it again *and* the shift casts. `LOGMASK` is capped at 16777215 (2^24) because
the table carries values as float, and float32 stops representing consecutive integers there.

**Use the `ms` column, not `t_ms`, for any timing analysis.** `t_ms` is stamped by the PC
when the line is *parsed*, and HC-05 delivers in bursts: a 15s parked capture on 2026-10-21
showed an apparent 9.37ms mean period with 26.7ms stdev and a 127ms "gap" — entirely a
Bluetooth buffering artifact, the loop was fine. `ms` is `millis()` read inside the tick, so
it reflects real robot time.

| Command | Effect |
|---|---|
| `log` | List rows, `[x]` = enabled |
| `log gyro` | Toggle one row (name match is case-insensitive) |
| `logall` / `lognone` | All / none |
| `logmask 127` | Set the bitfield directly (it is a `PARAMS` row, so it persists in EEPROM) |
| `logdiv 2` | Rate divider: 1 = 100Hz, 2 = 50Hz, 4 = 25Hz |
| `cols` | Re-emit the header line |

Off by default, enable when debugging: `pidout ctrlout ivel iang velf velerr dt`.
`ctrlout` is the one that matters when testing `arch 1` — under `arch 0` it equals `pidout`,
under `arch 1` it is their running integral.

**`#COLS` is the contract.** On startup and on every `log_mask` change the firmware emits

```
#COLS angle accel target gyro speed stepsL stepsR
```

and Python treats that as the sole source of truth: it sets `self._cols`, writes the `.tsv`
header (or a `#COLS` marker line if the set changes mid-session), and accepts a telemetry
row only when its field count matches. Column→panel mapping is `_TELEM_FIELDS`; unmapped
columns still reach the `.tsv`, they just do not show in the `t` panel. **Python holds no
list of field names beyond that mapping** — a new `VARS[]` row needs no Python change.

**Bandwidth**: 115200 baud ≈ 11.5 KB/s; each float field is ~7 bytes. Seven fields at 100Hz
is 4.3 KB/s (safe). Past ~10 fields, set `logdiv 2`.

## Mechanics & physical model

The firmware contains **no mechanical constants** — `MICROSTEP` is defined but never used,
and everything is in raw step/s. These are the conversions, measured 2026-10-21:

| | |
|---|---|
| Wheel diameter | 90mm (3D printed, GT2 belt as tyre), direct drive |
| Steps per rev | 200 × 8 microsteps = 1600 |
| **1 step** | **0.177mm** |
| `spd 7000` | 1.24 m/s |
| `acc 350` | 6.2 m/s² |
| Battery | 3× 18650 in series (11.1V), ≈160g |
| Axle height | 45mm; middle deck +55mm, top deck +124mm relative to axle |

Inverted-pendulum model (masses estimated, not weighed — refine if you weigh the parts):

```
m = 1.0kg   L_cg = 14mm above axle   I_cg = 1.45e-3 kg·m²   k = 38mm
p = √(m·g·L / (I_cg + m·L²)) = 9.0 rad/s   →   fall time constant τ = 111ms
p·T ≈ 0.21 with T ≈ 23ms (10ms loop + 13.8ms DLPF)
```

**Do not use `p = √(g/L)`** — that is the point-mass form and overstates `p` by 2.5× here.
The rigid-body form is non-monotonic: it peaks at `L = k`, so with `L < k` lowering the CG
makes the robot *more* stable, not less. Relocating the battery to the top deck moves
`L`→50mm but also `k`→53mm, landing exactly on the worst point; it was computed and
rejected (9.0 → 9.6 rad/s, no benefit).

**IMU placement.** The accelerometer at radius `r` from the axle reads a spurious `α·r`
during angular acceleration. At the original top-deck mount (r=124mm) a 10Hz / ±90°/s
oscillation injects ≈1.25g — enough to swamp gravity entirely, which is what produced the
`FALLEN (filter drift!)` events. The MPU was moved to the middle deck (r=55mm, ≈0.55g) on
2026-10-21, same orientation. The bottom (yellow) plate at r=21mm would be better still
(≈0.21g) and is the stiffest part of the frame, if space allows.

Orientation is load-bearing: the code assumes **Z up, X parallel to the wheel axle**
(`atan2f(-ay, az)`, `gyro_rate` from `gx`). Mounting the board inverted requires
`atan2f(ay, -az)`. Mount it face-up and this never comes up.

Whole-body horizontal acceleration corrupts the accel angle identically at every point
(`g·sinθ − ẍ·cosθ`) and cannot be fixed by moving the sensor — that is the complementary
filter's job.

**Motor torque budget.** NEMA17 ≈0.4 N·m holding at 1.7A. At Vref 0.6V (R_sense 0.1Ω →
0.75A, 44% of rated) and derated ~50% for speed at 11.1V, the pair delivers ≈0.18 N·m →
3.9 N at the wheel → **≈4 m/s²**. `acc 1500` commanded 26.5 m/s², roughly 6× that, which
guarantees skipped steps. Check Vref with a meter before trusting any of this.

## Relationship to B-ROBOT EVO2

Upstream: [jjrobots/B-ROBOT_EVO2](https://github.com/jjrobots/B-ROBOT_EVO2),
`Arduino/BROBOT_EVO2/`. This project borrowed the Timer1 ISR step generator but **not** the
control structure, which is the single largest divergence:

```c
// B-ROBOT: PD output is INTEGRATED — the PD commands acceleration
control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
setMotorSpeedM1(control_output + steering);

// here: PID output IS the speed
setMotorSpeed((int16_t)pid_out);
```

A stepper takes a *speed* command, so commanding speed directly puts an extra
differentiation in the loop: the effective law becomes `a = Kp·θ̇ + Kd·θ̈` instead of
`a = Kp·θ + Kd·θ̇`. `Kd` then depends on angular *acceleration*, the noisiest quantity the
MPU can supply. (For a DC motor, PWM ≈ torque ≈ acceleration, so direct output is correct —
this only bites with steppers.)

**Both forms are now compiled in**, selected at runtime by `integrate_out` (`arch 0|1` over
BT), so the comparison can be run without reflashing. The two modes need completely
different gains and are not interchangeable:

| `arch` | Output | Starting gains |
|---|---|---|
| 0 (default) | `control_output = pid_out` | `kp 80`, `kd 12` |
| 1 | `control_output += pid_out` | `kp 8`, `kd 125` |

The `arch 1` gains are derived from upstream `KP 0.32 / KD 0.050` scaled by the ×25
control-unit factor, with `Kd/DT` at DT=0.01 — **estimates from reading upstream, not
measured.** Expect to retune from there. Switching `arch` zeroes `control_output` and both
integrators, so do it while the robot is down.

Reference values, normalised to 1/8 microstep (upstream scales `×50` for 1/16, `×25` for 1/8):

| | B-ROBOT EVO2 | here |
|---|---|---|
| MAX_ACCEL | 14 × 25 = 350/tick | 350 (was 1500) |
| Max speed | 500 × 25 = 12500 step/s | 7000 |
| DLPF | 10Hz | 10Hz (was 21Hz) |
| Gyro FS | ±500°/s | ±250°/s — logged peak is 97°/s, so no saturation; left alone |
| CF alpha | 0.99 | 0.97 |
| Sampling | SMPLRT_DIV=9 + data-ready interrupt | `micros()` polling |
| Fall / recover | 74° / resume under 74° | 45° / 10° + gyro<100°/s |
| Gains | KP 0.32, KD 0.050 (integrating) | Kp 80, Kd 12 (direct) — not comparable |

## Measurements

### Baseline, parked, motors idle — 2026-10-21
IMU on the middle deck (moved down from the top deck that day), 15s, old firmware,
`logs\robot_20261021_162444_baseline_parked_raw.tsv`:

| | stdev | peak-to-peak |
|---|---|---|
| `angle` | 0.0091° | 0.05° |
| `accel` | 0.0314° | 0.24° |
| `gyro` | 0.0449°/s | 0.20°/s |
| drift (`angle − accel`) | 0.030° | max 0.14° |

**The sensor chain is clean at rest.** This is the reference: any noise seen while the
motors run is motor vibration, not the mount or the MPU. `gyro` shows a ~50Hz line at
0.023°/s amplitude — near Nyquist for a 107Hz capture, consistent with LSB dither
(1 LSB = 0.0076°/s at ±250°/s), not a mechanical mode.

Use `analyze_log.py` to reproduce: it prints per-column stats, the amplitude spectrum of
`angle`/`accel`/`gyro`, and the CF-vs-accel drift.

```powershell
python analyze_log.py                    # newest logs\*_raw.tsv
python analyze_log.py logs\<file>.tsv
python analyze_log.py --last             # the most recent segment, not the longest
```

**Use `--last` while tuning.** By default the tool analyses the *longest* contiguous block,
which is right when dissecting a finished session but wrong live: after a reboot or a
parameter change the segment you care about is the one just recorded, and it is usually
shorter than what came before. On 2026-10-26 the default picked a 7495-sample block from
before a reboot and reported `gyro` averaging −10.9 °/s, while the 3996 samples after it
averaged +1.2 — opposite conclusions from the same file.

### Loop timing — 2026-10-21, over USB
With the `ms` column, consecutive ticks read 5483 / 5493 / 5504 ms — a solid 100Hz. The
earlier "127ms gap, 26.7ms jitter" seen over Bluetooth was entirely HC-05 batching, not the
control loop. Always judge timing from `ms`, never from `t_ms`.

### Firmware verified on hardware — 2026-10-21
Uploaded to COM7 and checked end to end: `P` returns all 24 parameters with ranges, `LOG`
lists 17 variables with the first 8 enabled, `log dt` / `log ctrlout` / `logall` /
`logmask 255` each re-emit a correct `#COLS`, `logdiv` works, bare `KP` reads back
`> KP=80.0000`, and an unknown name returns `? XYZ`. Boot reports
`Using default params (no EEPROM data)` — nothing has ever been saved to EEPROM on this
robot, so the firmware's own defaults are what run.

### Motor vibration coupling — 2026-10-22, IMU on the middle deck
Robot parked, wheels free, velocity loop disabled (`KPV 0 KIV 0`, `MTO 8`), motors driven
open-loop by holding a fixed angle error (`BIAS b` → `pid_out = Kp·b`). ~700 samples each.

| speed | `gyro` σ | ×idle | spectral peak | `accel` σ |
|---|---|---|---|---|
| 0 (idle) | 0.0171°/s | 1.0 | 1.7 Hz | 0.0203° |
| 320 step/s | 0.0682°/s | 4.0 | **19.8 Hz** | 0.0315° |
| 640 step/s | 0.0375°/s | **2.2** | **20.1 Hz** | 0.0201° |
| 800 step/s | 0.0833°/s | 4.9 | 1.0 Hz | 0.2193° |

**Vibration is irrelevant to the controller.** Worst case is 0.083°/s, which through `Kd=12`
is ~1.0 step/s of command against a 350 step/s budget. This is the evidence that the May
2026 ±90°/s limit cycle was a **control-loop** problem — D-term on a speed-command
architecture plus step loss from `acc=1200` — and not sensor contamination.

Two things worth knowing for future tests:

- Motor vibration shows up as a ~20Hz line, and **320 step/s is worse than 640** — classic
  stepper low-speed resonance. Don't assume faster means noisier.
- The 800 step/s row is *not* vibration: the peak drops to 1.0Hz and `accel` σ jumps 10×.
  That is the whole robot rocking on its stand. Same artefact appeared when `L`/`R`
  (opposite-direction `bt_turn`) was used on a free-hanging robot — reaction torque twists
  the body, spectrum at 0.7–1.2Hz, `accel` spiking to 13.3°, drift 12.5°. **Drive both
  wheels the same direction, and treat any sub-5Hz peak as the robot moving, not vibration.**

### Power — resolved 2026-10-22 (loose pin headers)
For a day the Arduino looked dead or flaky on battery while being perfect on USB. The real
fault was **intermittent pin-header connections**; resoldering them fixed it completely
(0 resets in 25s, 100% telemetry throughput, no `ms` discontinuities).

Logged here because the symptoms were badly misleading and cost a lot of time:

| Symptom | What it actually was |
|---|---|
| "TX dead, RX works" | After each brownout reset the firmware spends ~4.5s in `setup()` + `calibrate()`, emitting **no telemetry**. Short commands landed in the brief running windows. |
| Motors spinning with no telemetry | Those same brief windows between resets |
| Throughput varying 100 / 75 / 14 / 0 Hz | How much of each capture fell inside a running window |
| Perfect on USB | USB bypasses the battery path entirely |
| A freshly charged pack did not help | It was never a capacity problem |

**The diagnostic that settled it**: two `=== ROBOT BALANCE v3 + BT ===` banners inside 12
seconds, plus `ms` uptimes that kept restarting small. When the link looks broken, count
boot banners and watch `ms` before theorising about radios, regulators or wiring.

**Operational note that still applies**: a robot parked at a non-zero angle with its wheels
free will drive the motors forever — the PID sees an error it can never correct. One session
accumulated 19,830 steps at −758 step/s. Park it level, or tip it past `fall_angle`.

### Oscillation frequency — finally measured, 2026-10-22
First real balancing run on the new firmware (`acc 350`, `DLPF 10Hz`, `arch 0`, IMU on the
middle deck). `logs\robot_20261022_111631_balance_run2_raw.tsv`, longest clean segment
2156 samples / 21.6s at 100Hz:

| | |
|---|---|
| **Oscillation** | **0.6 Hz** — `angle`, `accel`, `gyro`, `speed` and `target` all peak there |
| `angle` | σ 5.83°, range −15.7 .. +23.6° |
| `gyro` | σ 24.6°/s, peak 131.7°/s |
| `speed` | σ 523, peak 2643 step/s — **not** saturating against the 7000 limit |
| `stepsL − stepsR` | drifts only 32 steps over 21.6s — **no step loss** |
| drift (`angle − accel`) | σ 2.10°, max 17.08°, one `FALLEN (filter drift!)` |

**0.6Hz is the outer velocity loop, not the inner angle loop.** The robot's own unstable
pole is 9 rad/s ≈ 1.4Hz, so a 0.6Hz lurch is slower than it can fall; and `target` — which
only the 10Hz velocity loop moves — swings ±1.5° at exactly that frequency. Start tuning at
`kiv` and `kpv`, not `kp`/`kd`.

This also retires the old "±90°/s high-frequency limit cycle" reading of the May 2026 logs.
That was **aliasing**: the transcript sampled at 0.5s intervals, so a 0.6Hz lurch with
±130°/s peaks was unresolvable. The raw `.tsv` plus the `ms` column is what made the real
number visible.

`acc=350` looks vindicated: no step loss, no speed saturation.

### Power under motor load — resolved, confirmed 2026-10-25
Resoldering the pin headers fixed it. The idle case went clean first (0 resets in 25s), but
on 2026-10-22 it still browned out under motor load: **5 boot banners in a 75s balancing
run**, 2 of them visible as backwards `ms` jumps inside the telemetry, and that was the top
blocker for weeks.

**It no longer reproduces.** 2026-10-25, a full movement sweep — forward, back, both turns,
including `speed` pinned at the 3000 step/s ceiling — logged **0 boot banners**. Later runs
totalling several minutes of balancing and driving: still zero. Power is off the list.

Note the supply topology, since it was misdiagnosed repeatedly: a 12V→5V buck feeds the
Uno's `5V` pin directly, bypassing the onboard regulator. Whether the 100–470µF electrolytic
across the CNC shield's VIN that `HUONG_DAN.md` calls mandatory is actually fitted was never
confirmed, and no longer matters.

### IT BALANCES — `arch 1`, 2026-10-22
The robot balances with the **integrating** architecture. `arch 0` never could, and the
reason is structural, not a tuning failure: under `arch 0` a steady tilt commands a steady
wheel *speed*, and a wheel rolling at constant velocity exerts **no force**. The user's
description — "wheels turn the right way but there's not enough force" — is exactly that.

| | `arch 0` | `arch 1` kp12 kd180 | `arch 1` kp30 kd2.5 | **`arch 1` kp45 kd2.5** |
|---|---|---|---|---|
| Longest unbroken run | 21.6s | 5.8s | 79.6s | **88.6s** |
| `angle` σ | 5.83° | 7.58° | 5.50° | **1.46°** (1.04° once settled) |
| \|angle\| 2–4° after 0.25s | +20% | +20% | +33% | **−63%** |
| \|angle\| 4–7° after 0.25s | +2% | +2% | −8% | **−38%** |
| Oscillation | 0.6 Hz | — | 0.28 Hz | — |

**Working configuration, saved to EEPROM 2026-10-22** — the robot boots into this and
balances with no PC attached:

```
ARCH 1   KP 45   KD 2.5   KI 0
VDIR 1   KPV 0.008   KIV 0.001   VFA 0.7
ACC 350  SPD 3000   DLPF 5   BIAS 0
```

### The velocity-loop sign was inverted from the start
`vel_dir` defaulted to `-1`, which is **positive feedback**. Trace it:

```
vel_err = -vel_filtered × (-1) = +vel_filtered        (line ~900)
target_angle = ... + Kp_vel × vel_err                 → target ∝ +velocity
pid_out = kp_eff × (target - angle)                   → positive target → positive speed
speed > 0 → dirSignL = +1 → stepsL increases          → velocity grows further
```

Measured confirmation: correlation of `speed` against `target` was **+0.63** (2s-smoothed)
with `vel_dir = -1` and **−0.42** with `+1`. Flipping it halved `angle` σ (1.41° → 0.48°).

This is almost certainly why the old project notes warned that "`KPV` above 0.025 diverges —
`tgt` runs to `max_target_offset` and the robot falls". That is what positive feedback looks
like; it was read as excessive gain, and `kpv` was dialled down to 0.008 to hide it.

**But 0.008 turns out to be roughly right anyway.** With the sign fixed, raising it still
hurts: `kpv 0.03` gave `angle` σ 1.90° and only 11% of time within ±1° (`target` itself
swinging σ 1.90° and hitting the ±5° clamp), and `kpv 0.06` toppled the robot outright. The
outer loop at 10Hz must stay well below the inner loop's ~2.8Hz bandwidth. Tune `kiv`, not
`kpv` — the integral is slow and smooth, the proportional term chases velocity noise.

### Position loop — added 2026-10-25
`KPPOS` closes a third loop outside velocity, using the cumulative `stepsL`/`stepsR` as the
position feedback they already are. It produces a velocity setpoint that pulls the robot
back to `pos_target`:

```c
if (fabsf(bt_drive) > 0.01f || bt_turn != 0) pos_target = stepsAvg;   // dang lai -> nha
vel_setpoint = constrain(-Kp_pos * (stepsAvg - pos_target), -vel_cmd_max, vel_cmd_max);
vel_err      = (vel_setpoint - vel_filtered) * vel_dir;
```

`pos_target` is reset on recovery from a fall and by `S`. It tracks the robot while a drive
or turn command is active, so the loop never fights the pilot.

**`KPPOS 0.015`, not 0.05.** Measured: at 0.05 everything — `angle`, `target` and position —
locked into a 0.33Hz limit cycle, `angle` σ 1.855°, only 84% within ±1°. At 0.015 the peak
disappears (0.03Hz, i.e. slow wander), σ 0.272°, 99% within ±1°. The rule is the ordinary
cascade one: each outer loop must be slower than the loop it wraps, and the velocity loop
underneath takes tens of seconds to null a disturbance.

Drift with the loop on: **0.064 cm/s**, against 1.04 cm/s without it.

**Derive the gains from this robot's geometry, do not scale B-ROBOT's.** Upstream is a
taller robot with a much slower pole; porting its `Kd/Kp ≈ 15` ratio gave `kd 180`, which
put the dominant closed-loop pole at −0.11 rad/s (a 9-second time constant) and made the
robot unable to react at all. The correct derivation, with 1 step = 0.177mm at 100Hz:

```
accel produced   a = kp · θ(deg) · 0.0177  m/s²
accel to hold    a = g·sin θ = 0.171 · θ(deg)  m/s²
→ kp = 9.7 is neutral; kp = 45 gives ~4.6× gravity

closed loop:  θ̈ + 8.38·kd·θ̇ + (8.38·kp − 81)·θ = 0      (L_eff = g/p² = 0.121 m)
→ ω_n = √(8.38·kp − 81),  ζ = 8.38·kd / (2·ω_n)
→ kp 45 → ω_n = 17.6 rad/s;  ζ = 0.7 needs kd ≈ 2.9
```

**Starting an `arch 1` run**: `control_output` is a pure integrator with no leak. Holding
the robot upright for more than a few seconds winds it to saturation — one such hold ran the
wheels 21 metres at 7000 step/s and browned out the board 12 times. The integrator is zeroed
on a fall and on an `arch` command, so the protocol is: **let the robot lie fallen, stand it
up, release immediately.** Never hold it standing.

### Known limit: low-speed deadband
`setSingleMotor()` computes `period = 2000000 / speed` and treats `period >= ZERO_SPEED`
(32000) as stopped, so **any command below 62 step/s stops the motors** — double the declared
`MIN_STEP_RATE` of 30, and equal to 11 mm/s of wheel speed. Measured effect: motors are fully
stopped **28% of the time while |angle| < 2°**, versus 2.7% above 4°. That is why the robot
recovers big leans well but cannot trim finely, and it is the likely driver of the slow
0.28Hz limit cycle.

Fix (needs a USB upload): raise `ZERO_SPEED` toward 65000 — still inside `uint16_t` — which
brings the deadband down to ~31 step/s. Alternatively use a larger Timer1 prescaler, at the
cost of step-timing resolution at high speed.

### Gyro bias is the single biggest failure mode — 2026-10-25
A robot that had been balancing beautifully (`angle` σ 0.018°, ±3 steps of position drift
over 14s) crept forward **1 m over 100 s and then charged backwards at 3000 step/s and
fell**, twice, with no command sent. `logs
obot_20261025_194011_idle_raw.tsv` caught it.

The telltale is in the quiet part of the log, not the crash. While parked and visibly
still, `gyro` read −10 to −29 °/s and **never crossed zero**. A stationary robot cannot
rotate at −20 °/s for ten seconds. That is `gyroBias`, measured wrong at boot because the
robot was touched during `calibrateGyro()`'s 4-second window.

Everything else followed from it:

| | |
|---|---|
| CF steady-state error | `α·b·dt/(1−α)` = `0.97 × 20 × 0.01 / 0.03` ≈ 6.5°; measured `angle − accel` = −3.6° |
| Robot's belief | thinks it is vertical while leaning ~3.6° |
| Consequence | must creep continuously to stay up → 1 cm/s, exactly the pre-position-loop drift figure |
| Position loop | P-only, so a constant disturbance leaves a **steady-state error of ~1 m**, it does not converge |
| At 1 m of error | demands `0.015 × 5814 = 872 step/s`, and `VMAX 200` (= 2000 step/s) let it through |
| Result | `speed` ramps to the 3000 ceiling in 2.5 s, robot runs 73 cm backwards and falls |

Fixes, both live over BT:
- `cal` while fallen → re-measure `gyroBias`. It came back **−324.66 LSB (−2.48 °/s)**, i.e.
  the boot value had been off by ~20 °/s.
- `VMAX 200 → 40` (400 step/s). The position loop must never demand more than the robot can
  accelerate to without falling.

After: `gyro` mean **−0.01 °/s**, drift **+0.02°**, position **−0.02 cm in 25 s**. A knock
at t=15s moved it 2.57 cm and it returned to **0.00 cm**.

**Check `gyro`'s mean before trusting any other measurement.** It should sit within ±0.2 °/s
while the robot is standing still. If it does not, nothing downstream means anything.

### Leaning does not drive this robot — detent deadband, 2026-10-25
`bt_drive` used to be added straight to `target_angle`, on the assumption that a commanded
lean makes the robot roll. Measured, it does not:

| command | resulting `angle` | distance in 3 s |
|---|---|---|
| `DRIVE 0.4` | +0.41° | −0.04 cm |
| `DRIVE 1.0` | +1.01° | **+0.10 cm** |
| `DRIVE 2.5` | +2.73° | +6.45 cm |
| `DRIVE -4.0` | −4.07° | +18.61 cm |

The inner loop tracks `target` perfectly in every row — the controller is not at fault. The
CG is only **59 mm above the ground** (45 mm axle + 14 mm `L_cg`), so a 1° lean puts the CG
1.03 mm off-centre and the toppling torque about the contact patch is
`1.0 × 9.81 × 0.00103 ≈ 10 mN·m`, i.e. **5 mN·m per motor — below a NEMA17's 10–16 mN·m
detent torque.** The wheels simply refuse to roll. Threshold:
`asin(2 × 13 mN·m / (9.81 × 0.059)) ≈ 2.6°`, while `DRVMAX` was clamping at 1.5°. Every
drive command sat inside the deadband.

This is a *torque* deadband and has nothing to do with the step-rate deadband below.

**Fix (firmware, 2026-10-25): `bt_drive` now sets `vel_setpoint` instead of `target_angle`.**
`DRVVEL` (default 20) converts it: `DRIVE 1.0` = 20 steps/100ms = 200 step/s = 3.5 cm/s. The
velocity loop then leans as far as it needs to, 3° or 5°, instead of being capped at 1.5°.

Two related bugs fixed in the same upload:
- **Anti-windup ignored `bt_turn`.** A 2-second turn wound `pidI_vel` to −492 of its ±500
  clamp and left it there, biasing `target_angle` afterwards — the robot drifted 8 cm during
  a turn that should have been in place. The condition is now `bt_turn == 0`, and it no
  longer excludes `bt_drive` (that integral is now what *produces* the drive lean).
- **`F` / `B` / `DRIVE` zeroed `pidI_vel`**, wiping the level trim. Measured: `DRIVE 0.4`
  dropped `target` from 0.68 to 0.40, so the "forward" command leaned the robot *backwards*
  by 0.28°.

Also: the firmware now emits telemetry while `fallen`. The fall branch used to `return`
before the print, so the PC could not tell "robot is lying down" from "link is dead" — a
mistake made twice in one session.

**The runaway, reproduced on command.** `DRIVE -4.5` held for 6 s
(`logs
obot_20261025_210431_one_raw.tsv`):

```
t=1s   +4.08cm   angle -5.13  tgt -4.82  spd  +842
t=2s  +25.24cm   angle -5.65  tgt -5.56  spd +1678
t=3s  +72.58cm   angle -6.60  tgt -6.55  spd +2766
t=4s +100.90cm   angle -31.18 tgt -6.83  spd +3000   -> FALLEN
```

`tgt` walks itself outward, −4.82 → −5.56 → −6.55, until it hits the `±(MTO+2)` clamp. The
faster it goes the harder it leans. That is the "lao 1 mét rồi ngã" the operator kept
reporting, and it is positive feedback, not excessive acceleration.

It is **asymmetric**, which is the giveaway: `DRIVE +4.5` lurched 7.1 cm in two seconds and
then sat dead still for four more at a 4.45° lean, while `DRIVE -4.5` ran a metre. Same
magnitude, opposite outcomes.

Also note what a 4.45° sustained lean with stationary wheels means: the steppers are
**energised and holding**. 0.18 N·m of holding torque against ~45 mN·m of toppling torque
is no contest. A stepper is a position device; it does not roll just because the body
leans. This is the same lesson as the `arch 0` failure, one level deeper.

**Measuring the feedback sign by correlation does not work.** `corr(pos, target)` came out
at −0.49 / −0.95 / −0.69 across three command-free logs, which looks conclusive and proves
nothing: in a closed loop that correlation is dominated by the *plant* (lean → acceleration),
not by the controller. The reliable facts are the two large open-loop-ish runs:
`tgt +4.5 → −7.6 cm`, `tgt −5 → +100 cm`, i.e. **positive `target` ↔ negative steps**.

**Two-second drive pulses do not measure speed.** `DRIVE +2.5` read −6.27 cm during the
pulse and +6.45 cm after `S`; the wheels must roll backwards to tip the body forward, then
overtake it, and stopping reverses that. What a short pulse measures is the lurch and the
brake, not the travel. Hold a drive command for 5 s or more before reading a distance.

### Turning is the one drive path that already worked — 2026-10-25
`TURN 200` for 3 s: wheel differential **+220.9 mm**, accumulating evenly (71 → 143 → 216),
`angle` σ 0.38° inside −1.0 .. +0.3°, `speed` peak 270 against the 3000 ceiling, no fall,
no runaway. Forward/back could not manage any of that.

The reason is structural: `bt_turn` is applied as a differential on the motor command
(`L = speed - turn`, `R = speed + turn`) and **never passes through `target_angle`**. It was
already a velocity command. That is precisely what `bt_drive` was changed into.

`TURN` positive = right (clockwise), confirmed by eye. 220.9 mm of differential produced a
quarter turn, so **`WHEEL_TRACK_MM` = 220.9 / (π/2) = 140.6 mm**, not the 120 mm that had
been guessed from a photo — `spin <deg>` was under-rotating by about 17%.

One defect remains visible: `ivel` sat pinned at its −500 clamp for the whole turn. `S`
clears it, so a turn followed by `S` is safe, but a turn followed by anything else inherits
a saturated integrator. Fixed in the 2026-10-25 firmware (anti-windup now keys on
`bt_turn == 0`).

### Firmware verified on hardware — 2026-10-26
Uploaded over USB on **COM7** and checked end to end with the robot on USB power only.

Boot is correct, including the conditional motor enable added earlier:

```
Params loaded from EEPROM:     gyroBias = -1.00
Goc ban dau: 135.00
Robot dang NAM (135.0 do) - motor tat, dung len de chay
Bat dau!
```

`reset` returns every new default (`UPOFF 2.0`, `DRVVEL 20`, `BTIDLE 60`, `VMAX 40`,
`KPPOS 0.015`, `ARCH 1`, `KP 45`, `KD 2.5`), and `save` then `load` reads `UPOFF=2.0000`
back out of EEPROM.

Phone-app command set, all on hardware:

| sent | reply |
|---|---|
| `F` `F` `B` | `drive=0.40`, `0.80`, `0.40` |
| `L` `R` `S` | `turn=-100`, `turn=0`, `STOP` |
| `G` `I` `H` `J` | each produces **both** a drive and a turn |
| `3` / `9` | `app_speed=0.40` / `1.00` |
| `F` x6 at 50 ms | **six separate replies**, so the repeat-flush rule holds below `BTIDLE` |
| `BIAS 0.5`, `BIAS 0` | intact, never split |

The held-button row is the interesting one: it went `0.40 -> 0.60 -> 0.60 ...` and stopped,
because `app_speed` was 0.40 at the time and the ceiling is `DRVMAX 1.5 x 0.4 = 0.60`.

**Two traps hit during this session, both already documented and both ignored anyway:**

- Opening the port with `dtr=True` resets the board. On USB power alone the MPU is slow
  enough that `calibrateGyro()` takes **~57 s**, not the ~4.5 s it takes on battery, so
  every command sent in that window vanishes and the board looks dead. Open with
  `dtr=False` (the `open_no_reset()` pattern) unless you *want* the banner.
- While the board is blocked, buffered joystick characters arrive in one burst with no
  gaps between them and glue into `? FBLRSSGIHJ393S`. The timing rules assume the firmware
  is actually running. This is cosmetic, but it is why a stalled board produces a screen of
  `?` lines rather than silence.

### Still unmeasured
- **Push recovery.** Still the oldest open complaint ("tác động vào là trôi luôn"). With
  `kd 2.5` the damping ratio works out near ζ ≈ 0.17, which is exactly what "gets shoved and
  drifts" looks like. `kd` is a live command — sweep 2.5 / 6 / 10 against a repeatable push
  now that `gyro` is trustworthy.
- Steady-state drive speed against the new `DRVVEL` scaling, with pulses of 5 s or more.
- **Which sign of `DRVVEL` is forward.** Unknown until the new firmware runs; `DRVVEL`
  accepts negative values precisely so this costs a BT command, not an upload.
- A4988 Vref, still taken on faith from a code comment (0.6V).
- Vibration coupling with motors **running** while actually balancing.

## Tuning Notes

**Backward drift fix**: `target_bias` (BIAS command) compensates CG offset. The velocity
loop's `tgt` output tells you where it stabilizes — set BIAS to that value. Auto-detect via
`BIAS_HINT` handles most of it. In logs from 2026-05-12, `tgt` stabilizes at ~−0.4°
(the robot leans slightly backward at this CG position).

**Oscillation**: usually `Kp_angle` too high or `Kd_angle` too low. Reduce Kp first.

**KPV too high**: velocity loop integral diverges — `tgt` runs to `max_target_offset` and
the robot falls. Tested: 0.025 caused divergence. Safe range: 0.005–0.010.

**Filter drift** (robot visibly tilted but `angle` reads small, or a FALLEN event logged as
`filter drift!`): raise `alpf` toward 0.9 first, then lower `alpha` to ~0.95. Watch the
`drift` field in the log.

**Known-good baseline**: v3 config `Kp=80 Kd=12` with A4988 Vref 0.6V balanced ~25s and
recovered from ±13° disturbances.

## Build & Upload

No test suite and no linter. A clean `compile` is the verification step for firmware
changes; everything else is validated on hardware via the live-tuning commands.

```powershell
& "C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "d:\Project\robot_balance"
& "C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM8 --fqbn arduino:avr:uno "d:\Project\robot_balance"
```

**Do not trust a remembered COM number, and do not use `arduino-cli board list` to find
it.** That listing prints `Unknown` for every port on this machine, so it cannot tell the
Arduino from the seven Bluetooth virtual ports. Ask Windows instead:

```powershell
Get-PnpDevice -PresentOnly -Class Ports | Select-Object Status, FriendlyName
Get-PnpDevice | Where-Object { $_.InstanceId -like "USB\VID_1A86*" } |
    Select-Object Status, Present, FriendlyName        # CH340 specifically
```

The board is a **CH340 clone** (`VID_1A86&PID_7523`), so `USB-SERIAL CH340 (COMn)` is the
one to upload to. `Present: False` on it means the cable is not plugged in — the entry
survives in the registry as a ghost, with whatever COM number it last held, which is how
`COM8` ended up in these notes and in failed upload commands. On 2026-10-25 the ghost read
COM5.

Two traps on this machine specifically:

- **COM7 is claimed by two devices at once**, a Bluetooth virtual port and a `USB Serial
  Device` (`VID_0843&PID_5740`) that is *not* the Arduino. Seeing COM7 appear means
  nothing. This is what "thấy nhận COM7 mà không nhận CH340" was.
- A USB cable that charges but carries no data enumerates nothing at all, which looks
  identical to not being plugged in.

**Use `upload.py`, not `arduino-cli upload` directly.**

```powershell
python upload.py            # finds the board by VID:PID, then uploads
python upload.py --check    # just report the ports
```

It resolves the board by `VID:PID=0843:5740` instead of trusting a remembered
COM number, warns when that COM name is duplicated, gives each attempt a hard
timeout, and kills a stuck avrdude instead of waiting forever.

That last part matters because **opening a Bluetooth virtual COM port with
nothing connected blocks indefinitely** — no error, no timeout. On 2026-10-26
both avrdude and a plain `serial.Serial.open()` hung on COM7 and had to be
killed. Earlier the same day two uploads to COM7 succeeded, which made the
collision look harmless; it is not, and five consecutive attempts later that day
all hung. Do not treat a lucky upload as evidence the port is unambiguous.

**What actually cleared it, 2026-10-26, with no admin rights:**

1. Device Manager → Ports → right-click the Bluetooth port holding the duplicate
   number → **Disable device**. (Leave the one pointing at the robot's HC-05 alone.)
2. **Unplug the USB cable, wait two seconds, plug it back in.**

Both steps are needed. After the disable alone, avrdude stopped hanging but reported
`unable to open port COM7` — a disabled device keeps the name claimed until the board
re-enumerates and takes it. After the replug the upload went through on the first try.

Note that `serial.tools.list_ports` reads the registry, so the disabled entry still shows
up in any listing. A duplicate in the list is not by itself a problem; a duplicate that is
*enabled* is.

**The collision has to be removed, not retried around.** Settings → Bluetooth &
devices → More Bluetooth settings → COM Ports tab → Remove the *Incoming* port
holding the duplicate number. The incoming ports with an all-zero MAC
(`LOCALMFG&0000\...000000000000_*`) are leftovers and safe to delete. Keep the
outgoing one pointing at the robot's HC-05 (`...002106081CD4`).

`arduino-cli board list` is still fine once you know the number. The helper sketches are
separate Arduino projects — compile them by passing their folder
(`...\robot_balance\test_motor`) instead.

Watch the compile output for SRAM usage; the ATmega328P has 2KB and the firmware keeps all
strings in flash with `F()`. Keep new `Serial.print` literals wrapped in `F()`.

Unplug the 12V motor supply before uploading over USB.

## HC-05 Baud Change (one-time setup)

1. Upload `hc05_config\hc05_config.ino` while HC-05 is **not** in AT mode (normal power)
2. Unplug USB, hold KEY pin on HC-05, plug USB back → LED blinks slowly (AT mode)
3. Power cycle Arduino → sketch runs, sends `AT+UART=115200,0,0`, LED blinks 3×
4. Upload `robot_balance.ino` normally

`set_hc05_baud.py` does the same over USB serial and *can* read responses, but only works
when the HC-05 is reachable without the 16U2 fighting for D0 — the sketch route is the
reliable one.
