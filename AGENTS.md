# AGENTS.md

This file provides guidance to Codex (claude.ai/code) when working with code in this repository.

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
set_hc05_baud.py        — One-shot script: change HC-05 baud rate via AT mode
hc05_config\            — Arduino sketch to blindly send AT+UART=115200
passthrough\            — Minimal Serial passthrough sketch (rarely needed)
test_motor\             — Motor test sketch
test_imu\               — IMU test sketch
logs\                   — Session logs written by robot_control.py
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
Timer1 (prescaler 8 → 2MHz, 0.5µs/tick) runs two independent compare channels: OCR1A for
motor L, OCR1B for motor R. Each ISR emits one step pulse and reschedules itself at
`OCR + period`, with a slip guard (`< 4` ticks → push to `TCNT1 + 4`) so a missed compare
doesn't wrap into a 32ms stall. `period = 2000000 / steps_per_sec`.
`ZERO_SPEED` (32000) is the "stopped" sentinel — the ISR still fires but emits no pulse.
Speeds below `MIN_STEP_RATE` (30) are treated as stopped. No blocking in the main loop.

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
(BIAS/SAVE/LOAD/RESET) collided with the motion commands. Keep motion commands newline-
terminated on the Python side.

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

Values outside the range are clamped by `constrain()`, not rejected. `reset` restores
exactly this column. The in-app help text in `robot_control.py` still shows a couple of
superseded defaults (`alpf 0.70`, `acc 1200`) — this table and the firmware are authoritative.

Compile-time only (no live command): `BT_DRIVE_STEP` (0.4°), `BT_DRIVE_MAX`
(1.5°), `BT_TURN_STEP` (100 step/s), `BT_TURN_MAX` (300),
`SOFT_START_LOOPS`, `MICROSTEP`, `LOOP_PERIOD_US`, `VEL_LOOP_DIVIDER`. Everything else is
now a live BT command — see **Runtime-tunable** below.

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

### STARTUP_PARAMS
Sent automatically after the Arduino prints `"Bat dau!"` (detected via the
`_pending_startup` flag, sent from a worker thread with 120ms spacing). This **overrides
EEPROM on every run** — edit this list instead of recompiling firmware for quick tuning:

```python
STARTUP_PARAMS = [
    "ACC 350\n",     # must be re-sent: EEPROM may still hold the old 1500
    "SPD 7000\n",
    "ALPHA 0.97\n", "ALPF 0.85\n", "VFA 0.7\n",
    "KPV 0.008\n",  "KIV 0.001\n", "KD 12\n",
]
```

`ACC`/`SPD` are listed explicitly because `loadParams()` may restore stale EEPROM values at
boot; the startup burst overwrites them. No `BIAS` is sent — see `BIAS_HINT` above.

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
```

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

### Power under motor load — still failing, 2026-10-22
Resoldering the pin headers fixed the **idle** case completely (0 resets in 25s). Under real
motor load it still browns out: **5 boot banners in a 75s balancing run**, 2 of them visible
as backwards `ms` jumps inside the telemetry.

This is the top blocker. Nothing else can be evaluated honestly while it resets every ~15s —
and worse, **every reset re-runs `calibrate()`, redefining "vertical" as whatever angle the
robot happens to be lying at.**

Unverified and worth checking first: `HUONG_DAN.md` calls a 100–470µF electrolytic across
the CNC shield's VIN terminal mandatory, and it has never been confirmed present. Failing
that, bypass the Uno's regulator entirely with a 12V→5V buck feeding the `5V` pin.

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

### Known gap: no position loop
The velocity loop nulls **velocity**, it does not return the robot to where it started. Push
the robot and it rolls away, holding itself upright the whole time, and stops wherever the
velocity reaches zero. B-ROBOT has an optional position loop for this (`KP_POSITION 0.06`,
`KD_POSITION 0.45`) fed from the step counters; this firmware has none. Adding it is a
firmware change, so it pairs naturally with the `ZERO_SPEED` deadband fix below.

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

### Still unmeasured
- Vibration coupling with motors **running** — needs a capture while balancing, or an
  open-loop one driven through `L`/`R` (`bt_turn` bypasses the PID entirely).
- Oscillation frequency during a real balance attempt. The old 0.5s transcript aliased it;
  that is what the raw `.tsv` plus `analyze_log.py` exist to answer.
- A4988 Vref, still taken on faith from a code comment (0.6V).

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

COM8 = Arduino USB port (verify with `arduino-cli board list`). The helper sketches are
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
