# LINE PID Tuning Guide

## How it works

The robot reads 5 IR sensors and computes a **position** (0–4000).  
Centre = 2000. Line left of centre → position < 2000. Line right → position > 2000.

**Error = 2000 − position**

The PID turns that error into a steering correction applied to both motors.

---

## Serial terminal controls (while in L mode)

| Key | Action |
|-----|--------|
| `+` | Speed +10 |
| `-` | Speed −10 |
| `SPC` | Pause motors |
| `L` | Resume |
| `Z` | Toggle safe mode (stops if line lost 3 s) |
| `X` | Clear SAFE latch |

---

## Bluetooth PID tuning (2-byte frames)

Send **two bytes** in sequence: `[cmd byte] [value byte]`

| Cmd byte | What it sets | Value range |
|----------|-------------|-------------|
| `\x01`   | Kp          | 0–255 |
| `\x02`   | multiP      | 1–9   |
| `\x03`   | Ki          | 0–255 |
| `\x04`   | multiI      | 1–9   |
| `\x05`   | Kd          | 0–255 |
| `\x06`   | multiD      | 1–9   |
| `\x07`   | 0=stop / 1=start | 0 or 1 |

**Effective gain = raw value ÷ 10^multi**

Examples:
- Kp=20, multiP=1 → effective Kp = 20 ÷ 10 = **2.0**
- Kp=5,  multiP=2 → effective Kp = 5  ÷ 100 = **0.05**

---

## What each term does

| Term | Effect | Too high |
|------|--------|----------|
| **Kp** | Steers toward the line | Robot oscillates / wiggles |
| **Ki** | Corrects steady drift | Slow oscillation builds up over time |
| **Kd** | Damps the correction | Jerky or twitchy motion |

---

## Tuning procedure (start here)

1. **Set all gains to zero.** Robot should drive straight at your chosen speed.
2. **Increase Kp only** until the robot follows the line but wiggles.
3. **Add Kd** (start small) until the wiggle settles down.
4. **Add Ki only if** the robot drifts consistently to one side on a straight.

### Suggested starting point

| Param | Value | multi | Effective |
|-------|-------|-------|-----------|
| Kp    | 15    | 1     | 1.5       |
| Ki    | 0     | 1     | 0         |
| Kd    | 10    | 1     | 1.0       |
| Speed | 150   | —     | (default) |

---

## All LINE_PID parameters

These are all defined at the top of `main.c`. Edit them to retune without Bluetooth.

| Define | Current | Original | Meaning |
|--------|---------|----------|---------|
| `LINE_PID_LOOP_MS` | 8 | 8 | Control loop period in ms |
| `LINE_PID_LFSPEED_DEFAULT` | 230 | 230 | Forward speed on startup (0–255 scale) |
| `LINE_PID_LFSPEED_MIN` | 10 | — | Minimum speed allowed via +/− keys |
| `LINE_PID_LFSPEED_MAX` | 255 | — | Maximum speed allowed via +/− keys |
| `LINE_PID_SPEED_STEP` | 10 | — | How much +/− keys change speed per press |
| `LINE_PID_CORRECTION_LIMIT` | 255 | 255 | Max PID steering correction per cycle |
| `LINE_PID_I_LIMIT` | 5000 | 5000 | Integral windup clamp |
| `LINE_PID_RECOVERY_SPEED` | 902 | 902 | Platform-scale spin speed when line is lost |

**Gain defaults** (reset every time you press `L` or power on):

| Variable | Default | Set via Bluetooth |
|----------|---------|-------------------|
| `g_pid_kp` | 0 | cmd `\x01` |
| `g_pid_mp` | 1 | cmd `\x02` |
| `g_pid_ki` | 0 | cmd `\x03` |
| `g_pid_mi` | 1 | cmd `\x04` |
| `g_pid_kd` | 0 | cmd `\x05` |
| `g_pid_md` | 1 | cmd `\x06` |

> **Original** = value when LINE PID mode was first implemented (before tuning).  
> `—` = parameter did not exist originally; added during tuning.

---

### Notes on key parameters

- **`LINE_PID_CORRECTION_LIMIT`** — most impactful for overshoot. At 50 with speed 100, the slower wheel never drops below 50 and never reverses. Raise if turns feel sluggish; lower if still overshooting.
- **`LINE_PID_LFSPEED_DEFAULT`** — lowered from 230 to 100 so sensors have more time to react. Raise slowly once the PID is tuned.
- **`LINE_PID_RECOVERY_SPEED`** — used for the off-line spin. If you change `LINE_PID_LFSPEED_DEFAULT`, update this to roughly `new_speed / 255 × 1000`.
