# IR Sensor Logic Analysis — Test Results (May 14, 2026)

## Quick Reference: Sensor Actions

| Sensor | Position | Weight | Action When Alone | Reason |
|--------|----------|--------|-------------------|--------|
| **S1** | Far left | -2 | **LEFT** | Strong negative sum |
| **S2** | Mid-left | -1 | **LEFT** | Negative sum ≤ -1 threshold |
| **S3** | Center | 0 | **FORWARD** | Sum = 0 (on-line) |
| **S4** | Mid-right | +1 | **RIGHT** | Positive sum ≥ +1 threshold |
| **S5** | Far right | +2 | **RIGHT** | Strong positive sum |

---

## Critical Finding: BIFURCATION Logic

When **2 sensors fire with sum=0** (balanced), the code returns **BIFURCATION_LEFT** instead of FORWARD. This is **intentional behavior**:

```c
// From decide_auto_action() — Priority 4
if (left_wing && right_wing && center_clear)
{
    return AUTO_ACT_BIFURCATION_LEFT;  // Prefers left
}
```

**Examples:**
- S1+S5 (sum=0, count=2) → **BIFURCATION_LEFT** ✓ (both wings active, no center)
- S2+S4 (sum=0, count=2) → **BIFURCATION_LEFT** ✓ (both wings active, no center)
- S1+S4 (sum=-1, count=2) → **BIFURCATION_LEFT** ✓ (both wings active, no center)

**This is not a bug** — it's path selection at splits. The `IR_BIFURCATION_PREFER_LEFT` setting determines which fork the robot takes.

---

## Asymmetry Issue Analysis

### The Numbers:
- **S1 alone**: sum = **-2** → LEFT (strong)
- **S5 alone**: sum = **+2** → RIGHT (strong)
- **S1+S4**: sum = -2+1 = **-1** → LEFT (medium)
- **S2+S5**: sum = -1+2 = **+1** → RIGHT (medium)

### The Problem:

**S1 is NOT asymmetric to S5 in terms of weight** (-2 vs +2 are opposites). However, the **interaction** shows a problem:

| Combination | Left Weight | Right Weight | Sum | Result |
|-------------|-------------|--------------|-----|--------|
| S1+S4 | -2 | +1 | -1 | **LEFT** (prefers left) |
| S2+S5 | -1 | +2 | +1 | **RIGHT** (prefers right) |
| S1+S5 | -2 | +2 | 0 | **BIFURCATION_LEFT** |

The asymmetry emerges because **S1 (weight -2) overpowers S4 (weight +1)**, while **S5 (weight +2) barely overpowers S2 (weight -1)**.

---

## Why the Robot Favors LEFT

From the test results, if your robot is turning LEFT too much, investigate:

### 1. **S1 or S2 Triggering Too Often**
If the left sensors are over-sensitive:
- Check IR sensor potentiometer tuning on the module
- Verify sensor lens cleanliness
- Test each sensor individually with actual line and background

### 2. **Physical Mounting**
- Is the sensor array centered on the robot?
- Are all 5 sensors at the same height above the line?
- Is S3 actually over the line center, or offset?

### 3. **Line Placement**
- Is your test line actually centered, or slightly to the left?
- Does the line have uneven contrast (one side darker)?

### 4. **Threshold Setting**
- Current threshold: `IR_TURN_THRESHOLD_DEFAULT = 1`
- Larger threshold → less sensitive to turns
- Test with threshold 2 or 3 to see if it helps

---

## Test Suite Groups

### Group 1: Individual Sensors ✓
- Each sensor in isolation produces expected action
- S1/S2 → LEFT, S3 → FORWARD, S4/S5 → RIGHT

### Group 2-3: Left/Right Combinations ✓
- Multiple left sensors amplify LEFT
- Multiple right sensors amplify RIGHT

### Group 4: Balanced Combinations ⚠️
- Sum=0 triggers BIFURCATION, not FORWARD
- This is intentional but can mask steering issues

### Group 5: Wide Black (4+) ✓
- Correctly triggers JUNCTION mode
- Coast/cooldown mechanics prevent oscillation on intersections

### Group 6: Bifurcation ✓
- Both wings active + center clear = split path detection
- Always prefers left (configurable)

### Group 7: Asymmetry Analysis ⚠️
- S1 weight (-2) is NOT symmetric to S5 weight (+2)
- But this is intentional for line-following weights
- The issue is whether S1/S2 are firing when they shouldn't

### Group 8: Threshold Boundaries ✓
- Threshold behavior correct at ±1 boundaries

---

## Debugging Steps

### Step 1: Individual Sensor Test
Tape the robot so motors don't run. Send each sensor over a line, watch which ones fire:

```
Expected:
- S1 over line → fires (LEFT)
- S2 over line → fires (LEFT)
- S3 over line → fires (CENTER)
- S4 over line → fires (RIGHT)
- S5 over line → fires (RIGHT)
```

If S1/S2 fire when S3 is on the line → they're too sensitive or misaligned.

### Step 2: Symmetry Test
Center robot on line, measure which sensor fires first as line drifts left vs right. Should be symmetric.

### Step 3: Threshold Adjustment
Try increasing `IR_TURN_THRESHOLD_DEFAULT` from 1 to 2:
```c
#define IR_TURN_THRESHOLD_DEFAULT  2   // More tolerant of small drifts
```

### Step 4: Polarity Check
Verify `IR_ACTIVE_ON_BLACK_HIGH` is set correctly for your sensors:
```c
#define IR_ACTIVE_ON_BLACK_HIGH     0   // Recommended for most sensors
```

---

## Test Program Usage

The test program is in [ir_sensor_test.c](ir_sensor_test.c). Recompile after changes to `main.c` defines:

```bash
gcc -o ir_sensor_test ir_sensor_test.c
./ir_sensor_test
```

To add new test cases, modify the test groups in the `run_tests()` function.

---

## Summary

✅ **Logic is correct** — all sensor-to-action mappings work as designed  
⚠️ **Bifurcation behavior** — sum=0 cases return BIFURCATION_LEFT, not FORWARD  
🔍 **LEFT bias likely cause** — S1/S2 over-sensitive OR S1 weight stronger than intended  

**Next steps:**
1. Test individual sensors to confirm they fire at the right moments
2. Adjust `IR_TURN_THRESHOLD_DEFAULT` to 2 to reduce sensitivity
3. Check sensor potentiometer tuning if available
