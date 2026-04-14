# Expected IR Debug Stream Output

## New Behavior Summary
- **No sensors detecting** → STOP
- **Any single sensor detecting** → steer LEFT/RIGHT or go FWD based on position
- **All 5 sensors detecting** → STOP (treats full coverage as off-the-line)

## Format
```
IR: raw=0x[HEX] black=0x[HEX] sum=[VAL] cnt=[CNT] act=[ACTION]
```

---

## Scenario 1: Nothing Detected
**GPIO State:** All pins HIGH (no black detected)
```
IR: raw=0x1F black=0x00 sum=0 cnt=0 act=STOP
```
**Motor Output:** STOP (coast to halt)

---

## Scenario 2: Only Left Sensor (S1) Detects Black
**GPIO State:** S1 pin LOW, others HIGH
```
IR: raw=0x1E black=0x01 sum=-2 cnt=1 act=LEFT
```
**Motor Output:** Left motor at full speed, right motor at half speed (steer left)

---

## Scenario 3: Only Left-Center (S2) Detects Black
**GPIO State:** S2 pin LOW, others HIGH
```
IR: raw=0x1D black=0x02 sum=-1 cnt=1 act=FWD
```
**Motor Output:** Both motors at full speed (forward)
*Note: sum=-1 is not <= -2 (turn threshold default), so it's treated as center*

---

## Scenario 4: Only Center (S3) Detects Black
**GPIO State:** S3 pin LOW, others HIGH
```
IR: raw=0x17 black=0x08 sum=0 cnt=1 act=FWD
```
**Motor Output:** Both motors at full speed (forward)

---

## Scenario 5: Only Right-Center (S4) Detects Black
**GPIO State:** S4 pin LOW, others HIGH
```
IR: raw=0x0F black=0x10 sum=1 cnt=1 act=FWD
```
**Motor Output:** Both motors at full speed (forward)
*Note: sum=1 is not >= 2 (turn threshold), so it's treated as center*

---

## Scenario 6: Only Right Sensor (S5) Detects Black
**GPIO State:** S5 pin LOW, others HIGH
```
IR: raw=0x0E black=0x10 sum=2 cnt=1 act=RIGHT
```
**Motor Output:** Right motor at full speed, left motor at half speed (steer right)

---

## Scenario 7: Left Edge (S1 + S2) Detects Black
**GPIO State:** S1, S2 pins LOW, others HIGH
```
IR: raw=0x1C black=0x03 sum=-3 cnt=2 act=LEFT
```
**Motor Output:** Left motor at full speed, right motor at half speed (steer left)

---

## Scenario 8: Center Line (S2 + S3 + S4) Detects Black
**GPIO State:** S2, S3, S4 pins LOW, others HIGH
```
IR: raw=0x08 black=0x1C sum=0 cnt=3 act=FWD
```
**Motor Output:** Both motors at full speed (forward)

---

## Scenario 9: Right Edge (S4 + S5) Detects Black
**GPIO State:** S4, S5 pins LOW, others HIGH
```
IR: raw=0x06 black=0x18 sum=3 cnt=2 act=RIGHT
```
**Motor Output:** Right motor at full speed, left motor at half speed (steer right)

---

## Scenario 10: All 5 Sensors Detecting Black
**GPIO State:** All pins LOW
```
IR: raw=0x00 black=0x1F sum=0 cnt=5 act=STOP
```
**Motor Output:** STOP (treats as off-the-line)

---

## Key Rules
1. **Turn Threshold** (default=2): determines how asymmetric the black detection must be to trigger a turn
   - sum ≤ -2: turn LEFT
   - sum ≥ 2: turn RIGHT
   - -1 to 1: go FORWARD

2. **Sensor Weighting:**
   - S1 (leftmost): -2
   - S2: -1
   - S3 (center): 0
   - S4: +1
   - S5 (rightmost): +2

3. **Hardware:**
   - Sensors pull GPIO LOW when they see black (open-collector output)
   - `ir_mask_on_black()` flips the bits so black=1 in the output
