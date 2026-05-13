# Expected IR Debug Stream Output

## New Behavior Summary
- **No sensors detecting** → STOP
- **Any single sensor detecting** → steer LEFT/RIGHT or go FWD based on position
- **All 5 sensors detecting** → STOP (treats full coverage as off-the-line)

## Format
```
[IR] Raw=0x[HEX] Det=0x[HEX] Sum=[VAL] Cnt=[CNT] Act=[ACTION]
```

---

## Scenario 1: Nothing Detected
**GPIO State:** All pins HIGH (no sensor active)
```
[IR] Raw=0x1F Det=0x00 Sum=0 Cnt=0 Act=STOP
```
**Motor Output:** STOP (coast to halt)

---

## Scenario 2: Only Left Sensor (S1) Detects
**GPIO State:** S1 pin LOW, others HIGH
```
[IR] Raw=0x1E Det=0x01 Sum=-2 Cnt=1 Act=LEFT
```
**Motor Output:** Left motor at full speed, right motor at half speed (steer left)

---

## Scenario 3: Only Left-Center (S2) Detects
**GPIO State:** S2 pin LOW, others HIGH
```
[IR] Raw=0x1D Det=0x02 Sum=-1 Cnt=1 Act=FWD
```
**Motor Output:** Both motors at full speed (forward)
*Note: sum=-1 is not ≤ -2 (turn threshold), so treated as center*

---

## Scenario 4: Only Center (S3) Detects
**GPIO State:** S3 pin LOW, others HIGH
```
[IR] Raw=0x17 Det=0x08 Sum=0 Cnt=1 Act=FWD
```
**Motor Output:** Both motors at full speed (forward)

---

## Scenario 5: Only Right-Center (S4) Detects
**GPIO State:** S4 pin LOW, others HIGH
```
[IR] Raw=0x0F Det=0x10 Sum=+1 Cnt=1 Act=FWD
```
**Motor Output:** Both motors at full speed (forward)
*Note: sum=+1 is not ≥ 2 (turn threshold), so treated as center*

---

## Scenario 6: Only Right Sensor (S5) Detects
**GPIO State:** S5 pin LOW, others HIGH
```
[IR] Raw=0x0E Det=0x10 Sum=+2 Cnt=1 Act=RIGHT
```
**Motor Output:** Right motor at full speed, left motor at half speed (steer right)

---

## Scenario 7: Left Edge (S1 + S2) Detects
**GPIO State:** S1, S2 pins LOW, others HIGH
```
[IR] Raw=0x1C Det=0x03 Sum=-3 Cnt=2 Act=LEFT
```
**Motor Output:** Left motor at full speed, right motor at half speed (steer left)

---

## Scenario 8: Center Line (S2 + S3 + S4) Detects
**GPIO State:** S2, S3, S4 pins LOW, others HIGH
```
[IR] Raw=0x08 Det=0x1C Sum=0 Cnt=3 Act=FWD
```
**Motor Output:** Both motors at full speed (forward)

---

## Scenario 9: Right Edge (S4 + S5) Detects
**GPIO State:** S4, S5 pins LOW, others HIGH
```
[IR] Raw=0x06 Det=0x18 Sum=+3 Cnt=2 Act=RIGHT
```
**Motor Output:** Right motor at full speed, left motor at half speed (steer right)

---

## Scenario 10: All 5 Sensors Detecting
**GPIO State:** All pins LOW
```
[IR] Raw=0x00 Det=0x1F Sum=0 Cnt=5 Act=STOP
```
**Motor Output:** STOP (treats as off-the-line)

---

## Key Notes
1. **Polarity:** `IR_ACTIVE_ON_BLACK_HIGH = 1u` means sensors output HIGH when detecting (finger/black)
2. **Turn Threshold** (default=2): determines steering sensitivity
   - Sum ≤ -2: turn LEFT
   - Sum ≥ +2: turn RIGHT
   - -1 to +1: go FORWARD
3. **Sensor Weighting:**
   - S1 (leftmost): -2
   - S2: -1
   - S3 (center): 0
   - S4: +1
   - S5 (rightmost): +2
