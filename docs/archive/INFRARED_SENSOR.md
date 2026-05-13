# Infrared Sensor Array

## Overview
The robot uses a linear array of 5 infrared sensors to detect a black line on the ground. The sensors work together to determine the robot's position relative to the line and guide steering commands.

## How It Works

### Sensor Array Layout
- **S1** (Left): Weighted value -2
- **S2** (Left-Center): Weighted value -1
- **S3** (Center): Weighted value 0
- **S4** (Right-Center): Weighted value +1
- **S5** (Right): Weighted value +2

### Detection Mechanism
Each sensor outputs a HIGH signal when it detects black (the line). When all sensors see white, all outputs are HIGH (idle state). The robot reads these signals and calculates a weighted sum to determine steering direction.

### Line Following Logic

**No Line Detected** → STOP
- All sensors read white, no steering needed

**Single Sensor Detecting** → Navigate
- Left sensors (S1, S2) → Steer LEFT
- Center sensor (S3) → Go FORWARD
- Right sensors (S4, S5) → Steer RIGHT

**Multiple Sensors Detecting** → Follow the Center
- Edge detection (S1+S2 or S4+S5) → Steer toward the line
- Center line (S2+S3+S4) → Go FORWARD
- All sensors detecting → STOP (off the line)

### Steering Calculation
The robot calculates a weighted sum of all detected sensors:

- **Sum ≤ -2**: Turn LEFT (more left sensors active)
- **Sum -1 to +1**: Go FORWARD (center detected)
- **Sum ≥ +2**: Turn RIGHT (more right sensors active)

### Motor Control
Based on the steering decision:
- **FORWARD**: Both motors at full speed
- **LEFT**: Left motor full speed, right motor half speed
- **RIGHT**: Right motor full speed, left motor half speed
- **STOP**: Coast to halt
