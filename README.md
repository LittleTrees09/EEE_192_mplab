# EEE_192 Robot Firmware

Embedded firmware for an EEE_192 mobile robot using MPLAB XC32.

The project supports three runtime modes:
- Manual drive over serial
- IR line-following (5 digital sensors)
- Ultrasonic obstacle avoidance (front/left/right)

## Quick Start

1. Open the project in MPLAB X.
2. Build with XC32.
3. Flash the target board.
4. Open a serial terminal at `38400` baud.
5. Press the onboard button to enable controls.

## Runtime Controls

### Mode Selection

- `M`: Manual mode
- `U`: Auto infrared follow mode
- `O`: Auto ultrasonic avoid mode
- `SPACE`: Stop motors (and pause auto run)

### Manual Mode Keys

- `W`: Forward
- `S`: Backward
- `A`: Turn left
- `D`: Turn right
- `Q`: Approx. 90 deg left turn
- `E`: Approx. 90 deg right turn
- Up arrow: Increase speed
- Down arrow: Decrease speed

Notes:
- Motor command range is `-1000` to `+1000`.
- Manual movement has command timeout protection.
- Speed ramping is enabled for smoother acceleration/deceleration.

## IR Auto Mode

### Current Default Behavior

Firmware defaults in `main.c`:
- `IR_AUTO_POLICY = IR_POLICY_MOVE_IF_DETECT`
- `IR_ACTIVE_ON_BLACK_HIGH = 0` (black is interpreted as active-low by default)
- `IR_TURN_THRESHOLD_DEFAULT = 1`
- `IR_MIN_COUNT_DEFAULT = 1`
- `IR_NO_DETECT_CRAWL_ON_EMPTY = 1`
- `IR_SAMPLE_HISTORY_SIZE = 3`

Sensor order is left-to-right: `S1 S2 S3 S4 S5`.
Bit mapping is `bit0=S1 ... bit4=S5`.

### Decision Logic

Weighting:
- `S1 = -2`
- `S2 = -1`
- `S3 = 0`
- `S4 = +1`
- `S5 = +2`

Action selection:
- If detected black count is below minimum: `CRAWL` (slow forward)
- If weighted sum <= `-turn_threshold`: `LEFT`
- If weighted sum >= `+turn_threshold`: `RIGHT`
- Otherwise: `FORWARD`

### IR Live Tuning (in Auto IR mode)

- `I`: Toggle IR debug stream
- `1..4`: Set turn threshold (`1` reacts sooner, `4` reacts later)
- `N`: Decrease minimum black sensor count
- `B`: Increase minimum black sensor count
- `P`: Toggle black polarity interpretation

### IR Debug Output Format

When debug stream is enabled (`I`):

```text
IR: <raw_bits> B:<black_bits> Sum=<signed> Cnt=<count> <ACTION>
```

Example:

```text
IR: 11101 B:00010 Sum=-1 Cnt=1 LEFT
```

If tape is not being detected reliably, first tune the IR sensor module comparator/potentiometer, then verify polarity with `P`.

## Ultrasonic Auto Mode

Ultrasonic mode polls front/left/right distance and decides between:
- `FORWARD`
- `LEFT` / `RIGHT` (steering correction)
- `TURN_LEFT` / `TURN_RIGHT` (front blocked)
- `STOP` (sensor invalid or unsafe)

Default thresholds (in centimeters):
- Front stop: `< 25`
- Front caution: `< 40`
- Side close: `< 20`
- Side medium: `< 35`

Status is printed to serial in compact debug lines.

## Build

### Make

```bash
make -f Makefile
```

### Clean

```bash
make clean
```

### CMake/Ninja (project-generated layout)

This repository already includes generated CMake artifacts in `_build/` and `cmake/`.
If you are rebuilding manually, use the project's generated presets/files for your board configuration.

## Project Layout

```text
main.c               Application state machine and control loops
platform.c/.h        Hardware abstraction
gpio.c               GPIO and low-level motor/pin logic
usart.c              UART driver
systick.c            System timing
irq_handlers.c       Interrupt handlers
Makefile             Main build entry
cmake/               Generated CMake files
nbproject/           MPLAB project metadata
```

## Hardware Pin Summary

| Signal | Pin | Purpose |
|---|---|---|
| USART RX | PB08 | Serial input |
| USART TX | PB09 | Serial output |
| Button | PA23 | Control enable toggle (active LOW) |
| LED | PA15 | Status indicator |
| Motor STBY | PA07 | TB6612 standby/enable |
| IR Sensors | PA08, PA09, PA10, PA11, PA14 | 5-bit IR array input |

## Troubleshooting

- Robot ignores black tape:
  - Adjust IR module threshold potentiometer.
  - Toggle polarity with `P` in IR auto mode.
  - Lower minimum count with `N` during testing.
- Robot turns the opposite direction:
  - Check motor wiring/polarity and left-right mapping in the motor output layer.
- Auto mode seems paused:
  - `SPACE` stops and pauses auto execution; send `U` or `O` again to resume.

## License

Course project for EEE_192. Internal academic use.
