# EEE_192 Line-Following Robot

A microcontroller-based line-following robot built with MPLAB X IDE. The project implements dual-motor control, IR line-sensor array integration, and command processing for both manual and autonomous line-following modes.

## Features

- **Dual Motor Control**: TB6612 motor driver interface with independent left/right motor speed control
- **IR Line Sensor Array**: 5-sensor line detection with configurable polarity and tuning
- **Dual Drive Modes**: 
  - Manual mode: Direct speed control via serial commands
  - Auto line-following mode: Autonomous line detection and steering
- **Serial Communication**: USART interface for command reception and feedback
- **Debouncing & Timeouts**: Button debounce logic and command repeat support
- **Real-time Timebase**: 100 µs tick system for precise timing

## Hardware Platform

- **Microcontroller**: ARM Cortex-M0+ (SAM/ATSAMD series)
- **Motor Driver**: TB6612FNG (dual H-bridge)
- **Sensors**: 5-element IR line sensor array
- **Communication**: USART (SERCOM3 on PB08/PB09)
- **Control I/O**: 
  - Button (PA23, active LOW)
  - LED indicator (PA15)
  - Motor enable (PA07, STBY)

## Project Structure

```
├── main.c/h              # Main application loop and state machine
├── platform.c/h          # Hardware abstraction layer (HAL)
├── gpio.c                # GPIO initialization and control
├── i2c.c                 # I2C interface (if used)
├── usart.c               # Serial communication driver
├── systick.c             # System tick timer setup
├── irq_handlers.c        # Interrupt service routines
├── Makefile              # Build configuration
├── cmake/                # CMake build files
├── nbproject/            # MPLAB project configuration
└── build/                # Build output directory
```

## Building the Project

### Prerequisites
- MPLAB X IDE v6.x or higher
- XC32 C/C++ Compiler
- Make or CMake build system

### Build Commands

```bash
# Using Make
make -f Makefile

# Using CMake
mkdir -p _build
cd _build
cmake ..
ninja
```

### Clean Build
```bash
make clean
```

## Configuration & Tuning

All tunable parameters are defined in `main.c`:

### Speed Control
- `DEFAULT_SPEED_CMD`: Default motor speed (0-1000)
- `SPEED_STEP_CMD`: Speed increment step size
- `MAX_SPEED_CMD`: Maximum speed limit

### Line Following (Auto Mode)
- `SOFT_TURN_DIV`: Controls turn aggressiveness (2=stronger, 3=softer)
- `MIN_TURN_SPEED_CMD`: Minimum speed limit during turning
- `TURN90_SPEED`: Speed for 90° turns
- `TURN90_MS`: Duration of 90° turn

### IR Sensor
- `IR_ACTIVE_ON_BLACK_HIGH`: Polarity setting (0=LOW on black, 1=HIGH on black)
- `IR_MASK_S1` to `IR_MASK_S5`: Individual sensor bit masks
- `IR_SAMPLE_HISTORY_SIZE`: Short history window used to keep weak tape hits from flickering out immediately

### Timing
- `DEBOUNCE_MS`: Button debounce interval
- `AUTO_LOOP_MS`: Line-following loop period
- `CMD_TIMEOUT_FIRST_MS`: Initial command repeat delay
- `CMD_TIMEOUT_REPEAT_MS`: Command repeat interval

## API

### Platform Initialization
```c
void platform_initialization(void);        // Initialize all peripherals
void platform_gpio_init(void);              // Setup GPIO pins
void platform_ir_init(void);                // Initialize IR sensors
void platform_usart_init(void);             // Initialize UART/USB
void platform_systick_init(void);           // Start 100 µs timebase
```

### Motor Control
```c
void platform_motor_set(int16_t left, int16_t right);  // -1000..+1000
void platform_motor_stop(void);                         // Stop both motors
void platform_tb6612_enable(bool en);                   // Enable/disable driver
```

### Sensor Input
```c
uint8_t platform_ir_read_mask_raw(void);    // Read 5-bit sensor mask
bool platform_button_pressed(void);          // Read button state
```

### Serial I/O
```c
void platform_usart_write_str(const char *s);           // Send string
void platform_usart_write_buf(const char *buf, uint32_t n);
bool platform_usart_read_char(char *out);               // Non-blocking read
```

### Timing
```c
uint32_t platform_millis(void);              // Get milliseconds since boot
```

## Usage Modes

### Manual Mode
Send speed commands via serial (0-1000 for each motor). Speed is adjusted in steps, and commands repeat if held.

### Auto Line-Following Mode
Robot automatically:
1. Scans IR sensor array for line position
2. Adjusts left/right motor speeds proportionally
3. Makes coordinated turns when lines are detected
4. Maintains center alignment on the line

#### Current 5-Sensor IR Algorithm (S1..S5)

The current firmware uses the following defaults from `main.c`:
- `IR_AUTO_POLICY = IR_POLICY_MOVE_IF_DETECT`
- `IR_ACTIVE_ON_BLACK_HIGH = 0` (active-low detection)
- `IR_TURN_THRESHOLD_DEFAULT = 1`
- `IR_MIN_COUNT_DEFAULT = 1`
- `IR_NO_DETECT_CRAWL_ON_EMPTY = 1`

Sensor ordering (left to right):
- S1 = leftmost
- S2 = left-center
- S3 = center
- S4 = right-center
- S5 = rightmost

Bit mapping in the mask is `bit0=S1` ... `bit4=S5`.

If the robot does not react to standard black electrical tape, the main causes are usually hardware threshold or polarity, not a lack of “shade” detection in software. The IR sensors are read as digital inputs, so the module comparator must be adjusted to switch cleanly on your track surface.

Recommended tuning steps:
1. Adjust the sensor module potentiometer so the LEDs or digital outputs change state when the sensor passes over the tape.
2. Try the runtime polarity toggle in auto mode with `P` if the module is inverted relative to the firmware default.
3. Lower the count threshold with `N` and keep the turn threshold at `1` while testing on the tape.
4. Use the debug stream with `I` to confirm whether the tape is producing any black bits at all.
5. If the output still flickers, raise contrast on the track or lower the sensor height slightly.

Decision weighting used by the controller:
- S1 contributes `-2`
- S2 contributes `-1`
- S3 contributes `0`
- S4 contributes `+1`
- S5 contributes `+2`

Action selection:
- `black_mask == 0` -> `CRAWL` (slow forward search)
- `black_mask == 0x1F` (all sensors detect) -> `STOP`
- `sum <= -threshold` -> `LEFT`
- `sum >= +threshold` -> `RIGHT`
- otherwise -> `FORWARD`

Runtime IR tuning commands in auto mode:
- `1` to `4` change how quickly the robot commits to turning.
- `N` lowers the minimum number of black sensors required.
- `B` raises the minimum number of black sensors required.
- `P` toggles the black polarity interpretation.
- `I` toggles the live debug stream.

Example patterns (left to right shown as `S1 S2 S3 S4 S5`):

| Pattern | Interpretation | Action |
|---|---|---|
| `0 0 0 0 0` | no line seen | CRAWL |
| `1 0 0 0 0` | far-left hit | LEFT |
| `0 1 0 0 0` | left-center hit | LEFT |
| `0 0 1 0 0` | centered hit | FORWARD |
| `0 0 0 1 0` | right-center hit | RIGHT |
| `0 0 0 0 1` | far-right hit | RIGHT |
| `0 1 1 1 0` | centered cluster | FORWARD |
| `1 1 1 1 1` | full-array hit/intersection | STOP |

## Building & Debugging

- **Debug Output**: USART3 (115200 baud) provides real-time status
- **LED Indicator**: Via PA15 for visual feedback
- **Button**: Manual mode trigger/control on PA23

## Hardware Connections

| Signal | Pin | Purpose |
|--------|-----|---------|
| USART RX | PB08 | Serial input |
| USART TX | PB09 | Serial output |
| Button | PA23 | Mode select (active LOW) |
| LED | PA15 | Status indicator |
| Motor STBY | PA07 | TB6612 enable (active HIGH) |
| IR Sensors | PA08, PA09, PA10, PA11, PA14 | Line detection (bit0=S1...bit4=S5) |

## License

Course project for EEE_192. Internal use only.

## Notes

- Motor commands are speed values from -1000 (reverse) to +1000 (forward)
- 100 µs tick provides accurate timing for debounce and control loops
- All timing-critical operations use platform-provided abstractions
- IR sensor polarity should be verified during commissioning and adjusted in main.c if needed
