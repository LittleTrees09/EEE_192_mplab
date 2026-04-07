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
| IR Sensors | PA0-PA4 | Line detection (bit0=S1...bit4=S5) |

## License

Course project for EEE_192. Internal use only.

## Notes

- Motor commands are speed values from -1000 (reverse) to +1000 (forward)
- 100 µs tick provides accurate timing for debounce and control loops
- All timing-critical operations use platform-provided abstractions
- IR sensor polarity should be verified during commissioning and adjusted in main.c if needed
