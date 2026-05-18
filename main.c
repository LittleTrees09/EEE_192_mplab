// DO NOT REMOVE THIS COMMENT
// Microcontroller Unit: PIC32CM5164LS00064

#include "platform.h"
#include "robot.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS               150u
#define BUTTON_OFF_HOLD_MS        350u
#define AUTO_LOOP_MS              8u
#define CMD_TIMEOUT_MS            250u

#define DEFAULT_SPEED_CMD         400 //default 180
#define BUTTON_ON_SPEED_CMD       400 //default 180
#define MIN_SPEED_CMD             0
#define MAX_SPEED_CMD             2000 //default 1000
#define SPEED_STEP_CMD            200 //default 100

// ===== MANUAL CONTROL PARAMETERS =====
#define TURN_SENSITIVITY_PERCENT  100
#define TURN_INNER_PERCENT        0

#define RAMP_ENABLED              0
#define RAMP_STEP_PER_CYCLE       50
#define RAMP_CYCLE_MS             20

#define TURN_MODE_GENTLE_ARC      0 //default 0
#define TURN_MODE_PIVOT           1 //default 1

#if TURN_MODE_GENTLE_ARC
  #define TURN_OUTER_SPEED_PERCENT  100
  #define TURN_INNER_SPEED_PERCENT  30
#endif

#define IR_MASK_S1                (1u << 0)
#define IR_MASK_S2                (1u << 1)
#define IR_MASK_S3                (1u << 2)
#define IR_MASK_S4                (1u << 3)
#define IR_MASK_S5                (1u << 4)
#define IR_MASK_ALL               (IR_MASK_S1|IR_MASK_S2|IR_MASK_S3|IR_MASK_S4|IR_MASK_S5)

// ===== ULTRASONIC MAZE PARAMETERS =====
//
// Sensor state encoding:
//   0 = open path  — cm > ULTRA_OPEN_CM or read failed (no wall in range)
//   1 = wall clear — wall detected at an acceptable distance
//   2 = wall alert — wall at or below ULTRA_ALERT_CM (requires hysteresis)
//
// Calibration notes:
//   ULTRA_OPEN_CM     : set to roughly the corridor half-width. If the robot
//                       can "see" the far wall it will treat the side as closed.
//   ULTRA_ALERT_CM    : how close is "too close". Start at ~5 cm.
//   ULTRA_TURN_90_MS  : test on your floor, adjust in 50 ms steps.
//   ULTRA_TURN_180_MS : should be approximately 2 × ULTRA_TURN_90_MS.
//
// Track width is ~29 cm, so each side wall sits ~6–14 cm from the robot when
// centred. ULTRA_OPEN_CM is set above the far side-wall distance so the front
// sensor still triggers correctly at a T-junction.
//
// Timing budget (one poll cycle, no blocking delays between sensors):
//   ultra_read_stable_cm = 1 ping + ULTRA_SAMPLE_GAP_MS + 1 ping + gap + 1 ping
//                        = 3 pings × ~4 ms + 2 × ULTRA_SAMPLE_GAP_MS
//   3 sensors total      ≈ 3 × (12 ms + 2×5 ms) = ~66 ms worst-case
//   ULTRA_POLL_MS must be >= that; 30 ms gives the main loop a chance to breathe
//   because reads are blocking; actual effective rate ≈ 1 / (3×22 ms) ≈ 15 Hz.
//
#define ULTRA_POLL_MS            30u  // poll gate — reads themselves take ~22 ms each
#define ULTRA_OPEN_CM            16u  // >14 cm = open corridor; front triggers at ~7 cm
#define ULTRA_ALERT_CM            5u  // front wall "too close" threshold → trigger turn
#define ULTRA_FORWARD_SPEED       170 //default 170
#define ULTRA_TURN_SPEED          270 //default 220
#define ULTRA_TURN_90_MS          360u //default 420
#define ULTRA_TURN_180_MS         720u
#define ULTRA_REVERSE_MS          180u
#define ULTRA_ALERT_COUNT_THRESH  2u  // 2 consecutive alerts before state=2 (was 5 — too slow)
#define ULTRA_READ_SAMPLES        3u
#define ULTRA_SAMPLE_GAP_MS        5u // gap between the 3 pings inside one stable read (was 15)
#define ULTRA_TURN_COOLDOWN       10u // cycles of forced-forward after a turn (at ~15 Hz ≈ 650 ms)
#define ULTRA_STOP_BEFORE_TURN_MS 80u // brief stop before each turn (was 120)
#define ULTRA_NO_PATH_LIMIT       50u // consecutive all-blocked reads before 180 turn
#define ULTRA_MAX_VALID_CM       300u // readings >= this are treated as no echo
#define NO_ECHO_VALUE            999u // value returned by platform_ultrasonic_read_cm() on failure

// Double-wall centering: valid side reading range.
// Track is ~29 cm wide; robot centred → each side wall ≈ 6–14 cm away.
#define ULTRA_SIDE_MIN_CM          3u  // below this → too close, saturated reading
#define ULTRA_SIDE_MAX_CM         18u  // above this → wall is out of corridor range

#define ULTRA_CENTER_KP            18
#define ULTRA_CENTER_KI            0
#define ULTRA_CENTER_KD            8

#define ULTRA_CENTER_I_LIMIT       80
#define ULTRA_CENTER_CORR_LIMIT    90

#define ULTRA_LEFT_TRIM            0
#define ULTRA_RIGHT_TRIM           0

// ===== SAFE MODE PARAMETERS =====
#define SAFE_COMM_LOSS_MS         20000u
#define SAFE_ULTRA_FAIL_LIMIT     6u
#define SAFE_CLEAR_KEY            'x'
#define SAFE_IR_CRAWL_MS          3000u

#define IR_TURN_THRESHOLD_MIN      1
#define IR_TURN_THRESHOLD_MAX      3
#define IR_TURN_THRESHOLD_DEFAULT  1

#define IR_MIN_COUNT_MIN           1u
#define IR_MIN_COUNT_MAX           5u
#define IR_MIN_COUNT_DEFAULT       1u

#define IR_POLICY_MOVE_IF_NONE      0u
#define IR_POLICY_MOVE_IF_DETECT    1u
#define IR_AUTO_POLICY              IR_POLICY_MOVE_IF_DETECT

#define IR_ACTIVE_ON_BLACK_HIGH     0u
#define IR_SAMPLE_HISTORY_SIZE      2u

// =============================================================================
// PROPORTIONAL STEERING PARAMETERS  (no PID)
//
// outer motor = base_speed * IR_OUTER_PERCENT / 100
// inner motor = base_speed * max(IR_MIN_INNER_PERCENT,
//                 IR_OUTER_PERCENT - |error| * IR_STEER_STEP_PERCENT) / 100
//
// Speed table at IR_STEER_STEP_PERCENT=18, IR_OUTER_PERCENT=100:
//   error 0 -> inner 100%  (straight)
//   error 1 -> inner  82%  (gentle)
//   error 2 -> inner  64%  (medium)
//   error 3 -> inner  46%  (sharp)
//   error 4 -> inner  28%  (very sharp)
// =============================================================================
#define IR_OUTER_PERCENT            100
#define IR_STEER_STEP_PERCENT        18
#define IR_MIN_INNER_PERCENT         15

#define IR_CRAWL_SPEED_PERCENT       45
#define IR_CRAWL_MIN_SPEED           55   // guarantees PWM duty >= 1 tick
#define IR_CRAWL_STEER_STEP          10

// =============================================================================
// PID STEERING PARAMETERS
//
// This replaces the proportional-only steering in ir_compute_steer().
// The controller is kept inside main.c so the firmware remains self-contained.
// PID output is clamped to avoid sudden current spikes and brownout-prone
// full-speed reversals.
// =============================================================================
#define IR_PID_KP                    11 //good value is 10
#define IR_PID_KI                     0 //good value is 0
#define IR_PID_KD                    30 // good value is 29
#define IR_PID_INTEGRAL_LIMIT       12000
#define IR_PID_OUTPUT_LIMIT           100 //defualt 300
#define IR_PID_MIN_ACTIVE_SPEED        65

static int32_t g_ir_pid_integral = 0;
static int16_t g_ir_pid_last_error = 0;
static bool    g_ir_pid_initialized = false;

// =============================================================================
// LINE PID MODE — weighted-position PID with Bluetooth 2-byte tuning
//
// Position 0..4000: 0=leftmost sensor, 2000=center, 4000=rightmost sensor
// error = 2000 - position  (positive = line left, negative = line right)
// Gains sent as raw uint8 via Bluetooth; effective gain = raw / 10^multiX
// Speed range follows the spec (-255..255), scaled to platform (-1000..1000)
// =============================================================================
#define LINE_PID_LOOP_MS            8u
#define LINE_PID_LFSPEED_DEFAULT    230
#define LINE_PID_I_LIMIT            5000
#define LINE_PID_RECOVERY_SPEED     225   // ≈ 230/255 × 1000, platform scale

static uint8_t  g_pid_kp      = 13u; //default is 13
static uint8_t  g_pid_ki      = 0u; //default is 0
static uint8_t  g_pid_kd      = 5u; //default is 5
static uint8_t  g_pid_mp      = 2u;   // multiP: divide Kp by 10^multiP
static uint8_t  g_pid_mi      = 2u;
static uint8_t  g_pid_md      = 2u;
static int16_t  g_pid_lfspd   = LINE_PID_LFSPEED_DEFAULT;
static int32_t  g_pid_I       = 0;
static int16_t  g_pid_prev_err = 0;

// Bluetooth 2-byte frame parser (cmd=1-7, val=0-255)
static uint8_t  g_bt_cnt    = 0u;
static uint8_t  g_bt_v[3]   = {0u, 0u, 0u};   // 1-indexed

static const int32_t g_pow10[10] = {
    1L, 10L, 100L, 1000L, 10000L, 100000L, 1000000L, 10000000L, 100000000L, 1000000000L
};

static void ir_pid_reset(void)
{
    g_ir_pid_integral = 0;
    g_ir_pid_last_error = 0;
    g_ir_pid_initialized = true;
}

static int16_t ir_pid_apply_limits(int32_t value)
{
    if (value > IR_PID_OUTPUT_LIMIT) return IR_PID_OUTPUT_LIMIT;
    if (value < -IR_PID_OUTPUT_LIMIT) return (int16_t)(-IR_PID_OUTPUT_LIMIT);
    return (int16_t)value;
}

static int16_t ir_pid_limit_base_speed(int16_t base_speed)
{
    if (base_speed <= 0) return 0;
    if (base_speed < IR_PID_MIN_ACTIVE_SPEED) return IR_PID_MIN_ACTIVE_SPEED;
    if (base_speed > MAX_SPEED_CMD) return MAX_SPEED_CMD;
    return base_speed;
}

static int16_t clamp_motor_cmd(int32_t v)
{
    if (v < 0) return 0;
    if (v > MAX_SPEED_CMD) return MAX_SPEED_CMD;
    return (int16_t)v;
}

// Output smoother — FORWARD only. LEFT/RIGHT bypass for instant response.
// Reduced from 7/3 to 5/5 for faster response (50/50 weighting instead of 70/30)
#define IR_SMOOTH_OLD_WEIGHT          5
#define IR_SMOOTH_NEW_WEIGHT          5
#define IR_SMOOTH_TOTAL              (IR_SMOOTH_OLD_WEIGHT + IR_SMOOTH_NEW_WEIGHT)



typedef enum
{
    DRIVE_MODE_MANUAL = 0,
    DRIVE_MODE_AUTO_IR,
    DRIVE_MODE_AUTO_ULTRASONIC,
    DRIVE_MODE_LINE_PID,
    DRIVE_MODE_ROBOT_NAV
} drive_mode_t;

typedef enum
{
    AUTO_ACT_STOP = 0,
    AUTO_ACT_FORWARD,
    AUTO_ACT_LEFT,
    AUTO_ACT_RIGHT,
    AUTO_ACT_CRAWL
} auto_action_t;

typedef enum
{
    ULTRA_ACT_STOP           = 0,  // no valid path found (safe mode trigger)
    ULTRA_ACT_FORWARD        = 1,  // straight corridor, all clear
    ULTRA_ACT_TURN_RIGHT     = 2,  // right path is open (90-degree pivot)
    ULTRA_ACT_TURN_LEFT      = 3,  // left path is open (90-degree pivot)
    ULTRA_ACT_TURN_180       = 4,  // dead end (180-degree pivot)
    ULTRA_ACT_REVERSE        = 5,  // front state 2 — back up before next decision
    ULTRA_ACT_FORWARD_LEAN_R = 6,  // forward, bias right (left side wall alert)
    ULTRA_ACT_FORWARD_LEAN_L = 7   // forward, bias left (right side wall alert)
} ultra_action_t;

typedef enum
{
    SAFE_REASON_NONE = 0,
    SAFE_REASON_COMM_LOSS,
    SAFE_REASON_ULTRA_TIMEOUT,
    SAFE_REASON_BUTTON,
    SAFE_REASON_LINE_LOST
} safe_reason_t;

typedef struct
{
    bool active;
    safe_reason_t reason;
    uint32_t entered_ms;
} safe_state_t;

typedef struct {
    int16_t target_left;
    int16_t target_right;
    int16_t current_left;
    int16_t current_right;
    uint32_t last_ramp_ms;
} ramp_state_t;

static ramp_state_t g_ramp_state = {0, 0, 0, 0, 0};

static int16_t g_ir_smooth_left  = 0;
static int16_t g_ir_smooth_right = 0;


static const char UI_OFF[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[31mOFF\033[0m                       ||\r\n"
"||                                    ||\r\n"
"||  Press onboard button              ||\r\n"
"||  to enable controls.               ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_MANUAL_HEAD[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[36mMANUAL\033[0m                    ||\r\n"
"||  Baud:   9600 or 38400             ||\r\n"
"||                                    ||\r\n"
"||  On-Board Button Alternative       ||\r\n"
"||    [B] = On-Board Button           ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"||  Manual Drive:                     ||\r\n"
"||    [W] = Forward                   ||\r\n"
"||    [S] = Backward                  ||\r\n"
"||    [A] = Turn left                 ||\r\n"
"||    [D] = Turn right                ||\r\n"
"||  [SPC] = Stop                      ||\r\n"
"||                                    ||\r\n"
"||  Speed Control:                    ||\r\n"
"||    [UP]   = Increase speed         ||\r\n"
"||    [DOWN] = Decrease speed         ||\r\n"
"||                                    ||\r\n"
"||  Added Feature:                    ||\r\n"
"||    [T] = Test all motors           ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_AUTO_IR[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[33mAUTO IR FOLLOW\033[0m            ||\r\n"
"||  Baud:   9600 or 38400             ||\r\n"
"||                                    ||\r\n"
"||  On-Board Button Alternative       ||\r\n"
"||    [B] = On-Board Button           ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"||  IR Auto Behavior:                 ||\r\n"
"||    * Proportional steering         ||\r\n"
"||    * Turns: immediate response     ||\r\n"
"||    * Straight: smoother active     ||\r\n"
"||    * No PID - no current spikes    ||\r\n"
"||    * Line lost -> CRAWL forward    ||\r\n"
"||    * Intersection: coast + cooldown||\r\n"
"||    * Split path -> pick one fork   ||\r\n"
"||    * SAFE only on 3s line loss     ||\r\n"
"||                                    ||\r\n"
"||  [SPC] = Stop motors               ||\r\n"
"||  Send [U] again to resume.         ||\r\n"
"||                                    ||\r\n"
"||  Safe Mode:                        ||\r\n"
"||    * Onboard btn -> SAFE latch     ||\r\n"
"||    * No line 3s  -> SAFE latch     ||\r\n"
"||    [X] = Clear SAFE                ||\r\n"
"||    [U] = Resume auto               ||\r\n"
"||    [Z] = Toggle debug stream       ||\r\n"
"||    [B] = On-Board Button           ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_AUTO_ULTRA[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[35mAUTO ULTRASONIC MAZE\033[0m      ||\r\n"
"||  Baud:   9600 or 38400             ||\r\n"
"||                                    ||\r\n"
"||  On-Board Button Alternative       ||\r\n"
"||    [B] = On-Board Button           ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic maze      ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"||  Wall-Following Logic:             ||\r\n"
"||    State 0 = open path (no echo)   ||\r\n"
"||    State 1 = wall at distance      ||\r\n"
"||    State 2 = wall too close        ||\r\n"
"||    Priority: Front > Right > Left  ||\r\n"
"||    Dead end  -> 180-deg pivot      ||\r\n"
"||                                    ||\r\n"
"||  Tune in defines:                  ||\r\n"
"||    ULTRA_OPEN_CM    = 22           ||\r\n"
"||    ULTRA_ALERT_CM   = 5            ||\r\n"
"||    ULTRA_TURN_90_MS = 520          ||\r\n"
"||    ULTRA_TURN_180_MS= 1040         ||\r\n"
"||                                    ||\r\n"
"||  Serial: US: F=XX[S] L=XX[S]      ||\r\n"
"||              R=XX[S] -> ACTION     ||\r\n"
"||                                    ||\r\n"
"||  [SPC] = Stop                      ||\r\n"
"||  [X]   = Clear SAFE                ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_LINE_PID[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[35mLINE PID FOLLOW\033[0m           ||\r\n"
"||  Baud:   9600 or 38400             ||\r\n"
"||                                    ||\r\n"
"||  Bluetooth 2-byte PID tuning:      ||\r\n"
"||    cmd 1 + val  = set Kp           ||\r\n"
"||    cmd 2 + val  = set multiP       ||\r\n"
"||    cmd 3 + val  = set Ki           ||\r\n"
"||    cmd 4 + val  = set multiI       ||\r\n"
"||    cmd 5 + val  = set Kd           ||\r\n"
"||    cmd 6 + val  = set multiD       ||\r\n"
"||    cmd 7 + 0/1  = stop / start     ||\r\n"
"||                                    ||\r\n"
"||  Position 0..4000, target = 2000   ||\r\n"
"||  error = 2000 - position           ||\r\n"
"||  Off-line: spins to last direction ||\r\n"
"||                                    ||\r\n"
"||  [SPC] = Stop   [X] = Clear SAFE  ||\r\n"
"||  [Z] = Toggle safe mode on/off    ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_ROBOT_NAV[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[32mROBOT NAV\033[0m                 ||\r\n"
"||  Baud:   9600 or 38400             ||\r\n"
"||                                    ||\r\n"
"||  Autonomous maze navigation.       ||\r\n"
"||  Send [N] again while ON to start. ||\r\n"
"||  Reset MCU to stop mid-run.        ||\r\n"
"||                                    ||\r\n"
"||  Sensors: PA16 TRIG (shared)       ||\r\n"
"||           PA17 Front ECHO          ||\r\n"
"||           PA18 Left  ECHO          ||\r\n"
"||           PA19 Right ECHO          ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||    [L] = Line PID follow           ||\r\n"
"||    [N] = Robot nav mode            ||\r\n"
"||                                    ||\r\n"
"========================================\r\n";

static inline char to_lower(char c)
{
    if ((c >= 'A') && (c <= 'Z')) return (char)(c - 'A' + 'a');
    return c;
}

static inline int16_t clamp_speed(int32_t v)
{
    if (v < MIN_SPEED_CMD) return MIN_SPEED_CMD;
    if (v > MAX_SPEED_CMD) return MAX_SPEED_CMD;
    return (int16_t)v;
}

static uint8_t speed_percent_display(int16_t speed)
{
    uint16_t s;
    if (speed < 0) speed = (int16_t)(-speed);
    if (speed > MAX_SPEED_CMD) speed = MAX_SPEED_CMD;
    s = (uint16_t)speed;
    return (uint8_t)((((uint32_t)s + 50u) / 100u) * 10u);
}

static const char *speed_percent_text(int16_t speed)
{
    switch (speed_percent_display(speed))
    {
        case 0:   return "Speed: 0%\r\n";
        case 10:  return "Speed: 10%\r\n";
        case 20:  return "Speed: 20%\r\n";
        case 30:  return "Speed: 30%\r\n";
        case 40:  return "Speed: 40%\r\n";
        case 50:  return "Speed: 50%\r\n";
        case 60:  return "Speed: 60%\r\n";
        case 70:  return "Speed: 70%\r\n";
        case 80:  return "Speed: 80%\r\n";
        case 90:  return "Speed: 90%\r\n";
        case 100: return "Speed: 100%\r\n";
        default:  return "Speed: ?\r\n";
    }
}

static void refresh_ui(bool on, drive_mode_t mode, int16_t speed)
{
    if (!on) { platform_usart_write_str(UI_OFF); return; }
    switch (mode)
    {
        case DRIVE_MODE_MANUAL:
            platform_usart_write_str(UI_MANUAL_HEAD);
            platform_usart_write_str(speed_percent_text(speed));
            break;
        case DRIVE_MODE_AUTO_IR:
            platform_usart_write_str(UI_AUTO_IR);
            platform_usart_write_str(speed_percent_text(speed));
            break;
        case DRIVE_MODE_AUTO_ULTRASONIC:
            platform_usart_write_str(UI_AUTO_ULTRA);
            break;
        case DRIVE_MODE_LINE_PID:
            platform_usart_write_str(UI_LINE_PID);
            break;
        case DRIVE_MODE_ROBOT_NAV:
            platform_usart_write_str(UI_ROBOT_NAV);
            break;
        default: break;
    }
}

static void system_set_on(bool *controls_on_ptr)
{
    *controls_on_ptr = true;
    platform_tb6612_enable(true);
    platform_led_set(true);
}

static void system_set_off(bool *controls_on_ptr)
{
    *controls_on_ptr = false;
    platform_motor_stop();
    platform_tb6612_enable(false);
    platform_led_set(false);
    g_ramp_state.target_left   = 0;
    g_ramp_state.target_right  = 0;
    g_ramp_state.current_left  = 0;
    g_ramp_state.current_right = 0;
}

static const char *safe_reason_text(safe_reason_t reason)
{
    switch (reason)
    {
        case SAFE_REASON_COMM_LOSS:     return "comm loss";
        case SAFE_REASON_ULTRA_TIMEOUT: return "ultrasonic timeout";
        case SAFE_REASON_BUTTON:        return "button press";
        case SAFE_REASON_LINE_LOST:     return "IR line lost for too long";
        default:                        return "unknown";
    }
}

static void safe_enter(safe_state_t *safe, safe_reason_t reason,
                       uint32_t now, bool *auto_run_enabled)
{
    if (safe->active) return;
    safe->active      = true;
    safe->reason      = reason;
    safe->entered_ms  = now;
    *auto_run_enabled = false;
    platform_motor_stop();
    platform_usart_write_str("SAFE MODE ACTIVE: ");
    platform_usart_write_str(safe_reason_text(reason));
    platform_usart_write_str(". Send X to clear.\r\n");
}

static void safe_clear(safe_state_t *safe)
{
    safe->active     = false;
    safe->reason     = SAFE_REASON_NONE;
    safe->entered_ms = 0u;
    platform_motor_stop();
    platform_usart_write_str("SAFE MODE CLEARED\r\n");
}

static int16_t ramp_toward(int16_t current, int16_t target, int16_t step)
{
    int16_t diff = target - current;
    if (diff > step)       return current + step;
    else if (diff < -step) return current - step;
    else                   return target;
}

static void set_motor_with_ramp(int16_t left, int16_t right)
{
#if RAMP_ENABLED
    g_ramp_state.target_left  = left;
    g_ramp_state.target_right = right;
#else
    platform_motor_set(left, right);
#endif
}

static void update_motor_ramp(uint32_t now_ms)
{
#if RAMP_ENABLED
    if ((now_ms - g_ramp_state.last_ramp_ms) >= RAMP_CYCLE_MS) {
        g_ramp_state.current_left  = ramp_toward(g_ramp_state.current_left,
                                                  g_ramp_state.target_left,
                                                  RAMP_STEP_PER_CYCLE);
        g_ramp_state.current_right = ramp_toward(g_ramp_state.current_right,
                                                  g_ramp_state.target_right,
                                                  RAMP_STEP_PER_CYCLE);
        platform_motor_set(g_ramp_state.current_left, g_ramp_state.current_right);
        g_ramp_state.last_ramp_ms = now_ms;
    }
#else
    (void)now_ms;
#endif
}

static void apply_gentle_turn_left(int16_t base_speed)
{
#if TURN_MODE_GENTLE_ARC
    int16_t outer = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(inner, outer);
#elif TURN_MODE_PIVOT
    int16_t spd = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(-spd, +spd);
#else
    int16_t outer = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(0, outer);
#endif
}

static void apply_gentle_turn_right(int16_t base_speed)
{
#if TURN_MODE_GENTLE_ARC
    int16_t outer = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(outer, inner);
#elif TURN_MODE_PIVOT
    int16_t spd = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(+spd, -spd);
#else
    int16_t outer = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(outer, 0);
#endif
}

static bool try_parse_arrow_speed_char(char c, bool on, drive_mode_t mode, int16_t *speed)
{
    static uint8_t esc_state = 0u;
    if (!on || ((mode != DRIVE_MODE_MANUAL) && (mode != DRIVE_MODE_AUTO_IR))) {
        esc_state = 0u; return false;
    }
    if (c == 0x1B) { esc_state = 1u; return true; }
    if (esc_state == 1u) {
        if (c == '[') { esc_state = 2u; return true; }
        esc_state = 0u; return false;
    }
    if (esc_state == 2u) {
        esc_state = 0u;
        if (c == 'A') { *speed = clamp_speed((int32_t)*speed + SPEED_STEP_CMD); return true; }
        if (c == 'B') { *speed = clamp_speed((int32_t)*speed - SPEED_STEP_CMD); return true; }
    }
    return false;
}

static uint8_t ir_mask_on_black_runtime(uint8_t raw, bool active_on_black_high)
{
    return active_on_black_high ? raw : (uint8_t)(~raw) & IR_MASK_ALL;
}

static uint8_t ir_mask_or_history(const uint8_t *history, uint8_t history_count)
{
    if (history_count == 0u) return 0u;
    uint8_t counts[5] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0u; i < history_count; i++) {
        uint8_t m = history[i];
        if (m & IR_MASK_S1) counts[0]++;
        if (m & IR_MASK_S2) counts[1]++;
        if (m & IR_MASK_S3) counts[2]++;
        if (m & IR_MASK_S4) counts[3]++;
        if (m & IR_MASK_S5) counts[4]++;
    }
    uint8_t needed = (history_count + 1u) / 2u;
    uint8_t out = 0u;
    if (counts[0] >= needed) out |= IR_MASK_S1;
    if (counts[1] >= needed) out |= IR_MASK_S2;
    if (counts[2] >= needed) out |= IR_MASK_S3;
    if (counts[3] >= needed) out |= IR_MASK_S4;
    if (counts[4] >= needed) out |= IR_MASK_S5;
    return out;
}

static auto_action_t decide_auto_action(uint8_t steer_mask,
                                         uint8_t detect_mask,
                                         int8_t  turn_threshold,
                                         uint8_t min_black_count,
                                         int8_t  *sum_out,
                                         uint8_t *count_out)
{
    int8_t  sum   = 0;
    uint8_t count = 0u;

    if (detect_mask & IR_MASK_S1) count++;
    if (detect_mask & IR_MASK_S2) count++;
    if (detect_mask & IR_MASK_S3) count++;
    if (detect_mask & IR_MASK_S4) count++;
    if (detect_mask & IR_MASK_S5) count++;

    if (steer_mask & IR_MASK_S1) sum += -2;
    if (steer_mask & IR_MASK_S2) sum += -1;
    if (steer_mask & IR_MASK_S3) sum +=  0;
    if (steer_mask & IR_MASK_S4) sum += +1;
    if (steer_mask & IR_MASK_S5) sum += +2;

    if (sum_out)   *sum_out   = sum;
    if (count_out) *count_out = count;

    (void)turn_threshold;

    bool left_active   = ((detect_mask & (IR_MASK_S1 | IR_MASK_S2)) != 0u);
    bool center_active = ((detect_mask & IR_MASK_S3) != 0u);
    bool right_active  = ((detect_mask & (IR_MASK_S4 | IR_MASK_S5)) != 0u);

    if (count < min_black_count)
    {
        // Below detection threshold: default to CRAWL (line lost).
        if (sum_out) *sum_out = 0;
        return AUTO_ACT_CRAWL;
    }

    if (left_active && !right_active) {
        if (sum_out) *sum_out = -1;
        return AUTO_ACT_LEFT;
    }

    if (right_active && !left_active) {
        if (sum_out) *sum_out = +1;
        return AUTO_ACT_RIGHT;
    }

    if (center_active || (left_active && right_active)) {
        if (sum_out) *sum_out = 0;
        return AUTO_ACT_FORWARD;
    }

    return AUTO_ACT_FORWARD;
}

static void ir_smooth_reset(int16_t seed_left, int16_t seed_right)
{
    g_ir_smooth_left  = seed_left;
    g_ir_smooth_right = seed_right;
}

static void ir_smooth_apply(int16_t target_left, int16_t target_right)
{
    int16_t out_left  = (int16_t)(
        ((int32_t)g_ir_smooth_left  * IR_SMOOTH_OLD_WEIGHT +
         (int32_t)target_left       * IR_SMOOTH_NEW_WEIGHT) / IR_SMOOTH_TOTAL);

    int16_t out_right = (int16_t)(
        ((int32_t)g_ir_smooth_right * IR_SMOOTH_OLD_WEIGHT +
         (int32_t)target_right      * IR_SMOOTH_NEW_WEIGHT) / IR_SMOOTH_TOTAL);

    g_ir_smooth_left  = out_left;
    g_ir_smooth_right = out_right;

    platform_motor_set(out_left, out_right);
}

static void ir_compute_steer(int8_t  error,
                              int16_t base_speed,
                              int8_t  steer_step,
                              int16_t *left_out,
                              int16_t *right_out)
{
    (void)steer_step;

    if ((!left_out) || (!right_out)) {
        return;
    }

    if (!g_ir_pid_initialized) {
        ir_pid_reset();
    }

    base_speed = ir_pid_limit_base_speed(base_speed);

    int16_t error_delta = (int16_t)(error - g_ir_pid_last_error);
    g_ir_pid_integral += (int32_t)error;
    if (g_ir_pid_integral > IR_PID_INTEGRAL_LIMIT) {
        g_ir_pid_integral = IR_PID_INTEGRAL_LIMIT;
    } else if (g_ir_pid_integral < -IR_PID_INTEGRAL_LIMIT) {
        g_ir_pid_integral = -IR_PID_INTEGRAL_LIMIT;
    }

    int32_t p_term = (int32_t)IR_PID_KP * (int32_t)error;
    int32_t i_term = (int32_t)IR_PID_KI * g_ir_pid_integral / 8;
    int32_t d_term = (int32_t)IR_PID_KD * (int32_t)error_delta;

    int16_t correction = ir_pid_apply_limits((p_term + i_term + d_term) / 8);
    int16_t base_clamped = clamp_motor_cmd(base_speed);

    int32_t left_cmd = (int32_t)base_clamped - (int32_t)correction;
    int32_t right_cmd = (int32_t)base_clamped + (int32_t)correction;

    if (left_cmd > MAX_SPEED_CMD) left_cmd = MAX_SPEED_CMD;
    if (left_cmd < 0) left_cmd = 0;
    if (right_cmd > MAX_SPEED_CMD) right_cmd = MAX_SPEED_CMD;
    if (right_cmd < 0) right_cmd = 0;

    *left_out  = clamp_motor_cmd(left_cmd);
    *right_out = clamp_motor_cmd(right_cmd);

    g_ir_pid_last_error = error;
}

static void apply_auto_action(auto_action_t act,
                               int16_t base_speed,
                               int8_t  sum,
                               int8_t  last_nonzero_sum)
{
    (void)last_nonzero_sum;

    switch (act)
    {
        case AUTO_ACT_STOP:
            ir_smooth_reset(0, 0);
            ir_pid_reset();
            platform_motor_stop();
            break;

        case AUTO_ACT_FORWARD:
        {
            // Smoother ON — prevents micro-jitter on straight lines.
            int16_t tl, tr;
            ir_compute_steer(sum, base_speed, IR_STEER_STEP_PERCENT, &tl, &tr);
            ir_smooth_apply(tl, tr);
            break;
        }

        case AUTO_ACT_LEFT:
        {
            // Single-motor turn: right motor ON, left motor OFF
            ir_smooth_reset(0, base_speed);
            platform_motor_set(0, base_speed);
            break;
        }

        case AUTO_ACT_RIGHT:
        {
            // Single-motor turn: left motor ON, right motor OFF
            ir_smooth_reset(base_speed, 0);
            platform_motor_set(base_speed, 0);
            break;
        }

        case AUTO_ACT_CRAWL:
        {
            int16_t crawl_speed = (int16_t)(((int32_t)base_speed * IR_CRAWL_SPEED_PERCENT) / 100);
            if (crawl_speed < IR_CRAWL_MIN_SPEED) {
                crawl_speed = IR_CRAWL_MIN_SPEED;
            }

            if (last_nonzero_sum <= -2) {
                ir_smooth_reset(0, crawl_speed);
                platform_motor_set(0, crawl_speed);
            } else if (last_nonzero_sum >= 2) {
                ir_smooth_reset(crawl_speed, 0);
                platform_motor_set(crawl_speed, 0);
            } else {
                ir_smooth_reset(crawl_speed, crawl_speed);
                platform_motor_set(crawl_speed, crawl_speed);
            }
            break;
        }

        default: break;
    }
}

static void print_ir_debug_status(uint8_t raw_mask, uint8_t black_mask,
                                   int8_t sum, uint8_t count, auto_action_t act)
{
    char buf[80];
    uint32_t i = 0u;

    buf[i++]='I'; buf[i++]='R'; buf[i++]=':'; buf[i++]=' ';
    buf[i++]=(raw_mask & IR_MASK_S1)?'1':'0';
    buf[i++]=(raw_mask & IR_MASK_S2)?'1':'0';
    buf[i++]=(raw_mask & IR_MASK_S3)?'1':'0';
    buf[i++]=(raw_mask & IR_MASK_S4)?'1':'0';
    buf[i++]=(raw_mask & IR_MASK_S5)?'1':'0';

    buf[i++]=' '; buf[i++]='B'; buf[i++]=':';
    buf[i++]=(black_mask & IR_MASK_S1)?'1':'0';
    buf[i++]=(black_mask & IR_MASK_S2)?'1':'0';
    buf[i++]=(black_mask & IR_MASK_S3)?'1':'0';
    buf[i++]=(black_mask & IR_MASK_S4)?'1':'0';
    buf[i++]=(black_mask & IR_MASK_S5)?'1':'0';

    buf[i++]=' '; buf[i++]='S'; buf[i++]='u'; buf[i++]='m'; buf[i++]='=';
    if (sum < 0) { buf[i++]='-'; buf[i++]=(char)('0'+(uint8_t)(-sum)); }
    else         { buf[i++]='+'; buf[i++]=(char)('0'+(uint8_t)sum); }

    buf[i++]=' '; buf[i++]='C'; buf[i++]='n'; buf[i++]='t'; buf[i++]='=';
    buf[i++]=(char)('0'+count);
    buf[i++]=' ';

    switch (act) {
        case AUTO_ACT_STOP:              buf[i++]='S';buf[i++]='T';buf[i++]='O';buf[i++]='P'; break;
        case AUTO_ACT_FORWARD:           buf[i++]='F';buf[i++]='W';buf[i++]='D'; break;
        case AUTO_ACT_LEFT:              buf[i++]='L';buf[i++]='E';buf[i++]='F';buf[i++]='T'; break;
        case AUTO_ACT_RIGHT:             buf[i++]='R';buf[i++]='I';buf[i++]='G';buf[i++]='H';buf[i++]='T'; break;
        case AUTO_ACT_CRAWL:             buf[i++]='C';buf[i++]='R';buf[i++]='A';buf[i++]='W';buf[i++]='L'; break;
        default: break;
    }

    buf[i++]='\r'; buf[i++]='\n';
    platform_usart_write_buf(buf, i);
}

// =============================================================================
// ULTRASONIC MAZE FRONT-FIRST PROBE
//
// cm → state conversion:
//   read failed OR cm == 0 OR cm >= ULTRA_MAX_VALID_CM  → 0 (open path)
//   cm > ULTRA_OPEN_CM                                  → 0 (open path)
//   cm <= ULTRA_ALERT_CM                                → 2 (wall too close)
//   otherwise                                           → 1 (wall at distance)
//
// State 2 requires ULTRA_ALERT_COUNT_THRESH consecutive readings (hysteresis).
// Alert counts are held in ultra_alert_count[3] in main().
//
// Decision rules use a front-first probe strategy:
//   1. If the front is open, continue forward.
//   2. Otherwise stop and choose a turn direction.
//   3. After the turn, the next sensor cycle re-checks the new front.
//      If the front is still blocked, the robot turns again.
//   4. If all sides are blocked, turn 180 degrees left.
//
// "Open" means state 0 only. States 1 and 2 are treated as blocked for
// navigation; state 2 is still tracked separately by the hysteresis logic so
// the robot reacts smoothly when a wall gets too close.
// =============================================================================

static int16_t ultra_center_prev_err = 0;
static int32_t ultra_center_I = 0;
static uint8_t ultra_turn_cooldown = 0u;

static bool ultra_side_cm_valid(uint16_t cm)
{
    if (cm == 0u) return false;
    if (cm >= ULTRA_MAX_VALID_CM) return false;
    if (cm < ULTRA_SIDE_MIN_CM) return false;
    if (cm > ULTRA_SIDE_MAX_CM) return false;
    return true;
}

// Target distance from each side wall when perfectly centred in the corridor.
// With a ~29 cm track, the midpoint is ~14–15 cm; use 10 cm as the nominal
// setpoint so the PID keeps the robot equidistant from both walls.
#define ULTRA_CENTER_TARGET_CM   10

static void ultrasonic_forward_pid(uint16_t left_cm, uint16_t right_cm)
{
    int16_t error;
    int16_t d_err;
    int32_t correction;
    int32_t left_speed;
    int32_t right_speed;

    bool left_valid  = ultra_side_cm_valid(left_cm);
    bool right_valid = ultra_side_cm_valid(right_cm);

    /*
     * Double-wall mode: both sensors see a wall within the valid range.
     * error = right_cm - left_cm
     *   > 0 → robot drifted left  (left wall closer) → steer right (positive correction)
     *   < 0 → robot drifted right (right wall closer) → steer left (negative correction)
     * This naturally centres the robot between both walls.
     */
    if (left_valid && right_valid)
    {
        error = (int16_t)right_cm - (int16_t)left_cm;
    }
    /*
     * Single-wall fallback: only the left sensor has a usable reading.
     * Target: keep left wall at ULTRA_CENTER_TARGET_CM.
     * error > 0 → too close to left wall → steer right.
     */
    else if (left_valid)
    {
        error = (int16_t)ULTRA_CENTER_TARGET_CM - (int16_t)left_cm;
        error = -error; /* invert: left too close → positive correction (steer right) */
    }
    /*
     * Single-wall fallback: only the right sensor has a usable reading.
     * Target: keep right wall at ULTRA_CENTER_TARGET_CM.
     * error < 0 → too close to right wall → steer left.
     */
    else if (right_valid)
    {
        error = (int16_t)right_cm - (int16_t)ULTRA_CENTER_TARGET_CM;
    }
    /*
     * No usable side readings — drive straight with trim only.
     * Reset integrator so stale I-term doesn't jerk the robot when
     * readings return.
     */
    else
    {
        ultra_center_prev_err = 0;
        ultra_center_I        = 0;
        platform_motor_set(ULTRA_FORWARD_SPEED + ULTRA_LEFT_TRIM,
                           ULTRA_FORWARD_SPEED + ULTRA_RIGHT_TRIM);
        return;
    }

    ultra_center_I += error;

    if (ultra_center_I > ULTRA_CENTER_I_LIMIT) {
        ultra_center_I = ULTRA_CENTER_I_LIMIT;
    }

    if (ultra_center_I < -ULTRA_CENTER_I_LIMIT) {
        ultra_center_I = -ULTRA_CENTER_I_LIMIT;
    }

    d_err = error - ultra_center_prev_err;
    ultra_center_prev_err = error;

    correction =
        ((int32_t)ULTRA_CENTER_KP * error) +
        ((int32_t)ULTRA_CENTER_KI * ultra_center_I) +
        ((int32_t)ULTRA_CENTER_KD * d_err);

    if (correction > ULTRA_CENTER_CORR_LIMIT) {
        correction = ULTRA_CENTER_CORR_LIMIT;
    }

    if (correction < -ULTRA_CENTER_CORR_LIMIT) {
        correction = -ULTRA_CENTER_CORR_LIMIT;
    }

    /*
     * Positive correction = steer right.
     * To steer right: speed up left motor, slow down right motor.
     */
    left_speed  = (int32_t)ULTRA_FORWARD_SPEED + correction + ULTRA_LEFT_TRIM;
    right_speed = (int32_t)ULTRA_FORWARD_SPEED - correction + ULTRA_RIGHT_TRIM;

    if (left_speed  >  1000L) left_speed  =  1000L;
    if (left_speed  < -1000L) left_speed  = -1000L;
    if (right_speed >  1000L) right_speed =  1000L;
    if (right_speed < -1000L) right_speed = -1000L;

    platform_motor_set((int16_t)left_speed, (int16_t)right_speed);
}

static uint16_t ultra_sanitize_cm(uint16_t cm, bool ok)
{
    if (!ok) {
        return NO_ECHO_VALUE;
    }

    if (cm == 0u) {
        return NO_ECHO_VALUE;
    }

    if (cm >= ULTRA_MAX_VALID_CM) {
        return NO_ECHO_VALUE;
    }

    return cm;
}

static void ultra_sort3(uint16_t *a, uint16_t *b, uint16_t *c)
{
    uint16_t temp;

    if (*a > *b) {
        temp = *a;
        *a = *b;
        *b = temp;
    }

    if (*b > *c) {
        temp = *b;
        *b = *c;
        *c = temp;
    }

    if (*a > *b) {
        temp = *a;
        *a = *b;
        *b = temp;
    }
}

static bool ultra_read_stable_cm(uint8_t sensor, uint16_t *cm_out)
{
    uint16_t s0;
    uint16_t s1;
    uint16_t s2;

    bool ok0;
    bool ok1;
    bool ok2;

    /*
     * Take three rapid pings with a short gap between each.
     * ULTRA_SAMPLE_GAP_MS is kept small (5 ms) so the total read time
     * per sensor is ~12–15 ms. No extra delays are inserted between
     * sensors in the main loop — that was the main source of latency.
     * The median of the three readings rejects single-ping spikes.
     */
    ok0 = platform_ultrasonic_read_cm(sensor, &s0);
    s0  = ultra_sanitize_cm(s0, ok0);

    platform_delay_ms(ULTRA_SAMPLE_GAP_MS);

    ok1 = platform_ultrasonic_read_cm(sensor, &s1);
    s1  = ultra_sanitize_cm(s1, ok1);

    platform_delay_ms(ULTRA_SAMPLE_GAP_MS);

    ok2 = platform_ultrasonic_read_cm(sensor, &s2);
    s2  = ultra_sanitize_cm(s2, ok2);

    /* Median of 3 — rejects one bad spike in either direction. */
    ultra_sort3(&s0, &s1, &s2);

    *cm_out = s1;

    return (*cm_out != NO_ECHO_VALUE);
}

static uint8_t ultra_cm_to_raw_state(uint16_t cm, bool ok)
{
    /*
     * State encoding:
     *   0 = open / no echo   (cm > ULTRA_OPEN_CM=16, or read failed)
     *   1 = wall present     (ULTRA_ALERT_CM < cm <= ULTRA_OPEN_CM)
     *   2 = wall too close   (cm <= ULTRA_ALERT_CM=5) — triggers turn after hysteresis
     *
     * For the front sensor:  state 2 fires a 90° turn decision.
     * For the side sensors:  the raw cm value is used directly by the PID;
     *                        state is used only for the dead-end / no-path check.
     */
    if (!ok)                      return 0u;
    if (cm == 0u)                 return 0u;
    if (cm >= ULTRA_MAX_VALID_CM) return 0u;
    if (cm > ULTRA_OPEN_CM)       return 0u;
    if (cm <= ULTRA_ALERT_CM)     return 2u;
    return 1u;
}

static uint8_t ultra_apply_hysteresis(uint8_t raw, uint8_t *count)
{
    if (raw == 2u) {
        if (*count < ULTRA_ALERT_COUNT_THRESH) (*count)++;
    } else {
        *count = 0u;
    }
    if ((raw == 2u) && (*count >= ULTRA_ALERT_COUNT_THRESH)) return 2u;
    return (raw == 0u) ? 0u : 1u;
}

static ultra_action_t decide_ultrasonic_action(uint8_t F, uint8_t L, uint8_t R)
{
    bool left_open  = (L == 0u);
    bool right_open = (R == 0u);

    /*
     * After a turn, force the robot forward for ULTRA_TURN_COOLDOWN cycles.
     * This prevents the robot from immediately re-triggering a turn because
     * the old corner wall is still in the front sensor's field of view.
     */
    if (ultra_turn_cooldown > 0u) {
        ultra_turn_cooldown--;
        return ULTRA_ACT_FORWARD;
    }

    /*
     * Turn decision: trigger when the front wall is at state 1 OR 2.
     *
     * In a double-wall 29 cm corridor the robot is already quite close to
     * both side walls, so it may never dip below ULTRA_ALERT_CM before
     * contacting the front wall. Reacting at state 1 (wall present within
     * ULTRA_OPEN_CM) gives enough braking distance. The hysteresis filter
     * (ULTRA_ALERT_COUNT_THRESH = 2) still prevents single-ping noise from
     * causing phantom turns — but now the threshold applies to state 1 too
     * because state 2 is only produced after 2 consecutive state-2 readings.
     *
     * Priority: right turn > left turn > 180 (prefer right-hand rule).
     */
    if (F >= 1u)
    {
        if (right_open && !left_open) {
            return ULTRA_ACT_TURN_RIGHT;
        }

        if (left_open && !right_open) {
            return ULTRA_ACT_TURN_LEFT;
        }

        if (right_open && left_open) {
            /* Both sides open: default to right (right-hand wall following). */
            return ULTRA_ACT_TURN_RIGHT;
        }

        /* All three blocked — dead end, turn 180. */
        return ULTRA_ACT_TURN_180;
    }

    return ULTRA_ACT_FORWARD;
}

static void ultra_pivot(int16_t left_spd, int16_t right_spd, uint32_t duration_ms)
{
    uint32_t t;

    /* Brief stop so momentum doesn't carry into the turn arc. */
    platform_motor_stop();
    t = platform_millis();
    while ((platform_millis() - t) < ULTRA_STOP_BEFORE_TURN_MS) { asm("nop"); }

    /* Execute timed pivot. */
    platform_motor_set(left_spd, right_spd);
    t = platform_millis();
    while ((platform_millis() - t) < duration_ms) { asm("nop"); }
    platform_motor_stop();

    /*
     * Clear PID state so the first forward cycle after the turn starts
     * from a clean slate. Without this the derivative term fires a large
     * spike because the error jumps discontinuously after a 90° rotation.
     */
    ultra_center_prev_err = 0;
    ultra_center_I        = 0;
}

static void apply_ultrasonic_action(ultra_action_t act,
                                    uint16_t left_cm,
                                    uint16_t right_cm)
{
    switch (act)
    {
        case ULTRA_ACT_FORWARD:
            ultrasonic_forward_pid(left_cm, right_cm);
            break;
        case ULTRA_ACT_FORWARD_LEAN_R:
            // Left wall too close: slow the left motor to steer right
            platform_motor_set(ULTRA_FORWARD_SPEED * 6 / 10, ULTRA_FORWARD_SPEED);
            break;
        case ULTRA_ACT_FORWARD_LEAN_L:
            // Right wall too close: slow the right motor to steer left
            platform_motor_set(ULTRA_FORWARD_SPEED, ULTRA_FORWARD_SPEED * 6 / 10);
            break;
        case ULTRA_ACT_REVERSE:
            ultra_pivot(-ULTRA_FORWARD_SPEED, -ULTRA_FORWARD_SPEED, ULTRA_REVERSE_MS);
            break;
        case ULTRA_ACT_TURN_RIGHT:
            ultra_pivot(-ULTRA_TURN_SPEED, +ULTRA_TURN_SPEED, ULTRA_TURN_90_MS);
            ultra_turn_cooldown = ULTRA_TURN_COOLDOWN;
            break;

        case ULTRA_ACT_TURN_LEFT:
            ultra_pivot(+ULTRA_TURN_SPEED, -ULTRA_TURN_SPEED, ULTRA_TURN_90_MS);
            ultra_turn_cooldown = ULTRA_TURN_COOLDOWN;
            break;

        case ULTRA_ACT_TURN_180:
            ultra_pivot(-ULTRA_TURN_SPEED, +ULTRA_TURN_SPEED, ULTRA_TURN_180_MS);
            ultra_turn_cooldown = ULTRA_TURN_COOLDOWN;
            break;
        case ULTRA_ACT_STOP:
        default:
            platform_motor_stop();
            break;
    }
}

static void print_ultrasonic_status(uint16_t front_cm, uint8_t front_state,
                                    uint16_t left_cm,  uint8_t left_state,
                                    uint16_t right_cm, uint8_t right_state,
                                    ultra_action_t act)
{
    char buf[96];
    uint32_t i = 0u;

    buf[i++]='U'; buf[i++]='S'; buf[i++]=':'; buf[i++]=' ';

    buf[i++]='F'; buf[i++]='=';
    if (front_cm >= 100u) { buf[i++]=(char)('0' + (front_cm / 100u)); }
    if (front_cm >= 10u)  { buf[i++]=(char)('0' + ((front_cm / 10u) % 10u)); }
    buf[i++]=(char)('0' + (front_cm % 10u));
    buf[i++]='['; buf[i++]=(char)('0' + front_state); buf[i++]=']'; buf[i++]=' ';

    buf[i++]='L'; buf[i++]='=';
    if (left_cm >= 100u) { buf[i++]=(char)('0' + (left_cm / 100u)); }
    if (left_cm >= 10u)  { buf[i++]=(char)('0' + ((left_cm / 10u) % 10u)); }
    buf[i++]=(char)('0' + (left_cm % 10u));
    buf[i++]='['; buf[i++]=(char)('0' + left_state); buf[i++]=']'; buf[i++]=' ';

    buf[i++]='R'; buf[i++]='=';
    if (right_cm >= 100u) { buf[i++]=(char)('0' + (right_cm / 100u)); }
    if (right_cm >= 10u)  { buf[i++]=(char)('0' + ((right_cm / 10u) % 10u)); }
    buf[i++]=(char)('0' + (right_cm % 10u));
    buf[i++]='['; buf[i++]=(char)('0' + right_state); buf[i++]=']'; buf[i++]=' ';

    buf[i++]='-'; buf[i++]='>'; buf[i++]=' ';

    switch (act)
    {
        case ULTRA_ACT_FORWARD:
            buf[i++]='F'; buf[i++]='W'; buf[i++]='D';
            break;
        case ULTRA_ACT_TURN_RIGHT:
            buf[i++]='R'; buf[i++]='I'; buf[i++]='G'; buf[i++]='H'; buf[i++]='T';
            break;
        case ULTRA_ACT_TURN_LEFT:
            buf[i++]='L'; buf[i++]='E'; buf[i++]='F'; buf[i++]='T';
            break;
        case ULTRA_ACT_TURN_180:
            buf[i++]='1'; buf[i++]='8'; buf[i++]='0';
            break;
        case ULTRA_ACT_REVERSE:
            buf[i++]='R'; buf[i++]='E'; buf[i++]='V';
            break;
        case ULTRA_ACT_FORWARD_LEAN_R:
            buf[i++]='F'; buf[i++]='W'; buf[i++]='D'; buf[i++]='_'; buf[i++]='R';
            break;
        case ULTRA_ACT_FORWARD_LEAN_L:
            buf[i++]='F'; buf[i++]='W'; buf[i++]='D'; buf[i++]='_'; buf[i++]='L';
            break;
        case ULTRA_ACT_STOP:
        default:
            buf[i++]='S'; buf[i++]='T'; buf[i++]='O'; buf[i++]='P';
            break;
    }

    buf[i++]='\r'; buf[i++]='\n';
    platform_usart_write_buf(buf, i);
}

// -----------------------------------------------------------------------------
// line_position_from_mask
//
// Converts a 5-bit digital sensor mask (bit i = 1 means sensor i+1 sees black)
// to a weighted-average position 0..4000.
//   0    = line under leftmost  sensor (S1)
//   2000 = line centred         (S3)
//   4000 = line under rightmost sensor (S5)
// Returns 2000 when no sensor sees black (used to seed recovery spin).
// -----------------------------------------------------------------------------
static uint16_t line_position_from_mask(uint8_t black_mask)
{
    uint32_t weighted_sum = 0u;
    uint32_t total_weight = 0u;
    uint8_t  i;

    for (i = 0u; i < 5u; i++) {
        if (black_mask & (1u << i)) {
            weighted_sum += (uint32_t)i * 1000000u;   // weight × position (×1000 each)
            total_weight += 1000u;
        }
    }

    if (total_weight == 0u) return 2000u;
    return (uint16_t)(weighted_sum / total_weight);
}

// -----------------------------------------------------------------------------
// line_pid_apply
//
// Runs one PID iteration given the signed error and base speed (spec scale
// 0..255).  Clamps output to ±255, then scales to the platform's 0..1000
// range before calling platform_motor_set().
// -----------------------------------------------------------------------------
static void line_pid_apply(int16_t error, int16_t lfspeed)
{
    int32_t div_p = g_pow10[(g_pid_mp < 10u) ? g_pid_mp : 9u];
    int32_t div_i = g_pow10[(g_pid_mi < 10u) ? g_pid_mi : 9u];
    int32_t div_d = g_pow10[(g_pid_md < 10u) ? g_pid_md : 9u];
    int16_t D_val = (int16_t)(error - g_pid_prev_err);
    int32_t Pvalue, Ivalue, Dvalue, PIDvalue;
    int32_t left_speed, right_speed;

    g_pid_I += (int32_t)error;
    if (g_pid_I >  LINE_PID_I_LIMIT) g_pid_I =  LINE_PID_I_LIMIT;
    if (g_pid_I < -LINE_PID_I_LIMIT) g_pid_I = -LINE_PID_I_LIMIT;

    Pvalue   = ((int32_t)g_pid_kp * (int32_t)error) / div_p;
    Ivalue   = ((int32_t)g_pid_ki * g_pid_I)         / div_i;
    Dvalue   = ((int32_t)g_pid_kd * (int32_t)D_val)  / div_d;
    PIDvalue = Pvalue + Ivalue + Dvalue;

    g_pid_prev_err = error;

    left_speed  = (int32_t)lfspeed - PIDvalue;
    right_speed = (int32_t)lfspeed + PIDvalue;

    if (left_speed  >  255L) left_speed  =  255L;
    if (left_speed  < -255L) left_speed  = -255L;
    if (right_speed >  255L) right_speed =  255L;
    if (right_speed < -255L) right_speed = -255L;

    // Scale from spec's ±255 to platform's ±1000
    platform_motor_set(
        (int16_t)((left_speed  * 1000L) / 255L),
        (int16_t)((right_speed * 1000L) / 255L)
    );
}


/*
=====================================
MAIN LOOP
=====================================
*/
int main(void)
{
    bool         controls_on   = false;
    drive_mode_t mode          = DRIVE_MODE_MANUAL;
    int16_t      current_speed = DEFAULT_SPEED_CMD;

    char     active_move_key = 0;
    uint32_t last_cmd_ms     = 0u;
    uint32_t last_auto_ms    = 0u;
    uint32_t last_ultra_ms   = 0u;

    int8_t   ir_turn_threshold       = IR_TURN_THRESHOLD_DEFAULT;
    uint8_t  ir_min_black_count      = IR_MIN_COUNT_DEFAULT;
    bool     ir_debug_stream_enabled = false;
    bool     auto_run_enabled        = false;
    bool     ir_active_on_black_high = (IR_ACTIVE_ON_BLACK_HIGH != 0u);
    int8_t   last_nonzero_sum        = 0;

    safe_state_t safe          = {false, SAFE_REASON_NONE, 0u};
    uint32_t     last_rx_ms    = 0u;
    uint8_t      ultra_fail_streak = 0u;

    uint8_t  ir_black_history[IR_SAMPLE_HISTORY_SIZE] = {0u, 0u};
    uint8_t  ir_black_history_count = 0u;
    uint8_t  ir_black_history_index = 0u;
    uint32_t ir_crawl_since_ms      = 0u;

    bool     ultra_safe_enabled    = false;
    bool     line_pid_safe_enabled = false;

    uint16_t ultra_front_cm = 0u;
    uint16_t ultra_left_cm  = 0u;
    uint16_t ultra_right_cm = 0u;

    // State tracking for maze wall-following
    uint8_t  ultra_alert_count[3]  = {0u, 0u, 0u}; // hysteresis per sensor [F,L,R]
    uint8_t  ultra_no_path_streak  = 0u;            // consecutive all-zero reads

    bool     last_reading   = false;
    bool     stable_state   = false;
    uint32_t last_change_ms = 0u;
    uint32_t button_press_ms = 0u;
    bool     button_hold_fired = false;

    platform_initialization();
    refresh_ui(controls_on, mode, current_speed);

    while (1)
    {
        uint32_t now = platform_millis();

        update_motor_ramp(now);

        // ===== BUTTON DEBOUNCE =====
        {
            bool reading = platform_button_pressed();
            if (reading != last_reading) { last_change_ms = now; last_reading = reading; }

            if ((now - last_change_ms) > DEBOUNCE_MS) {
                if (stable_state != reading) {
                    stable_state = reading;

                    if (stable_state) {
                        if (!controls_on) {
                            platform_usart_write_str("BTN: Button pressed — Controls ENABLED. Select mode: M / U / O\r\n");
                            system_set_on(&controls_on);
                            current_speed     = BUTTON_ON_SPEED_CMD;
                            auto_run_enabled  = !safe.active &&
                                               ((mode == DRIVE_MODE_AUTO_IR)        ||
                                                (mode == DRIVE_MODE_AUTO_ULTRASONIC) ||
                                                (mode == DRIVE_MODE_LINE_PID));
                            if ((mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled) {
                                last_ultra_ms        = now - ULTRA_POLL_MS;
                                ultra_no_path_streak = 0u;
                                ultra_alert_count[0] = 0u;
                                ultra_alert_count[1] = 0u;
                                ultra_alert_count[2] = 0u;
                            }
                            last_rx_ms        = now;
                            ultra_fail_streak = 0u;
                            active_move_key   = 0;
                            /* Prevent immediate OFF while the same press is still held. */
                            button_hold_fired = true;
                            button_press_ms   = now;
                            refresh_ui(controls_on, mode, current_speed);
                        } else {
                            /* When controls are ON, require deliberate hold before OFF/SAFE. */
                            button_press_ms = now;
                            button_hold_fired = false;
                        }
                    } else {
                        /* Re-arm hold detector on release. */
                        button_hold_fired = false;
                    }
                }

                if (controls_on && stable_state && !button_hold_fired) {
                    if ((now - button_press_ms) >= BUTTON_OFF_HOLD_MS) {
                        if (((mode == DRIVE_MODE_AUTO_IR)        ||
                             (mode == DRIVE_MODE_AUTO_ULTRASONIC) ||
                             (mode == DRIVE_MODE_LINE_PID)) &&
                            auto_run_enabled && !safe.active) {
                            platform_usart_write_str("BTN: Hold detected — entering SAFE mode.\r\n");
                            safe_enter(&safe, SAFE_REASON_BUTTON, now, &auto_run_enabled);

                        } else {
                            platform_usart_write_str("BTN: Hold detected — Controls DISABLED. Motors stopped.\r\n");
                            system_set_off(&controls_on);
                            auto_run_enabled  = false;
                            safe.active       = false;
                            safe.reason       = SAFE_REASON_NONE;
                            safe.entered_ms   = 0u;
                            ultra_fail_streak = 0u;
                            active_move_key   = 0;
                        }

                        button_hold_fired = true;
                        refresh_ui(controls_on, mode, current_speed);
                    }
                }
            }
        }

        // ===== SERIAL INPUT =====
        {
            char c;
            while (platform_usart_read_char(&c))
            {
                now = platform_millis();
                last_rx_ms = now;

                // ── Bluetooth 2-byte PID frame (bytes 1-7 are control chars) ──
                // Complete second byte of an in-progress frame
                if (g_bt_cnt == 1u) {
                    g_bt_v[2] = (uint8_t)c;
                    g_bt_cnt  = 0u;
                    switch (g_bt_v[1]) {
                        case 1u: g_pid_kp  = g_bt_v[2]; break;
                        case 2u: g_pid_mp  = (g_bt_v[2] >= 1u && g_bt_v[2] <= 9u) ? g_bt_v[2] : 1u; break;
                        case 3u: g_pid_ki  = g_bt_v[2]; break;
                        case 4u: g_pid_mi  = (g_bt_v[2] >= 1u && g_bt_v[2] <= 9u) ? g_bt_v[2] : 1u; break;
                        case 5u: g_pid_kd  = g_bt_v[2]; break;
                        case 6u: g_pid_md  = (g_bt_v[2] >= 1u && g_bt_v[2] <= 9u) ? g_bt_v[2] : 1u; break;
                        case 7u:
                            if (mode == DRIVE_MODE_LINE_PID) {
                                auto_run_enabled = (g_bt_v[2] != 0u) && controls_on && !safe.active;
                                if (!auto_run_enabled) platform_motor_stop();
                            }
                            break;
                        default: break;
                    }
                    continue;
                }
                // First byte of a new PID frame (control chars 1-7)
                if (((uint8_t)c >= 1u) && ((uint8_t)c <= 7u)) {
                    g_bt_cnt  = 1u;
                    g_bt_v[1] = (uint8_t)c;
                    continue;
                }

                {
                    int16_t speed_before = current_speed;
                    if (try_parse_arrow_speed_char(c, controls_on, mode, &current_speed)) {
                        if (current_speed != speed_before)
                            refresh_ui(controls_on, mode, current_speed);
                        continue;
                    }
                }

                if ((c == 'M') || (c == 'm')) {
                    bool was_auto_ir  = (mode == DRIVE_MODE_AUTO_IR);
                    bool was_line_pid = (mode == DRIVE_MODE_LINE_PID);
                    mode              = DRIVE_MODE_MANUAL;
                    active_move_key  = 0;
                    last_cmd_ms      = now;
                    auto_run_enabled = false;
                    ultra_fail_streak = 0u;
                    platform_motor_stop();
                    if (was_auto_ir) {
                        last_nonzero_sum        = 0;
                        ir_debug_stream_enabled = false;
                        ir_black_history_count  = 0u;
                        ir_black_history_index  = 0u;
                        ir_crawl_since_ms       = 0u;
                        ir_smooth_reset(0, 0);
                        for (uint8_t idx = 0u; idx < IR_SAMPLE_HISTORY_SIZE; idx++)
                            ir_black_history[idx] = 0u;
                    }
                    if (was_line_pid) {
                        g_pid_I          = 0;
                        g_pid_prev_err   = 0;
                        g_bt_cnt         = 0u;
                        ir_crawl_since_ms          = 0u;
                        ir_smooth_reset(0, 0);
                    }
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'U') || (c == 'u')) {
                    mode             = DRIVE_MODE_AUTO_IR;
                    active_move_key  = 0;
                    last_cmd_ms      = 0u;
                    auto_run_enabled = controls_on && !safe.active;
                    ultra_fail_streak          = 0u;
                    last_nonzero_sum           = 0;
                    ir_debug_stream_enabled    = true;
                    ir_black_history_count     = 0u;
                    ir_black_history_index     = 0u;
                    ir_crawl_since_ms          = 0u;
                    ir_smooth_reset(0, 0);
                    ir_pid_reset();
                    for (uint8_t idx = 0u; idx < IR_SAMPLE_HISTORY_SIZE; idx++)
                        ir_black_history[idx] = 0u;
                    platform_motor_stop();
                    if (!controls_on)
                        platform_usart_write_str("IR: controls OFF — press button first\r\n");
                    else if (safe.active)
                        platform_usart_write_str("IR: SAFE active — send X to clear\r\n");
                    else {
                        platform_usart_write_str("IR: auto follow RUNNING\r\n");
                        platform_usart_write_str("IR: debug stream ON\r\n");
                    }
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'O') || (c == 'o')) {
                    mode             = DRIVE_MODE_AUTO_ULTRASONIC;
                    active_move_key  = 0;
                    auto_run_enabled = controls_on && !safe.active;
                    ultra_fail_streak = 0u;
                    ultra_no_path_streak = 0u;
                    ultra_alert_count[0] = 0u;
                    ultra_alert_count[1] = 0u;
                    ultra_alert_count[2] = 0u;
                    if (auto_run_enabled)
                        last_ultra_ms = now - ULTRA_POLL_MS;
                    platform_motor_stop();
                    if (!controls_on)
                        platform_usart_write_str("US: controls OFF — press button first\r\n");
                    else if (safe.active)
                        platform_usart_write_str("US: SAFE active — send X to clear\r\n");
                    else
                        platform_usart_write_str("US: auto maze RUNNING\r\n");
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'L') || (c == 'l')) {
                    mode             = DRIVE_MODE_LINE_PID;
                    active_move_key  = 0;
                    auto_run_enabled = controls_on && !safe.active;
                    g_pid_I          = 0;
                    g_pid_prev_err   = 0;
                    g_bt_cnt         = 0u;
                    ir_crawl_since_ms          = 0u;
                    ir_smooth_reset(0, 0);
                    platform_motor_stop();
                    if (!controls_on)
                        platform_usart_write_str("PID: controls OFF — press button first\r\n");
                    else if (safe.active)
                        platform_usart_write_str("PID: SAFE active — send X to clear\r\n");
                    else
                        platform_usart_write_str("PID: line PID RUNNING\r\n");
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'N') || (c == 'n')) {
                    if (!controls_on) {
                        platform_usart_write_str("NAV: controls OFF — press button first\r\n");
                    } else {
                        mode             = DRIVE_MODE_ROBOT_NAV;
                        active_move_key  = 0;
                        auto_run_enabled = false;
                        platform_motor_stop();
                        refresh_ui(controls_on, mode, current_speed);
                        platform_usart_write_str("NAV: running — press button or send [SPC/N/B] to stop\r\n");
                        robot_run();
                        /* Restore manual mode after robot_run() returns */
                        mode            = DRIVE_MODE_MANUAL;
                        active_move_key = 0;
                        last_rx_ms      = platform_millis();
                        refresh_ui(controls_on, mode, current_speed);
                    }
                    continue;
                }

                if ((c == 'Z') || (c == 'z')) {
                    if (mode == DRIVE_MODE_AUTO_IR) {
                        ir_debug_stream_enabled = !ir_debug_stream_enabled;
                        platform_usart_write_str(ir_debug_stream_enabled
                            ? "IR: debug stream ON\r\n"
                            : "IR: debug stream OFF\r\n");
                    }
                    if (mode == DRIVE_MODE_LINE_PID) {
                        line_pid_safe_enabled = !line_pid_safe_enabled;
                        platform_usart_write_str(line_pid_safe_enabled
                            ? "PID: safe mode ON\r\n"
                            : "PID: safe mode OFF\r\n");
                    }
                    continue;
                }

                /* Keyboard alternative to the onboard button: toggle controls ON/OFF */
                if ((c == 'B') || (c == 'b')) {
                    if (!controls_on) {
                        platform_usart_write_str("SER: 'b' pressed — Controls ENABLED. Select mode: M / U / O\r\n");
                        system_set_on(&controls_on);
                        current_speed     = BUTTON_ON_SPEED_CMD;
                        auto_run_enabled  = !safe.active &&
                                           ((mode == DRIVE_MODE_AUTO_IR)        ||
                                            (mode == DRIVE_MODE_AUTO_ULTRASONIC) ||
                                            (mode == DRIVE_MODE_LINE_PID));
                        if ((mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled) {
                            last_ultra_ms        = now - ULTRA_POLL_MS;
                            ultra_no_path_streak = 0u;
                            ultra_alert_count[0] = 0u;
                            ultra_alert_count[1] = 0u;
                            ultra_alert_count[2] = 0u;
                        }
                        last_rx_ms        = now;
                        ultra_fail_streak = 0u;
                        active_move_key   = 0;
                        /* Prevent immediate OFF while the same press is still held. */
                        button_hold_fired = true;
                        button_press_ms   = now;
                        refresh_ui(controls_on, mode, current_speed);
                    } else {
                        platform_usart_write_str("SER: 'b' pressed — Controls DISABLED. Motors stopped.\r\n");
                        system_set_off(&controls_on);
                        auto_run_enabled  = false;
                        safe.active       = false;
                        safe.reason       = SAFE_REASON_NONE;
                        safe.entered_ms   = 0u;
                        ultra_fail_streak = 0u;
                        active_move_key   = 0;
                        refresh_ui(controls_on, mode, current_speed);
                    }
                    continue;
                }

                c = to_lower(c);
                if (!controls_on) continue;

                if (c == SAFE_CLEAR_KEY) {
                    if (safe.active) {
                        safe_clear(&safe);
                        ir_crawl_since_ms          = 0u;
                        ir_smooth_reset(0, 0);
                        refresh_ui(controls_on, mode, current_speed);
                    }
                    continue;
                }

                if (c == ' ') {
                    platform_motor_stop();
                    ir_smooth_reset(0, 0);
                    g_ramp_state.target_left   = 0;
                    g_ramp_state.target_right  = 0;
                    g_ramp_state.current_left  = 0;
                    g_ramp_state.current_right = 0;
                    active_move_key = 0;
                    if ((mode == DRIVE_MODE_AUTO_IR) || (mode == DRIVE_MODE_AUTO_ULTRASONIC)) {
                        auto_run_enabled = false;
                        platform_usart_write_str("AUTO: paused — send U or O to resume\r\n");
                    }
                    if (mode == DRIVE_MODE_LINE_PID) {
                        auto_run_enabled = false;
                        g_pid_I = 0;
                        platform_usart_write_str("PID: paused — send L to resume\r\n");
                    }
                    continue;
                }

                if (safe.active) continue;

                if ((mode == DRIVE_MODE_AUTO_IR) &&
                    ((c=='i')||(c=='p')||(c=='n')||(c=='b')||
                     ((c>='1')&&(c<='3')))) {
                    platform_usart_write_str("IR: live tuning disabled; fixed settings are active\r\n");
                    continue;
                }

                if (mode != DRIVE_MODE_MANUAL) continue;

                // ===== HARDWARE DIAGNOSTIC =====
                if (c == 't') {
                    uint32_t ts;
                    platform_usart_write_str("MOTOR TEST: Left Forward\r\n");
                    platform_motor_set(+500, 0); ts = platform_millis();
                    while ((platform_millis() - ts) < 1000) { asm("nop"); }
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Left Reverse\r\n");
                    platform_motor_set(-500, 0); ts = platform_millis();
                    while ((platform_millis() - ts) < 1000) { asm("nop"); }
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Right Forward\r\n");
                    platform_motor_set(0, +500); ts = platform_millis();
                    while ((platform_millis() - ts) < 1000) { asm("nop"); }
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Right Reverse\r\n");
                    platform_motor_set(0, -500); ts = platform_millis();
                    while ((platform_millis() - ts) < 1000) { asm("nop"); }
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Complete\r\n");
                    continue;
                }

                // ===== MANUAL DRIVE =====
                switch (c)
                {
                    case 'w':
                        active_move_key = 'w'; last_cmd_ms = now;
                        set_motor_with_ramp(+current_speed, +current_speed); break;
                    case 's':
                        active_move_key = 's'; last_cmd_ms = now;
                        set_motor_with_ramp(-current_speed, -current_speed); break;
                    case 'a':
                        active_move_key = 'a'; last_cmd_ms = now;
                        apply_gentle_turn_left(current_speed); break;
                    case 'd':
                        active_move_key = 'd'; last_cmd_ms = now;
                        apply_gentle_turn_right(current_speed); break;
                    default: break;
                }
            }
        }

        // ===== SAFE MODE: COMM LOSS CHECK =====
        if (controls_on && !safe.active && auto_run_enabled &&
            (mode == DRIVE_MODE_AUTO_ULTRASONIC)) {
            if ((now - last_rx_ms) > SAFE_COMM_LOSS_MS)
                safe_enter(&safe, SAFE_REASON_COMM_LOSS, now, &auto_run_enabled);
        }

        // ===== ULTRASONIC MAZE LOOP =====
        //
        // Double-wall corridor following (track ~29 cm wide):
        //   1. Read all three sensors (median-of-3 each, no extra inter-sensor delay).
        //   2. Convert front cm → state 0/1/2 with 2-reading hysteresis.
        //      Side cm values are passed raw to the PID; state used only for
        //      the dead-end / no-path streak check.
        //   3. Decide action:
        //        F >= 1 (wall present) → turn toward open side (right preferred).
        //        F == 0               → FORWARD with PID centering between walls.
        //   4. FORWARD is non-blocking; PID runs every poll cycle.
        //      Turns are blocking pivot calls that reset the PID state on exit.
        //   5. All-blocked streak → 180° turn after ULTRA_NO_PATH_LIMIT cycles.
        //
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled)
        {
            if ((now - last_ultra_ms) >= ULTRA_POLL_MS)
            {
                /*
                 * Read all three sensors back-to-back with no extra delays
                 * between them. Each ultra_read_stable_cm() already waits
                 * ULTRA_SAMPLE_GAP_MS between its three internal pings, so
                 * adding more delay here was the main cause of slow reaction.
                 * Total blocking time ≈ 3 × (2 × ULTRA_SAMPLE_GAP_MS + ping) ≈ 45–60 ms.
                 */
                bool ok_f = ultra_read_stable_cm(ULTRA_FRONT, &ultra_front_cm);
                bool ok_l = ultra_read_stable_cm(ULTRA_LEFT,  &ultra_left_cm);
                bool ok_r = ultra_read_stable_cm(ULTRA_RIGHT, &ultra_right_cm);

                // Count streak on ANY failure; reset only when all three pass
                if (!ok_f || !ok_l || !ok_r) {
                    if (ultra_fail_streak < 255u) ultra_fail_streak++;
                } else {
                    ultra_fail_streak = 0u;
                }

                if (!ok_f && !ok_l && !ok_r)
                {
                    platform_motor_stop();
                    platform_usart_write_str("US: read timeout\r\n");
                    if (ultra_fail_streak >= SAFE_ULTRA_FAIL_LIMIT) {
                        if (ultra_safe_enabled) {
                            safe_enter(&safe, SAFE_REASON_ULTRA_TIMEOUT, now, &auto_run_enabled);
                        } else {
                            platform_usart_write_str("US: fail limit — SAFE disabled\r\n");
                        }
                    }
                    last_ultra_ms = now;
                    continue;
                }

                // Partial failure: at least one sensor timed out repeatedly
                if (ultra_safe_enabled && (ultra_fail_streak >= SAFE_ULTRA_FAIL_LIMIT))
                {
                    platform_usart_write_str("US: partial sensor fail limit — entering SAFE\r\n");
                    safe_enter(&safe, SAFE_REASON_ULTRA_TIMEOUT, now, &auto_run_enabled);
                    last_ultra_ms = now;
                    continue;
                }

                // Convert cm readings to states with hysteresis
                uint8_t st_f = ultra_apply_hysteresis(
                                   ultra_cm_to_raw_state(ultra_front_cm, ok_f),
                                   &ultra_alert_count[0]);
                uint8_t st_l = ultra_apply_hysteresis(
                                   ultra_cm_to_raw_state(ultra_left_cm,  ok_l),
                                   &ultra_alert_count[1]);
                uint8_t st_r = ultra_apply_hysteresis(
                                   ultra_cm_to_raw_state(ultra_right_cm, ok_r),
                                   &ultra_alert_count[2]);

                // Increment when all three directions are blocked (genuine dead end)
                if ((st_f != 0u) && (st_l != 0u) && (st_r != 0u))
                    ultra_no_path_streak++;
                else
                    ultra_no_path_streak = 0u;

                if (ultra_no_path_streak >= ULTRA_NO_PATH_LIMIT)
                {
                    /*
                    * A dead end is not a sensor failure.
                    * Turn around instead of entering SAFE mode.
                    */
                    platform_usart_write_str("US: dead end - turning around\r\n");

                    apply_ultrasonic_action(ULTRA_ACT_TURN_180, ultra_left_cm, ultra_right_cm);

                    ultra_no_path_streak = 0u;
                    last_ultra_ms = now;
                    continue;
                }

                ultra_action_t ua = decide_ultrasonic_action(st_f, st_l, st_r);
                print_ultrasonic_status(ultra_front_cm, st_f,
                                        ultra_left_cm,  st_l,
                                        ultra_right_cm, st_r, ua);
                apply_ultrasonic_action(ua, ultra_left_cm, ultra_right_cm);

                last_ultra_ms = now;
            }
        }

        // ===== IR AUTO LOOP =====
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_AUTO_IR) && auto_run_enabled) {
            if ((now - last_auto_ms) >= AUTO_LOOP_MS) {
                uint8_t raw_mask   = platform_ir_read_mask_raw();
                uint8_t black_mask = ir_mask_on_black_runtime(raw_mask, ir_active_on_black_high);
                uint8_t filtered_black_mask;
                int8_t  sum   = 0;
                uint8_t count = 0u;

                ir_black_history[ir_black_history_index] = black_mask;
                ir_black_history_index = (uint8_t)(
                    (ir_black_history_index + 1u) % IR_SAMPLE_HISTORY_SIZE);
                if (ir_black_history_count < IR_SAMPLE_HISTORY_SIZE)
                    ir_black_history_count++;

                filtered_black_mask = ir_mask_or_history(ir_black_history,
                                                          ir_black_history_count);

                auto_action_t act = decide_auto_action(black_mask,
                                                        filtered_black_mask,
                                                        ir_turn_threshold,
                                                        ir_min_black_count,
                                                        &sum, &count);

                if ((sum <= -2) || (sum >= 2)) last_nonzero_sum = sum;

                if (act == AUTO_ACT_CRAWL) {
                    if (ir_crawl_since_ms == 0u) {
                        ir_crawl_since_ms = now;
                    } else if ((now - ir_crawl_since_ms) >= SAFE_IR_CRAWL_MS) {
                        safe_enter(&safe, SAFE_REASON_LINE_LOST, now, &auto_run_enabled);
                        ir_smooth_reset(0, 0);
                        ir_crawl_since_ms = 0u;
                        last_auto_ms = now;
                        continue;
                    }
                } else {
                    ir_crawl_since_ms = 0u;
                }

                apply_auto_action(act, current_speed, sum, last_nonzero_sum);

                if (ir_debug_stream_enabled)
                    print_ir_debug_status(raw_mask, black_mask, sum, count, act);

                last_auto_ms = now;
            }
        }

        // ===== LINE PID LOOP =====
        //
        // Each iteration:
        //   1. Read digital IR mask → weighted position (0..4000).
        //   2. error = 2000 - position  (0 = centred).
        //   3. All sensors white → off-line recovery spin toward last error.
        //      If off-line for > SAFE_IR_CRAWL_MS → enter SAFE mode.
        //   4. Otherwise → PID → platform_motor_set().
        //
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_LINE_PID) && auto_run_enabled)
        {
            if ((now - last_auto_ms) >= LINE_PID_LOOP_MS)
            {
                uint8_t raw_mask   = platform_ir_read_mask_raw();
                uint8_t black_mask = ir_mask_on_black_runtime(raw_mask, ir_active_on_black_high);

                if (black_mask == 0u)
                {
                    // Off-line recovery
                    if (ir_crawl_since_ms == 0u) {
                        ir_crawl_since_ms = now;
                    } else if ((now - ir_crawl_since_ms) >= SAFE_IR_CRAWL_MS) {
                        if (line_pid_safe_enabled) {
                            safe_enter(&safe, SAFE_REASON_LINE_LOST, now, &auto_run_enabled);
                            ir_smooth_reset(0, 0);
                            ir_crawl_since_ms = 0u;
                            last_auto_ms = now;
                            continue;
                        }
                        ir_crawl_since_ms = now;   // safe OFF: reset timer, keep spinning
                    }

                    // Spin toward the side where the line was last seen
                    if (g_pid_prev_err > 0) {
                        platform_motor_set(-(int16_t)LINE_PID_RECOVERY_SPEED,
                                           +(int16_t)LINE_PID_RECOVERY_SPEED);
                    } else {
                        platform_motor_set(+(int16_t)LINE_PID_RECOVERY_SPEED,
                                           -(int16_t)LINE_PID_RECOVERY_SPEED);
                    }
                }
                else
                {
                    ir_crawl_since_ms = 0u;
                    uint16_t position = line_position_from_mask(black_mask);
                    int16_t  error    = (int16_t)(2000 - (int32_t)position);
                    line_pid_apply(error, g_pid_lfspd);
                }

                last_auto_ms = now;
            }
        }

        // ===== MANUAL CMD TIMEOUT =====
        if (controls_on && (mode == DRIVE_MODE_MANUAL) && (active_move_key != 0)) {
            if ((now - last_cmd_ms) >= CMD_TIMEOUT_MS) {
                platform_motor_stop();
                active_move_key = 0;
            }
        }
    }
}

