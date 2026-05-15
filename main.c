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

#define DEFAULT_SPEED_CMD         180
#define BUTTON_ON_SPEED_CMD       180
#define MIN_SPEED_CMD             0
#define MAX_SPEED_CMD             1000
#define SPEED_STEP_CMD            100

// ===== MANUAL CONTROL PARAMETERS =====
#define TURN_SENSITIVITY_PERCENT  100
#define TURN_INNER_PERCENT        0

#define RAMP_ENABLED              0
#define RAMP_STEP_PER_CYCLE       50
#define RAMP_CYCLE_MS             20

#define TURN_MODE_GENTLE_ARC      0
#define TURN_MODE_PIVOT           1

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
#define ULTRA_POLL_MS             60u    // ms between sensor reads
#define ULTRA_OPEN_CM             22u    // > this = open path (state 0)
#define ULTRA_ALERT_CM            5u     // <= this = wall too close (state 2)
#define ULTRA_FORWARD_SPEED       260    // PWM for forward movement
#define ULTRA_TURN_SPEED          300    // PWM for pivot turns
#define ULTRA_TURN_90_MS          520u   // duration of a 90-degree pivot
#define ULTRA_TURN_180_MS         1040u  // duration of a 180-degree pivot
#define ULTRA_STOP_BEFORE_TURN_MS 120u   // brief stop before each turn
#define ULTRA_REVERSE_MS          200u   // back-up duration on front wall alert (state 2)
#define ULTRA_ALERT_COUNT_THRESH  2u     // consecutive close readings for state 2
#define ULTRA_NO_PATH_LIMIT       50u    // consecutive all-zero reads before safe (~3s at 60ms)
#define ULTRA_MAX_VALID_CM        300u   // readings >= this are treated as no echo

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
#define IR_PID_KP                    16
#define IR_PID_KI                     1
#define IR_PID_KD                    13
#define IR_PID_INTEGRAL_LIMIT      12000
#define IR_PID_OUTPUT_LIMIT           300
#define IR_PID_MIN_ACTIVE_SPEED        55

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
#define LINE_PID_LFSPEED_DEFAULT    200
#define LINE_PID_LFSPEED_MIN        10
#define LINE_PID_LFSPEED_MAX        255
#define LINE_PID_SPEED_STEP         10
#define LINE_PID_CORRECTION_LIMIT   255    // full range; inner wheel can reverse on sharp turns
#define LINE_PID_I_LIMIT            5000
#define LINE_PID_RECOVERY_SPEED     602   // ≈ 230/255 × 1000, platform scale

static uint8_t  g_pid_kp      = 0u;
static uint8_t  g_pid_ki      = 0u;
static uint8_t  g_pid_kd      = 0u;
static uint8_t  g_pid_mp      = 1u;   // multiP: divide Kp by 10^multiP
static uint8_t  g_pid_mi      = 1u;
static uint8_t  g_pid_md      = 1u;
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
"||  [+]/[-] = Speed +/- 10           ||\r\n"
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
    bool line_pid = (mode == DRIVE_MODE_LINE_PID);
    if (!on || ((mode != DRIVE_MODE_MANUAL) && (mode != DRIVE_MODE_AUTO_IR) && !line_pid)) {
        esc_state = 0u; return false;
    }
    if (c == 0x1B) { esc_state = 1u; return true; }
    if (esc_state == 1u) {
        if (c == '[') { esc_state = 2u; return true; }
        esc_state = 0u; return false;
    }
    if (esc_state == 2u) {
        esc_state = 0u;
        if (c == 'A') {
            if (line_pid) { int32_t v = (int32_t)*speed + LINE_PID_SPEED_STEP; *speed = (int16_t)(v > LINE_PID_LFSPEED_MAX ? LINE_PID_LFSPEED_MAX : v); }
            else          { *speed = clamp_speed((int32_t)*speed + SPEED_STEP_CMD); }
            return true;
        }
        if (c == 'B') {
            if (line_pid) { int32_t v = (int32_t)*speed - LINE_PID_SPEED_STEP; *speed = (int16_t)(v < LINE_PID_LFSPEED_MIN ? LINE_PID_LFSPEED_MIN : v); }
            else          { *speed = clamp_speed((int32_t)*speed - SPEED_STEP_CMD); }
            return true;
        }
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

static uint8_t ultra_cm_to_raw_state(uint16_t cm, bool read_ok)
{
    if (!read_ok)                                              return 1u;  // sensor fail = blocked, not open
    if ((cm == 0u) || (cm >= ULTRA_MAX_VALID_CM))              return 0u;  // no echo = open path
    if (cm > ULTRA_OPEN_CM)                                    return 0u;
    if (cm <= ULTRA_ALERT_CM)                                  return 2u;
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
    bool front_open = (F == 0u);
    bool left_open  = (L == 0u);
    bool right_open = (R == 0u);

    // Front wall alert: back up to create clearance; next cycle re-evaluates
    if (F == 2u)    return ULTRA_ACT_REVERSE;

    if (front_open) {
        // Steer away from a side wall that is dangerously close while moving forward
        if (L == 2u) return ULTRA_ACT_FORWARD_LEAN_R;
        if (R == 2u) return ULTRA_ACT_FORWARD_LEAN_L;
        return ULTRA_ACT_FORWARD;
    }

    if (right_open) return ULTRA_ACT_TURN_RIGHT;
    if (left_open)  return ULTRA_ACT_TURN_LEFT;
    return ULTRA_ACT_TURN_180;
}

static void ultra_pivot(int16_t left_spd, int16_t right_spd, uint32_t duration_ms)
{
    uint32_t t;
    platform_motor_stop();
    t = platform_millis();
    while ((platform_millis() - t) < ULTRA_STOP_BEFORE_TURN_MS) { asm("nop"); }
    platform_motor_set(left_spd, right_spd);
    t = platform_millis();
    while ((platform_millis() - t) < duration_ms) { asm("nop"); }
    platform_motor_stop();
}

static void apply_ultrasonic_action(ultra_action_t act)
{
    switch (act)
    {
        case ULTRA_ACT_FORWARD:
            platform_motor_set(ULTRA_FORWARD_SPEED, ULTRA_FORWARD_SPEED);
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
            ultra_pivot(+ULTRA_TURN_SPEED, -ULTRA_TURN_SPEED, ULTRA_TURN_90_MS);
            break;
        case ULTRA_ACT_TURN_LEFT:
            ultra_pivot(-ULTRA_TURN_SPEED, +ULTRA_TURN_SPEED, ULTRA_TURN_90_MS);
            break;
        case ULTRA_ACT_TURN_180:
            ultra_pivot(-ULTRA_TURN_SPEED, +ULTRA_TURN_SPEED, ULTRA_TURN_180_MS);
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
    if (PIDvalue >  LINE_PID_CORRECTION_LIMIT) PIDvalue =  LINE_PID_CORRECTION_LIMIT;
    if (PIDvalue < -LINE_PID_CORRECTION_LIMIT) PIDvalue = -LINE_PID_CORRECTION_LIMIT;

    g_pid_prev_err = error;

    left_speed  = (int32_t)lfspeed - PIDvalue;
    right_speed = (int32_t)lfspeed + PIDvalue;

    {
        int32_t abs_l   = (left_speed  < 0L) ? -left_speed  : left_speed;
        int32_t abs_r   = (right_speed < 0L) ? -right_speed : right_speed;
        int32_t max_abs = (abs_l > abs_r) ? abs_l : abs_r;
        if (max_abs > 255L) {
            left_speed  = (left_speed  * 255L) / max_abs;
            right_speed = (right_speed * 255L) / max_abs;
        }
    }

    // Scale from spec's ±255 to platform's ±1000
    platform_motor_set(
        (int16_t)((left_speed  * 1000L) / 255L),
        (int16_t)((right_speed * 1000L) / 255L)
    );
}

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
                    int16_t *speed_ptr   = (mode == DRIVE_MODE_LINE_PID) ? &g_pid_lfspd : &current_speed;
                    int16_t  speed_before = *speed_ptr;
                    if (try_parse_arrow_speed_char(c, controls_on, mode, speed_ptr)) {
                        if (*speed_ptr != speed_before) {
                            if (mode == DRIVE_MODE_LINE_PID) {
                                char buf[18];
                                uint32_t i = 0u;
                                uint16_t v = (uint16_t)g_pid_lfspd;
                                buf[i++]='P';buf[i++]='I';buf[i++]='D';buf[i++]=':';buf[i++]=' ';
                                buf[i++]='s';buf[i++]='p';buf[i++]='d';buf[i++]='=';
                                if (v >= 100u) buf[i++]=(char)('0'+v/100u);
                                if (v >= 10u)  buf[i++]=(char)('0'+(v/10u)%10u);
                                buf[i++]=(char)('0'+v%10u);
                                buf[i++]='\r'; buf[i++]='\n';
                                platform_usart_write_buf(buf, i);
                            } else {
                                refresh_ui(controls_on, mode, current_speed);
                            }
                        }
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

                if (mode == DRIVE_MODE_LINE_PID) {
                    if ((c == '+') || (c == '-')) {
                        int32_t v = (int32_t)g_pid_lfspd +
                                    ((c == '+') ? LINE_PID_SPEED_STEP : -LINE_PID_SPEED_STEP);
                        if (v > LINE_PID_LFSPEED_MAX) v = LINE_PID_LFSPEED_MAX;
                        if (v < LINE_PID_LFSPEED_MIN) v = LINE_PID_LFSPEED_MIN;
                        g_pid_lfspd = (int16_t)v;
                        char buf[18];
                        uint32_t i = 0u;
                        uint16_t sv = (uint16_t)g_pid_lfspd;
                        buf[i++]='P';buf[i++]='I';buf[i++]='D';buf[i++]=':';buf[i++]=' ';
                        buf[i++]='s';buf[i++]='p';buf[i++]='d';buf[i++]='=';
                        if (sv >= 100u) buf[i++]=(char)('0'+sv/100u);
                        if (sv >= 10u)  buf[i++]=(char)('0'+(sv/10u)%10u);
                        buf[i++]=(char)('0'+sv%10u);
                        buf[i++]='\r'; buf[i++]='\n';
                        platform_usart_write_buf(buf, i);
                        continue;
                    }
                }

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
        // Each cycle:
        //   1. Read all three sensors.
        //   2. Convert raw cm → state 0/1/2 with per-sensor alert hysteresis.
        //   3. Decide action using the 9-case wall-following table.
        //   4. Apply: FORWARD is non-blocking; turns are blocking pivot calls.
        //   5. Track all-zero streak → safe mode after ULTRA_NO_PATH_LIMIT cycles.
        //
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled)
        {
            if ((now - last_ultra_ms) >= ULTRA_POLL_MS)
            {
                bool ok_f = platform_ultrasonic_read_cm(ULTRA_FRONT, &ultra_front_cm);
                bool ok_l = platform_ultrasonic_read_cm(ULTRA_LEFT,  &ultra_left_cm);
                bool ok_r = platform_ultrasonic_read_cm(ULTRA_RIGHT, &ultra_right_cm);

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
                    if (ultra_safe_enabled) {
                        platform_usart_write_str("US: no valid path — entering SAFE\r\n");
                        safe_enter(&safe, SAFE_REASON_ULTRA_TIMEOUT, now, &auto_run_enabled);
                        last_ultra_ms = now;
                        continue;
                    } else {
                        platform_usart_write_str("US: no valid path — SAFE disabled, stopping motors\r\n");
                        platform_motor_stop();
                        last_ultra_ms = now;
                        continue;
                    }
                }

                ultra_action_t ua = decide_ultrasonic_action(st_f, st_l, st_r);
                print_ultrasonic_status(ultra_front_cm, st_f,
                                        ultra_left_cm,  st_l,
                                        ultra_right_cm, st_r, ua);
                apply_ultrasonic_action(ua);

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