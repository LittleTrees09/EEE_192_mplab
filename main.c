// DO NOT REMOVE THIS COMMENT
// Microcontroller Unit: PIC32CM5164LS00064

#include "platform.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS               30u
#define AUTO_LOOP_MS              8u
#define CMD_TIMEOUT_MS            250u

#define DEFAULT_SPEED_CMD         300
#define BUTTON_ON_SPEED_CMD       100
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

// ===== ULTRASONIC PARAMETERS =====
#define ULTRA_POLL_MS             80u
#define ULTRA_FRONT_STOP_CM       25u
#define ULTRA_FRONT_CAUTION_CM    40u
#define ULTRA_SIDE_CLOSE_CM       20u
#define ULTRA_SIDE_MEDIUM_CM      35u
#define ULTRA_FORWARD_SPEED       400
#define ULTRA_SLOW_SPEED          200
#define ULTRA_TURN_SPEED          300
#define ULTRA_TURN_DURATION_MS    500u
#define ULTRA_SIDE_DIFF_MIN_CM    5u
#define ULTRA_MAX_VALID_CM        300u

// ===== SAFE MODE PARAMETERS =====
#define SAFE_COMM_LOSS_MS         10000u
#define SAFE_ULTRA_FAIL_LIMIT     3u
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

#define IR_NO_DETECT_CRAWL_ON_EMPTY 1u
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
#define IR_PID_KP                    24
#define IR_PID_KI                     1
#define IR_PID_KD                    18
#define IR_PID_INTEGRAL_LIMIT      12000
#define IR_PID_OUTPUT_LIMIT           450
#define IR_PID_MIN_ACTIVE_SPEED        55

static int32_t g_ir_pid_integral = 0;
static int16_t g_ir_pid_last_error = 0;
static bool    g_ir_pid_initialized = false;

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

// Output smoother — FORWARD only. LEFT/RIGHT bypass for instant response.
// Reduced from 7/3 to 5/5 for faster response (50/50 weighting instead of 70/30)
#define IR_SMOOTH_OLD_WEIGHT          5
#define IR_SMOOTH_NEW_WEIGHT          5
#define IR_SMOOTH_TOTAL              (IR_SMOOTH_OLD_WEIGHT + IR_SMOOTH_NEW_WEIGHT)

// ===== JUNCTION PARAMETERS =====
// How the robot handles wide horizontal bars / T-junctions / crosses:
//
//   COAST phase  (~144 ms): robot drives straight through at reduced speed.
//   COOLDOWN phase (~80 ms): coast has finished but junction detection is
//     suppressed. If the sensors still see wide black (robot is still on or
//     near the bar), the robot is forced straight rather than re-triggering
//     a new coast. This prevents the re-trigger loop that caused the robot
//     to stall on wide intersection bars.
//
// Total suppression window = COAST + COOLDOWN = 28 cycles = ~224 ms.
// Tune IR_JUNCTION_COAST_CYCLES if the bar is cleared too early/late.
// Tune IR_JUNCTION_COOLDOWN_CYCLES if the robot re-triggers after clearing.
#define IR_JUNCTION_COAST_CYCLES     18u   // ~144 ms at 8 ms/loop
#define IR_JUNCTION_COOLDOWN_CYCLES  10u   // ~80 ms — suppresses re-trigger after coast
#define IR_JUNCTION_SPEED_PERCENT    70

#define IR_BIFURCATION_PREFER_LEFT    1
#define IR_BIFURCATION_OUTER_PERCENT 90

typedef enum
{
    DRIVE_MODE_MANUAL = 0,
    DRIVE_MODE_AUTO_IR,
    DRIVE_MODE_AUTO_ULTRASONIC
} drive_mode_t;

typedef enum
{
    AUTO_ACT_STOP = 0,
    AUTO_ACT_FORWARD,
    AUTO_ACT_LEFT,
    AUTO_ACT_RIGHT,
    AUTO_ACT_CRAWL,
    AUTO_ACT_JUNCTION,
    AUTO_ACT_BIFURCATION_LEFT,
    AUTO_ACT_BIFURCATION_RIGHT
} auto_action_t;

typedef enum
{
    ULTRA_ACT_STOP = 0,
    ULTRA_ACT_FORWARD,
    ULTRA_ACT_LEFT,
    ULTRA_ACT_RIGHT,
    ULTRA_ACT_TURN_LEFT,
    ULTRA_ACT_TURN_RIGHT
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

// Junction state — two-phase system:
//   g_junction_coast_cycles   : counts down during the active coast
//   g_junction_cooldown_cycles: counts down after coast expires to block re-triggers
static uint8_t g_junction_coast_cycles    = 0u;
static uint8_t g_junction_cooldown_cycles = 0u;

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
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_MANUAL_HEAD[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[36mMANUAL\033[0m                    ||\r\n"
"||  Baud:   9600 or 3840              ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
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
"||  Baud:   9600 or 3840              ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
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
"||                                    ||\r\n"
"========================================\r\n";

static const char UI_AUTO_ULTRA[] =
"\033[2J\033[H"
"========================================\r\n"
"||  EEE 192 MoBot Control             ||\r\n"
"||                                    ||\r\n"
"||  STATUS: \033[32mON\033[0m                        ||\r\n"
"||  MODE:   \033[35mAUTO ULTRASONIC\033[0m           ||\r\n"
"||  Baud:   9600 or 3840              ||\r\n"
"||                                    ||\r\n"
"||  Mode Commands:                    ||\r\n"
"||    [M] = Manual mode               ||\r\n"
"||    [U] = Auto IR follow mode       ||\r\n"
"||    [O] = Auto ultrasonic avoid     ||\r\n"
"||                                    ||\r\n"
"||  Ultrasonic Auto Behavior:         ||\r\n"
"||    * Path clear   -> full speed    ||\r\n"
"||    * Approaching  -> slow down     ||\r\n"
"||    * Too close    -> stop + turn   ||\r\n"
"||    * Side blocked -> steer away    ||\r\n"
"||                                    ||\r\n"
"||  Sensor Configuration:             ||\r\n"
"||    FRONT: Stop    < 25cm           ||\r\n"
"||    FRONT: Caution < 40cm           ||\r\n"
"||    SIDES: Close   < 20cm           ||\r\n"
"||    SIDES: Medium  < 35cm           ||\r\n"
"||                                    ||\r\n"
"||  [SPC] = Stop motors               ||\r\n"
"||  Send [O] again to resume.         ||\r\n"
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

// =============================================================================
// decide_auto_action()
//
// Priority order:
//
//   1. Junction COAST  — actively driving straight through an intersection.
//      When the coast counter expires it arms the COOLDOWN counter.
//
//   2. Junction COOLDOWN — coast has finished but junction detection is
//      suppressed for IR_JUNCTION_COOLDOWN_CYCLES more cycles.
//      If sensors still show 4+ black during cooldown the robot is forced
//      FORWARD (straight) — it does NOT re-trigger a new coast.
//      This solves the stall loop on wide bars like the ones in Image 1.
//
//   3. Junction DETECT — 4+ sensors black and no cooldown active.
//      Arms the coast counter and returns AUTO_ACT_JUNCTION.
//
//   4. Bifurcation — both outer wings active, centre clear = split path.
//
//   5. Normal proportional line following.
// =============================================================================
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

    // -------------------------------------------------------------------------
    // Priority 1: COAST — robot is actively crossing an intersection.
    // Drive straight (sum forced to 0). When counter hits zero, arm cooldown.
    // -------------------------------------------------------------------------
    if (g_junction_coast_cycles > 0u)
    {
        g_junction_coast_cycles--;
        if (g_junction_coast_cycles == 0u)
        {
            // Coast just expired — start the cooldown to block re-triggers.
            g_junction_cooldown_cycles = IR_JUNCTION_COOLDOWN_CYCLES;
        }
        if (sum_out)   *sum_out   = 0;
        if (count_out) *count_out = count;
        return AUTO_ACT_FORWARD;
    }

    if (sum_out)   *sum_out   = sum;
    if (count_out) *count_out = count;

    // -------------------------------------------------------------------------
    // Priority 2: COOLDOWN — coast has finished but detection is suppressed.
    // If sensors still see wide black (robot still near the bar), force
    // straight. This prevents the immediate re-trigger that caused stalling.
    // -------------------------------------------------------------------------
    if (g_junction_cooldown_cycles > 0u)
    {
        g_junction_cooldown_cycles--;

        if (count >= 4u)
        {
            // Still crossing the wide bar — keep going straight.
            if (sum_out) *sum_out = 0;
            return AUTO_ACT_FORWARD;
        }
        // count < 4 during cooldown — normal following is safe; fall through.
    }

    // -------------------------------------------------------------------------
    // Priority 3: JUNCTION DETECT — new intersection, no cooldown active.
    // -------------------------------------------------------------------------
    if (count >= 4u)
    {
        g_junction_coast_cycles = IR_JUNCTION_COAST_CYCLES;
        return AUTO_ACT_JUNCTION;
    }

    // -------------------------------------------------------------------------
    // Priority 4: BIFURCATION — both outer wings active, centre clear.
    // -------------------------------------------------------------------------
    {
        bool left_wing    = ((detect_mask & IR_MASK_S1) != 0u) ||
                            ((detect_mask & IR_MASK_S2) != 0u);
        bool right_wing   = ((detect_mask & IR_MASK_S4) != 0u) ||
                            ((detect_mask & IR_MASK_S5) != 0u);
        bool center_clear = ((detect_mask & IR_MASK_S3) == 0u);

        if (left_wing && right_wing && center_clear)
        {
#if IR_BIFURCATION_PREFER_LEFT
            return AUTO_ACT_BIFURCATION_LEFT;
#else
            return AUTO_ACT_BIFURCATION_RIGHT;
#endif
        }
    }

    // -------------------------------------------------------------------------
    // Priority 5: Normal line following
    // -------------------------------------------------------------------------
#if (IR_AUTO_POLICY == IR_POLICY_MOVE_IF_DETECT)
    if (count < min_black_count)
    {
#if IR_NO_DETECT_CRAWL_ON_EMPTY
        return AUTO_ACT_CRAWL;
#else
        return AUTO_ACT_STOP;
#endif
    }
#else
    if (count >= min_black_count) return AUTO_ACT_STOP;
#endif

    if (sum <= -turn_threshold) return AUTO_ACT_LEFT;
    if (sum >= +turn_threshold) return AUTO_ACT_RIGHT;
    return AUTO_ACT_FORWARD;
}

static int16_t clamp_motor_cmd(int32_t v)
{
    if (v < 0)             return 0;
    if (v > MAX_SPEED_CMD) return MAX_SPEED_CMD;
    return (int16_t)v;
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
        case AUTO_ACT_RIGHT:
        {
            // Smoother OFF — immediate response on the first sensor cycle.
            // Seed smoother so the exit back to FORWARD is still smooth.
            int16_t tl, tr;
            ir_compute_steer(sum, base_speed, IR_STEER_STEP_PERCENT, &tl, &tr);
            ir_smooth_reset(tl, tr);
            platform_motor_set(tl, tr);
            break;
        }

        case AUTO_ACT_CRAWL:
        {
            int32_t crawl_base = ((int32_t)base_speed * IR_CRAWL_SPEED_PERCENT) / 100;
            if (crawl_base < (int32_t)IR_CRAWL_MIN_SPEED)
                crawl_base = (int32_t)IR_CRAWL_MIN_SPEED;

            int16_t tl, tr;
            ir_compute_steer(last_nonzero_sum, (int16_t)crawl_base,
                             IR_CRAWL_STEER_STEP, &tl, &tr);
            // Smoother OFF — must respond immediately, not ramp up from zero.
            ir_smooth_reset(tl, tr);
            platform_motor_set(tl, tr);
            break;
        }

        case AUTO_ACT_JUNCTION:
        {
            int32_t js = ((int32_t)base_speed * IR_JUNCTION_SPEED_PERCENT) / 100;
            int16_t j  = clamp_motor_cmd(js);
            ir_smooth_reset(j, j);
            platform_motor_set(j, j);
            break;
        }

        case AUTO_ACT_BIFURCATION_LEFT:
        {
            int32_t outer = ((int32_t)base_speed * IR_BIFURCATION_OUTER_PERCENT) / 100;
            int32_t inner = ((int32_t)base_speed * IR_MIN_INNER_PERCENT) / 100;
            int16_t l = clamp_motor_cmd(inner);
            int16_t r = clamp_motor_cmd(outer);
            ir_smooth_reset(l, r);
            platform_motor_set(l, r);
            break;
        }

        case AUTO_ACT_BIFURCATION_RIGHT:
        {
            int32_t outer = ((int32_t)base_speed * IR_BIFURCATION_OUTER_PERCENT) / 100;
            int32_t inner = ((int32_t)base_speed * IR_MIN_INNER_PERCENT) / 100;
            int16_t l = clamp_motor_cmd(outer);
            int16_t r = clamp_motor_cmd(inner);
            ir_smooth_reset(l, r);
            platform_motor_set(l, r);
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
        case AUTO_ACT_JUNCTION:          buf[i++]='J';buf[i++]='U';buf[i++]='N';buf[i++]='C';buf[i++]='T';buf[i++]='I';buf[i++]='O';buf[i++]='N'; break;
        case AUTO_ACT_BIFURCATION_LEFT:  buf[i++]='B';buf[i++]='I';buf[i++]='F';buf[i++]='_';buf[i++]='L';buf[i++]='E';buf[i++]='F';buf[i++]='T'; break;
        case AUTO_ACT_BIFURCATION_RIGHT: buf[i++]='B';buf[i++]='I';buf[i++]='F';buf[i++]='_';buf[i++]='R';buf[i++]='G';buf[i++]='H';buf[i++]='T'; break;
        default: break;
    }

    buf[i++]='\r'; buf[i++]='\n';
    platform_usart_write_buf(buf, i);
}

static ultra_action_t decide_ultrasonic_action(uint16_t front_cm,
                                                uint16_t left_cm,
                                                uint16_t right_cm)
{
    bool front_valid = (front_cm > 0u) && (front_cm < ULTRA_MAX_VALID_CM);
    bool left_valid  = (left_cm  > 0u) && (left_cm  < ULTRA_MAX_VALID_CM);
    bool right_valid = (right_cm > 0u) && (right_cm < ULTRA_MAX_VALID_CM);

    if (!front_valid) return ULTRA_ACT_STOP;

    bool front_blocked = (front_cm < ULTRA_FRONT_STOP_CM);
    bool front_caution = (front_cm < ULTRA_FRONT_CAUTION_CM) && !front_blocked;

    if (front_blocked) {
        uint16_t sl = left_valid  ? left_cm  : 0u;
        uint16_t sr = right_valid ? right_cm : 0u;
        int16_t  cd = (int16_t)sl - (int16_t)sr;
        if      (cd >  (int16_t)ULTRA_SIDE_DIFF_MIN_CM) return ULTRA_ACT_TURN_LEFT;
        else if (cd < -(int16_t)ULTRA_SIDE_DIFF_MIN_CM) return ULTRA_ACT_TURN_RIGHT;
        else return (sl >= sr) ? ULTRA_ACT_TURN_LEFT : ULTRA_ACT_TURN_RIGHT;
    }

    if (front_caution) {
        bool lc = left_valid  && (left_cm  < ULTRA_SIDE_CLOSE_CM);
        bool rc = right_valid && (right_cm < ULTRA_SIDE_CLOSE_CM);
        if      (lc && !rc) return ULTRA_ACT_RIGHT;
        else if (rc && !lc) return ULTRA_ACT_LEFT;
        else                return ULTRA_ACT_FORWARD;
    }

    bool lm = left_valid  && (left_cm  < ULTRA_SIDE_MEDIUM_CM);
    bool rm = right_valid && (right_cm < ULTRA_SIDE_MEDIUM_CM);
    bool lc = left_valid  && (left_cm  < ULTRA_SIDE_CLOSE_CM);
    bool rc = right_valid && (right_cm < ULTRA_SIDE_CLOSE_CM);

    if      (lc && !rc) return ULTRA_ACT_RIGHT;
    else if (rc && !lc) return ULTRA_ACT_LEFT;
    else if (lm && !rm) return ULTRA_ACT_RIGHT;
    else if (rm && !lm) return ULTRA_ACT_LEFT;
    return ULTRA_ACT_FORWARD;
}

static int16_t ultra_fwd_speed(uint16_t front_cm)
{
    if (front_cm < ULTRA_FRONT_CAUTION_CM) {
        int32_t range = ULTRA_FRONT_CAUTION_CM - ULTRA_FRONT_STOP_CM;
        int32_t dist  = (int32_t)front_cm - (int32_t)ULTRA_FRONT_STOP_CM;
        if (range > 0 && dist >= 0)
            return (int16_t)(ULTRA_SLOW_SPEED +
                             (dist * (ULTRA_FORWARD_SPEED - ULTRA_SLOW_SPEED)) / range);
        return ULTRA_SLOW_SPEED;
    }
    return ULTRA_FORWARD_SPEED;
}

static void apply_ultrasonic_action(ultra_action_t act, int16_t base_speed,
                                    uint16_t front_cm)
{
    int16_t l = 0, r = 0;
    switch (act) {
        case ULTRA_ACT_STOP:    l=0; r=0; break;
        case ULTRA_ACT_FORWARD: l=r=ultra_fwd_speed(front_cm); break;
        case ULTRA_ACT_LEFT:  { int16_t f=ultra_fwd_speed(front_cm); l=f; r=(int16_t)((int32_t)f*70/100); } break;
        case ULTRA_ACT_RIGHT: { int16_t f=ultra_fwd_speed(front_cm); l=(int16_t)((int32_t)f*70/100); r=f; } break;
        case ULTRA_ACT_TURN_LEFT:  l=-ULTRA_TURN_SPEED; r=+ULTRA_TURN_SPEED; break;
        case ULTRA_ACT_TURN_RIGHT: l=+ULTRA_TURN_SPEED; r=-ULTRA_TURN_SPEED; break;
        default: l=0; r=0; break;
    }
    platform_motor_set(l, r);
}

static void print_ultrasonic_status(uint16_t front_cm, uint16_t left_cm, uint16_t right_cm)
{
    char buf[48];
    uint32_t i = 0u;
    buf[i++]='U'; buf[i++]='S'; buf[i++]=':'; buf[i++]=' ';
    buf[i++]='F'; buf[i++]='=';
    if (front_cm>=100u){buf[i++]=(char)('0'+(front_cm/100u));front_cm%=100u;}
    if (front_cm>=10u) {buf[i++]=(char)('0'+(front_cm/10u)); front_cm%=10u;}
    buf[i++]=(char)('0'+front_cm);
    buf[i++]=' '; buf[i++]='L'; buf[i++]='=';
    if (left_cm>=100u){buf[i++]=(char)('0'+(left_cm/100u));left_cm%=100u;}
    if (left_cm>=10u) {buf[i++]=(char)('0'+(left_cm/10u)); left_cm%=10u;}
    buf[i++]=(char)('0'+left_cm);
    buf[i++]=' '; buf[i++]='R'; buf[i++]='=';
    if (right_cm>=100u){buf[i++]=(char)('0'+(right_cm/100u));right_cm%=100u;}
    if (right_cm>=10u) {buf[i++]=(char)('0'+(right_cm/10u)); right_cm%=10u;}
    buf[i++]=(char)('0'+right_cm);
    buf[i++]=' '; buf[i++]='c'; buf[i++]='m'; buf[i++]='\r'; buf[i++]='\n';
    platform_usart_write_buf(buf, i);
}

static void print_ultrasonic_action(ultra_action_t act)
{
    const char *s = "";
    switch (act) {
        case ULTRA_ACT_STOP:       s="STOP (obstacle too close or sensor error)"; break;
        case ULTRA_ACT_FORWARD:    s="FORWARD (path clear or slowing)"; break;
        case ULTRA_ACT_LEFT:       s="STEER LEFT (avoiding right obstacle)"; break;
        case ULTRA_ACT_RIGHT:      s="STEER RIGHT (avoiding left obstacle)"; break;
        case ULTRA_ACT_TURN_LEFT:  s="TURN LEFT (front blocked, turning to open space)"; break;
        case ULTRA_ACT_TURN_RIGHT: s="TURN RIGHT (front blocked, turning to open space)"; break;
        default: s="UNKNOWN"; break;
    }
    platform_usart_write_str("  Action: ");
    platform_usart_write_str(s);
    platform_usart_write_str("\r\n");
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

    uint16_t ultra_front_cm = 0u;
    uint16_t ultra_left_cm  = 0u;
    uint16_t ultra_right_cm = 0u;

    bool     last_reading   = false;
    bool     stable_state   = false;
    uint32_t last_change_ms = 0u;

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
                        if (controls_on && (mode == DRIVE_MODE_AUTO_IR) &&
                            auto_run_enabled && !safe.active) {
                            platform_usart_write_str("BTN: Button pressed — entering SAFE mode.\r\n");
                            safe_enter(&safe, SAFE_REASON_BUTTON, now, &auto_run_enabled);

                        } else if (controls_on) {
                            platform_usart_write_str("BTN: Button pressed — Controls DISABLED. Motors stopped.\r\n");
                            system_set_off(&controls_on);
                            auto_run_enabled  = false;
                            safe.active       = false;
                            safe.reason       = SAFE_REASON_NONE;
                            safe.entered_ms   = 0u;
                            ultra_fail_streak = 0u;
                            active_move_key   = 0;

                        } else {
                            platform_usart_write_str("BTN: Button pressed — Controls ENABLED. Select mode: M / U / O\r\n");
                            system_set_on(&controls_on);
                            current_speed     = BUTTON_ON_SPEED_CMD;
                            auto_run_enabled  = !safe.active &&
                                               ((mode == DRIVE_MODE_AUTO_IR) ||
                                                (mode == DRIVE_MODE_AUTO_ULTRASONIC));
                            last_rx_ms        = now;
                            ultra_fail_streak = 0u;
                            active_move_key   = 0;
                        }
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

                {
                    int16_t speed_before = current_speed;
                    if (try_parse_arrow_speed_char(c, controls_on, mode, &current_speed)) {
                        if (current_speed != speed_before)
                            refresh_ui(controls_on, mode, current_speed);
                        continue;
                    }
                }

                if ((c == 'M') || (c == 'm')) {
                    bool was_auto_ir = (mode == DRIVE_MODE_AUTO_IR);
                    mode             = DRIVE_MODE_MANUAL;
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
                        g_junction_coast_cycles    = 0u;
                        g_junction_cooldown_cycles = 0u;
                        ir_smooth_reset(0, 0);
                        for (uint8_t idx = 0u; idx < IR_SAMPLE_HISTORY_SIZE; idx++)
                            ir_black_history[idx] = 0u;
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
                    g_junction_coast_cycles    = 0u;
                    g_junction_cooldown_cycles = 0u;
                    ir_smooth_reset(0, 0);
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
                    platform_motor_stop();
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'Z') || (c == 'z')) {
                    if (mode == DRIVE_MODE_AUTO_IR) {
                        ir_debug_stream_enabled = !ir_debug_stream_enabled;
                        platform_usart_write_str(ir_debug_stream_enabled
                            ? "IR: debug stream ON\r\n"
                            : "IR: debug stream OFF\r\n");
                    }
                    continue;
                }

                c = to_lower(c);
                if (!controls_on) continue;

                if (c == SAFE_CLEAR_KEY) {
                    if (safe.active) {
                        safe_clear(&safe);
                        ir_crawl_since_ms          = 0u;
                        g_junction_coast_cycles    = 0u;
                        g_junction_cooldown_cycles = 0u;
                        ir_smooth_reset(0, 0);
                        refresh_ui(controls_on, mode, current_speed);
                    }
                    continue;
                }

                if (c == ' ') {
                    platform_motor_stop();
                    ir_smooth_reset(0, 0);
                    g_junction_coast_cycles    = 0u;
                    g_junction_cooldown_cycles = 0u;
                    g_ramp_state.target_left   = 0;
                    g_ramp_state.target_right  = 0;
                    g_ramp_state.current_left  = 0;
                    g_ramp_state.current_right = 0;
                    active_move_key = 0;
                    if ((mode == DRIVE_MODE_AUTO_IR) || (mode == DRIVE_MODE_AUTO_ULTRASONIC)) {
                        auto_run_enabled = false;
                        platform_usart_write_str("AUTO: paused — send U or O to resume\r\n");
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

        // ===== ULTRASONIC AUTO LOOP =====
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled) {
            if ((now - last_ultra_ms) >= ULTRA_POLL_MS) {
                bool ok_f = platform_ultrasonic_read_cm(ULTRA_FRONT, &ultra_front_cm);
                bool ok_l = platform_ultrasonic_read_cm(ULTRA_LEFT,  &ultra_left_cm);
                bool ok_r = platform_ultrasonic_read_cm(ULTRA_RIGHT, &ultra_right_cm);
                if (ok_f && ok_l && ok_r) {
                    ultra_fail_streak = 0u;
                    ultra_action_t ua = decide_ultrasonic_action(ultra_front_cm,
                                                                  ultra_left_cm,
                                                                  ultra_right_cm);
                    print_ultrasonic_status(ultra_front_cm, ultra_left_cm, ultra_right_cm);
                    apply_ultrasonic_action(ua, current_speed, ultra_front_cm);
                    print_ultrasonic_action(ua);
                } else {
                    platform_motor_stop();
                    platform_usart_write_str("US: read timeout\r\n");
                    if (ultra_fail_streak < 255u) ultra_fail_streak++;
                    if (ultra_fail_streak >= SAFE_ULTRA_FAIL_LIMIT)
                        safe_enter(&safe, SAFE_REASON_ULTRA_TIMEOUT, now, &auto_run_enabled);
                }
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

                // CRAWL is allowed to keep moving slowly, but it should still
                // latch SAFE after lingering in crawl for too long.
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

        // ===== MANUAL CMD TIMEOUT =====
        if (controls_on && (mode == DRIVE_MODE_MANUAL) && (active_move_key != 0)) {
            if ((now - last_cmd_ms) >= CMD_TIMEOUT_MS) {
                platform_motor_stop();
                active_move_key = 0;
            }
        }
    }
}