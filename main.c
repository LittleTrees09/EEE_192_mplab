#include "platform.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS               30u
#define AUTO_LOOP_MS              5u
// How long with no key received before the motor stops in manual mode.
// Must be longer than the ESP32 hold-repeat interval (40 ms) but short
// enough to stop quickly on button release. 250 ms allows for slower repeat rates.
#define CMD_TIMEOUT_MS            250u

#define DEFAULT_SPEED_CMD         300
#define MIN_SPEED_CMD             0
#define MAX_SPEED_CMD             1000
#define SPEED_STEP_CMD            100

// ===== MANUAL CONTROL PARAMETERS =====
#define TURN_SENSITIVITY_PERCENT  100
#define TURN_INNER_PERCENT        0

// Speed ramping — DISABLED for direct, lag-free manual response
#define RAMP_ENABLED              0   // was 1 — disabled so motor responds instantly
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
#define IR_MASK_ALL               (IR_MASK_S1 | IR_MASK_S2 | IR_MASK_S3 | IR_MASK_S4 | IR_MASK_S5)

// ===== ULTRASONIC OBSTACLE AVOIDANCE PARAMETERS =====
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
#define SAFE_COMM_LOSS_MS         10000u  // raised from 2500 — gives 10 s before comm-loss SAFE fires in IR mode
#define SAFE_ULTRA_FAIL_LIMIT     3u
#define SAFE_CLEAR_KEY            'x'
#define SAFE_IR_CRAWL_MS          3000u   // continuous crawl (no line) before SAFE triggers in IR mode

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
// 0 = sensor pin goes LOW on black (most common: TCRT5000, LM393 comparator).
// 1 = sensor pin goes HIGH on black.
// If the robot never reacts to tape, toggle with the 'P' key at runtime
// and watch the B: column in the IR debug stream ('I' key to enable).
#define IR_ACTIVE_ON_BLACK_HIGH   0u
#define IR_SAMPLE_HISTORY_SIZE       2u

#define IR_STEER_STEP_PERCENT      35
#define IR_STEER_MAX_PERCENT       100

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
    AUTO_ACT_CRAWL
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

static const char UI_OFF[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"\r\n"
"STATUS: OFF\r\n"
"Press onboard button to enable controls.\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic infrared follow mode\r\n"
"  O = automatic ultrasonic avoid mode\r\n";

static const char UI_MANUAL_HEAD[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: MANUAL (IMPROVED CONTROL)\r\n"
"Baud: 38400\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic infrared follow mode\r\n"
"  O = automatic ultrasonic avoid mode\r\n"
"\r\n"
"Manual drive:\r\n"
"  W = forward\r\n"
"  S = backward\r\n"
"  A = turn left (pivot: left reverse, right forward)\r\n"
"  D = turn right (pivot: left forward, right reverse)\r\n"
"  SPACE = stop\r\n"
"\r\n"
"Manual speed control:\r\n"
"  UP ARROW    = increase speed\r\n"
"  DOWN ARROW  = decrease speed\r\n"
"\r\n"
"IMPROVED FEATURES:\r\n"
"  - Matched turn speed (same as forward/backward)\r\n"
"  - Direct motor response (no ramp delay)\r\n"
"  - Pivot turns (opposite wheel directions)\r\n"
"\r\n"
"ADDED FEATURE:\r\n"
"  - Press T to test all motors."
"\r\n";

static const char UI_AUTO_IR[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: AUTO INFRARED FOLLOW\r\n"
"Baud: 38400\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic infrared follow mode\r\n"
"  O = automatic ultrasonic avoid mode\r\n"
"\r\n"
"IR auto behavior:\r\n"
#if (IR_AUTO_POLICY == IR_POLICY_MOVE_IF_DETECT)
"  - Move toward where black is detected\r\n"
"  - Stop if NO sensor sees black\r\n"
"  - Left-side black  -> turn left\r\n"
"  - Right-side black -> turn right\r\n"
"  - Center / balanced black -> forward\r\n"
#else
"  - Move if NO sensor sees black\r\n"
"  - Stop if ANY sensor sees black\r\n"
#endif
"\r\n"
"Recommended IR settings:\r\n"
"  TURN THRESHOLD = 1\r\n"
"  BLACK COUNT    = 1\r\n"
"  POLARITY       = LOW on black\r\n"
"  DEBUG STREAM   = ON\r\n"
"\r\n"
"SPACE stops the motors; send U again to resume auto.\r\n"
"\r\n"
"SAFE MODE:\r\n"
"  Press onboard button while running -> SAFE (motors stop, latched)\r\n"
"  No line detected for 3 s (crawling) -> SAFE (motors stop, latched)\r\n"
"  Send X to clear SAFE, then U to resume.\r\n";

static const char UI_AUTO_ULTRA[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: AUTO ULTRASONIC OBSTACLE AVOIDANCE\r\n"
"Baud: 38400\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic infrared follow mode\r\n"
"  O = automatic ultrasonic avoid mode\r\n"
"\r\n"
"Ultrasonic auto behavior:\r\n"
"  - Path clear ahead        -> forward at full speed\r\n"
"  - Obstacle approaching    -> slow down gradually\r\n"
"  - Obstacle too close      -> STOP and turn to open side\r\n"
"  - Side obstacles detected -> steer away while moving\r\n"
"\r\n"
"Sensor configuration:\r\n"
"  FRONT: Stop < 25cm, Caution < 40cm\r\n"
"  SIDES: Close < 20cm, Medium < 35cm\r\n"
"\r\n"
"SPACE stops the motors; send O again to resume auto.\r\n";

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
    if (!on) {
        platform_usart_write_str(UI_OFF);
        return;
    }

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

        default:
            break;
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

    g_ramp_state.target_left  = 0;
    g_ramp_state.target_right = 0;
    g_ramp_state.current_left = 0;
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

static void safe_enter(safe_state_t *safe,
                       safe_reason_t reason,
                       uint32_t now,
                       bool *auto_run_enabled)
{
    if (safe->active) {
        return;
    }

    safe->active     = true;
    safe->reason     = reason;
    safe->entered_ms = now;
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

// ===== MOTOR CONTROL =====
// RAMP_ENABLED = 0: set_motor_with_ramp passes straight through to
// platform_motor_set with no delay, giving instant manual response.

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
    // Direct — no ramp, instant response
    platform_motor_set(left, right);
#endif
}

static void update_motor_ramp(uint32_t now_ms)
{
#if RAMP_ENABLED
    if ((now_ms - g_ramp_state.last_ramp_ms) >= RAMP_CYCLE_MS) {
        g_ramp_state.current_left = ramp_toward(
            g_ramp_state.current_left,
            g_ramp_state.target_left,
            RAMP_STEP_PER_CYCLE
        );
        g_ramp_state.current_right = ramp_toward(
            g_ramp_state.current_right,
            g_ramp_state.target_right,
            RAMP_STEP_PER_CYCLE
        );

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
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner_speed = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(inner_speed, outer_speed);
#elif TURN_MODE_PIVOT
    int16_t turn_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(-turn_speed, +turn_speed);
#else
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(0, outer_speed);
#endif
}

static void apply_gentle_turn_right(int16_t base_speed)
{
#if TURN_MODE_GENTLE_ARC
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner_speed = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(outer_speed, inner_speed);
#elif TURN_MODE_PIVOT
    int16_t turn_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(+turn_speed, -turn_speed);
#else
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(outer_speed, 0);
#endif
}

static bool try_parse_arrow_speed_char(char c, bool on, drive_mode_t mode, int16_t *speed)
{
    static uint8_t esc_state = 0u;

    // Allow arrow-key speed changes in MANUAL and AUTO IR modes
    if (!on || ((mode != DRIVE_MODE_MANUAL) && (mode != DRIVE_MODE_AUTO_IR))) {
        esc_state = 0u;
        return false;
    }

    if (c == 0x1B) { esc_state = 1u; return true; }

    if (esc_state == 1u) {
        if (c == '[') { esc_state = 2u; return true; }
        esc_state = 0u;
        return false;
    }

    if (esc_state == 2u) {
        esc_state = 0u;

        if (c == 'A') { *speed = clamp_speed((int32_t)*speed + SPEED_STEP_CMD); return true; }
        if (c == 'B') { *speed = clamp_speed((int32_t)*speed - SPEED_STEP_CMD); return true; }
    }

    return false;
}

static uint8_t ir_mask_on_black(uint8_t raw)
{
#if IR_ACTIVE_ON_BLACK_HIGH
    return raw;
#else
    return (uint8_t)(~raw) & IR_MASK_ALL;
#endif
}

static uint8_t ir_mask_on_black_runtime(uint8_t raw, bool active_on_black_high)
{
    return active_on_black_high ? raw : (uint8_t)(~raw) & IR_MASK_ALL;
}

static uint8_t ir_mask_or_history(const uint8_t *history, uint8_t history_count)
{
    uint8_t mask = 0u;

    for (uint8_t i = 0u; i < history_count; i++) {
        mask |= history[i];
    }

    return mask;
}

static auto_action_t decide_auto_action(uint8_t steer_mask,
                                         uint8_t detect_mask,
                                         int8_t turn_threshold,
                                         uint8_t min_black_count,
                                         int8_t *sum_out,
                                         uint8_t *count_out)
{
    int8_t  sum   = 0;
    uint8_t count = 0u;

    // count and sum both run off the same mask so the "enough sensors?" gate
    // and the steering direction are always consistent for the same frame.
    // detect_mask (OR-history) is still checked but only for the count gate,
    // keeping the hysteresis benefit while avoiding a steer/count split.
    if (detect_mask & IR_MASK_S1) { count++; }
    if (detect_mask & IR_MASK_S2) { count++; }
    if (detect_mask & IR_MASK_S3) { count++; }
    if (detect_mask & IR_MASK_S4) { count++; }
    if (detect_mask & IR_MASK_S5) { count++; }

    // sum always uses steer_mask (current frame) for responsive steering
    if (steer_mask & IR_MASK_S1) { sum += -2; }
    if (steer_mask & IR_MASK_S2) { sum += -1; }
    if (steer_mask & IR_MASK_S3) { sum +=  0; }
    if (steer_mask & IR_MASK_S4) { sum += +1; }
    if (steer_mask & IR_MASK_S5) { sum += +2; }

    if (sum_out)   *sum_out   = sum;
    if (count_out) *count_out = count;

#if (IR_AUTO_POLICY == IR_POLICY_MOVE_IF_DETECT)
    if (count < min_black_count) {
#if IR_NO_DETECT_CRAWL_ON_EMPTY
        return AUTO_ACT_CRAWL;
#else
        return AUTO_ACT_STOP;
#endif
    }
#else
    if (count >= min_black_count) {
        return AUTO_ACT_STOP;
    }
#endif

    if (sum <= -turn_threshold) return AUTO_ACT_LEFT;
    if (sum >= +turn_threshold) return AUTO_ACT_RIGHT;

    return AUTO_ACT_FORWARD;
}

static int16_t clamp_motor_cmd(int32_t v)
{
    if (v < 0)            return 0;
    if (v > MAX_SPEED_CMD) return MAX_SPEED_CMD;
    return (int16_t)v;
}

static void apply_auto_action(auto_action_t act, int16_t base_speed,
                               int8_t sum, int8_t last_nonzero_sum)
{
    int16_t left_speed;
    int16_t right_speed;

    switch (act)
    {
        case AUTO_ACT_STOP:
            platform_motor_stop();
            break;

        case AUTO_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case AUTO_ACT_LEFT:
        case AUTO_ACT_RIGHT:
            {
                int8_t  abs_sum        = (sum < 0) ? (int8_t)(-sum) : sum;
                // Keep both wheels moving forward so the robot arcs instead of spinning in place.
                // Stronger sensor error makes the inner wheel much slower, not reversed.
                int32_t outer_scale_percent = 100;
                int32_t inner_scale_percent = (abs_sum >= 2) ? 25 : 55;

                int32_t outer_speed = ((int32_t)base_speed * outer_scale_percent) / 100;
                int32_t inner_speed = ((int32_t)base_speed * inner_scale_percent) / 100;

                // Add extra separation between inner and outer wheels when the error is larger.
                if (abs_sum == 1) {
                    inner_speed = ((int32_t)inner_speed * 90) / 100;
                }

                if (act == AUTO_ACT_LEFT) {
                    left_speed  = clamp_motor_cmd(inner_speed);
                    right_speed = clamp_motor_cmd(outer_speed);
                } else {
                    left_speed  = clamp_motor_cmd(outer_speed);
                    right_speed = clamp_motor_cmd(inner_speed);
                }

                platform_motor_set(left_speed, right_speed);
            }
            break;

        case AUTO_ACT_CRAWL:
                {
                // Crawl uses a reduced base; also scale with last_nonzero_sum for steering
                int8_t abs_s = (last_nonzero_sum < 0) ? (int8_t)(-last_nonzero_sum) : last_nonzero_sum;
                int32_t scale_percent = 100 - (abs_s * 12);
                if (scale_percent < 30) scale_percent = 30;
                int16_t crawl_speed = (int16_t)((((int32_t)base_speed * scale_percent) / 100) / 2);
                if (last_nonzero_sum != 0) {
                    int32_t correction     = ((int32_t)crawl_speed * IR_STEER_STEP_PERCENT * abs_s) / 100;
                    int32_t correction_max = ((int32_t)crawl_speed * IR_STEER_MAX_PERCENT) / 100;
                    if (correction > correction_max) correction = correction_max;
                    if (last_nonzero_sum < 0) {
                        platform_motor_set(clamp_motor_cmd((int32_t)crawl_speed - correction),
                                           clamp_motor_cmd((int32_t)crawl_speed + correction));
                    } else {
                        platform_motor_set(clamp_motor_cmd((int32_t)crawl_speed + correction),
                                           clamp_motor_cmd((int32_t)crawl_speed - correction));
                    }
                } else {
                    platform_motor_set(+crawl_speed, +crawl_speed);
                }
            }
            break;

        default:
            break;
    }
}

static void print_ir_debug_status(uint8_t raw_mask,
                                   uint8_t black_mask,
                                   int8_t sum,
                                   uint8_t count,
                                   auto_action_t act)
{
    char buf[64];
    uint32_t i = 0u;

    buf[i++] = 'I'; buf[i++] = 'R'; buf[i++] = ':'; buf[i++] = ' ';

    buf[i++] = (raw_mask & IR_MASK_S1) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S2) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S3) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S4) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S5) ? '1' : '0';

    buf[i++] = ' '; buf[i++] = 'B'; buf[i++] = ':';
    buf[i++] = (black_mask & IR_MASK_S1) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S2) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S3) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S4) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S5) ? '1' : '0';

    buf[i++] = ' '; buf[i++] = 'S'; buf[i++] = 'u'; buf[i++] = 'm'; buf[i++] = '=';
    if (sum < 0) { buf[i++] = '-'; buf[i++] = (char)('0' + (uint8_t)(-sum)); }
    else         { buf[i++] = '+'; buf[i++] = (char)('0' + (uint8_t)sum); }

    buf[i++] = ' '; buf[i++] = 'C'; buf[i++] = 'n'; buf[i++] = 't'; buf[i++] = '=';
    buf[i++] = (char)('0' + count);
    buf[i++] = ' ';

    switch (act)
    {
        case AUTO_ACT_STOP:    buf[i++]='S'; buf[i++]='T'; buf[i++]='O'; buf[i++]='P'; break;
        case AUTO_ACT_FORWARD: buf[i++]='F'; buf[i++]='W'; buf[i++]='D'; break;
        case AUTO_ACT_LEFT:    buf[i++]='L'; buf[i++]='E'; buf[i++]='F'; buf[i++]='T'; break;
        case AUTO_ACT_RIGHT:   buf[i++]='R'; buf[i++]='I'; buf[i++]='G'; buf[i++]='H'; buf[i++]='T'; break;
        case AUTO_ACT_CRAWL:   buf[i++]='C'; buf[i++]='R'; buf[i++]='A'; buf[i++]='W'; buf[i++]='L'; break;
        default: break;
    }

    buf[i++] = '\r'; buf[i++] = '\n';
    platform_usart_write_buf(buf, i);
}

static void print_ir_tuning_status(int8_t turn_threshold,
                                   uint8_t min_black_count,
                                   bool debug_enabled,
                                   bool active_on_black_high)
{
    char buf[80];
    uint32_t i = 0u;

    buf[i++]='I'; buf[i++]='R'; buf[i++]=' ';
    buf[i++]='T'; buf[i++]='U'; buf[i++]='N'; buf[i++]='I'; buf[i++]='N'; buf[i++]='G'; buf[i++]=':'; buf[i++]=' ';
    buf[i++]='T'; buf[i++]='u'; buf[i++]='r'; buf[i++]='n'; buf[i++]='T'; buf[i++]='h'; buf[i++]='r'; buf[i++]='e'; buf[i++]='s'; buf[i++]='h'; buf[i++]='=';
    buf[i++] = (char)('0' + turn_threshold);
    buf[i++]=','; buf[i++]=' ';
    buf[i++]='M'; buf[i++]='i'; buf[i++]='n'; buf[i++]='B'; buf[i++]='l'; buf[i++]='a'; buf[i++]='c'; buf[i++]='k'; buf[i++]='=';
    buf[i++] = (char)('0' + min_black_count);
    buf[i++]=','; buf[i++]=' ';
    buf[i++]='D'; buf[i++]='e'; buf[i++]='b'; buf[i++]='u'; buf[i++]='g'; buf[i++]='='; buf[i++]='O';
    buf[i++] = debug_enabled ? 'N' : 'F';
    buf[i++] = debug_enabled ? ' ' : 'F';
    buf[i++]=' '; buf[i++]='P'; buf[i++]='o'; buf[i++]='l'; buf[i++]='=';
    buf[i++] = active_on_black_high ? 'H' : 'L';
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
        uint16_t safe_left_cm  = left_valid  ? left_cm  : 0u;
        uint16_t safe_right_cm = right_valid ? right_cm : 0u;
        int16_t  clearance_diff = (int16_t)safe_left_cm - (int16_t)safe_right_cm;

        if      (clearance_diff >  (int16_t)ULTRA_SIDE_DIFF_MIN_CM) return ULTRA_ACT_TURN_LEFT;
        else if (clearance_diff < -(int16_t)ULTRA_SIDE_DIFF_MIN_CM) return ULTRA_ACT_TURN_RIGHT;
        else return (safe_left_cm >= safe_right_cm) ? ULTRA_ACT_TURN_LEFT : ULTRA_ACT_TURN_RIGHT;
    }

    if (front_caution) {
        bool left_close  = left_valid  && (left_cm  < ULTRA_SIDE_CLOSE_CM);
        bool right_close = right_valid && (right_cm < ULTRA_SIDE_CLOSE_CM);

        if      (left_close  && !right_close) return ULTRA_ACT_RIGHT;
        else if (right_close && !left_close)  return ULTRA_ACT_LEFT;
        else                                  return ULTRA_ACT_FORWARD;
    }

    bool left_medium  = left_valid  && (left_cm  < ULTRA_SIDE_MEDIUM_CM);
    bool right_medium = right_valid && (right_cm < ULTRA_SIDE_MEDIUM_CM);
    bool left_close   = left_valid  && (left_cm  < ULTRA_SIDE_CLOSE_CM);
    bool right_close  = right_valid && (right_cm < ULTRA_SIDE_CLOSE_CM);

    if      (left_close  && !right_close)  return ULTRA_ACT_RIGHT;
    else if (right_close && !left_close)   return ULTRA_ACT_LEFT;
    else if (left_medium && !right_medium) return ULTRA_ACT_RIGHT;
    else if (right_medium && !left_medium) return ULTRA_ACT_LEFT;

    return ULTRA_ACT_FORWARD;
}

static int16_t ultra_fwd_speed(uint16_t front_cm)
{
    if (front_cm < ULTRA_FRONT_CAUTION_CM) {
        int32_t range = ULTRA_FRONT_CAUTION_CM - ULTRA_FRONT_STOP_CM;
        int32_t dist  = (int32_t)front_cm - (int32_t)ULTRA_FRONT_STOP_CM;
        if (range > 0 && dist >= 0) {
            return (int16_t)(ULTRA_SLOW_SPEED +
                             (dist * (ULTRA_FORWARD_SPEED - ULTRA_SLOW_SPEED)) / range);
        }
        return ULTRA_SLOW_SPEED;
    }
    return ULTRA_FORWARD_SPEED;
}

static void apply_ultrasonic_action(ultra_action_t act,
                                    int16_t base_speed,
                                    uint16_t front_cm)
{
    int16_t left_speed  = 0;
    int16_t right_speed = 0;

    switch (act)
    {
        case ULTRA_ACT_STOP:
            left_speed = 0; right_speed = 0;
            break;
        case ULTRA_ACT_FORWARD:
            left_speed = right_speed = ultra_fwd_speed(front_cm);
            break;
        case ULTRA_ACT_LEFT:
            {
                int16_t fwd = ultra_fwd_speed(front_cm);
                left_speed  = fwd;
                right_speed = (int16_t)((int32_t)fwd * 70 / 100);
            }
            break;
        case ULTRA_ACT_RIGHT:
            {
                int16_t fwd = ultra_fwd_speed(front_cm);
                left_speed  = (int16_t)((int32_t)fwd * 70 / 100);
                right_speed = fwd;
            }
            break;
        case ULTRA_ACT_TURN_LEFT:
            left_speed = -ULTRA_TURN_SPEED; right_speed = +ULTRA_TURN_SPEED;
            break;
        case ULTRA_ACT_TURN_RIGHT:
            left_speed = +ULTRA_TURN_SPEED; right_speed = -ULTRA_TURN_SPEED;
            break;
        default:
            left_speed = 0; right_speed = 0;
            break;
    }

    platform_motor_set(left_speed, right_speed);
}

static void print_ultrasonic_status(uint16_t front_cm, uint16_t left_cm, uint16_t right_cm)
{
    char buf[48];
    uint32_t i = 0u;

    buf[i++]='U'; buf[i++]='S'; buf[i++]=':'; buf[i++]=' ';
    buf[i++]='F'; buf[i++]='=';
    if (front_cm >= 100u) { buf[i++] = (char)('0' + (front_cm / 100u)); front_cm %= 100u; }
    if (front_cm >= 10u)  { buf[i++] = (char)('0' + (front_cm / 10u));  front_cm %= 10u;  }
    buf[i++] = (char)('0' + front_cm);

    buf[i++]=' '; buf[i++]='L'; buf[i++]='=';
    if (left_cm >= 100u) { buf[i++] = (char)('0' + (left_cm / 100u)); left_cm %= 100u; }
    if (left_cm >= 10u)  { buf[i++] = (char)('0' + (left_cm / 10u));  left_cm %= 10u;  }
    buf[i++] = (char)('0' + left_cm);

    buf[i++]=' '; buf[i++]='R'; buf[i++]='=';
    if (right_cm >= 100u) { buf[i++] = (char)('0' + (right_cm / 100u)); right_cm %= 100u; }
    if (right_cm >= 10u)  { buf[i++] = (char)('0' + (right_cm / 10u));  right_cm %= 10u;  }
    buf[i++] = (char)('0' + right_cm);

    buf[i++]=' '; buf[i++]='c'; buf[i++]='m'; buf[i++]='\r'; buf[i++]='\n';
    platform_usart_write_buf(buf, i);
}

static void print_ultrasonic_action(ultra_action_t act)
{
    const char *action_str = "";

    switch (act)
    {
        case ULTRA_ACT_STOP:       action_str = "STOP (obstacle too close or sensor error)";     break;
        case ULTRA_ACT_FORWARD:    action_str = "FORWARD (path clear or slowing)";               break;
        case ULTRA_ACT_LEFT:       action_str = "STEER LEFT (avoiding right obstacle)";          break;
        case ULTRA_ACT_RIGHT:      action_str = "STEER RIGHT (avoiding left obstacle)";          break;
        case ULTRA_ACT_TURN_LEFT:  action_str = "TURN LEFT (front blocked, turning to open space)";  break;
        case ULTRA_ACT_TURN_RIGHT: action_str = "TURN RIGHT (front blocked, turning to open space)"; break;
        default:                   action_str = "UNKNOWN";                                       break;
    }

    platform_usart_write_str("  Action: ");
    platform_usart_write_str(action_str);
    platform_usart_write_str("\r\n");
}

int main(void)
{
    bool controls_on   = false;
    drive_mode_t mode  = DRIVE_MODE_MANUAL;
    int16_t current_speed = DEFAULT_SPEED_CMD;

    char     active_move_key = 0;     // Currently active continuous move command
    uint32_t last_cmd_ms    = 0u;     // Timestamp of last direction key received
    uint32_t last_auto_ms   = 0u;
    uint32_t last_ultra_ms  = 0u;

    int8_t  ir_turn_threshold      = IR_TURN_THRESHOLD_DEFAULT;
    uint8_t ir_min_black_count     = IR_MIN_COUNT_DEFAULT;
    bool    ir_debug_stream_enabled = false;
    bool    auto_run_enabled        = false;
    bool    ir_active_on_black_high = (IR_ACTIVE_ON_BLACK_HIGH != 0u);
    int8_t  last_nonzero_sum        = 0;
    safe_state_t safe = {false, SAFE_REASON_NONE, 0u};
    uint32_t last_rx_ms       = 0u;
    uint8_t  ultra_fail_streak = 0u;

    uint8_t ir_black_history[IR_SAMPLE_HISTORY_SIZE] = {0u, 0u};
    uint8_t ir_black_history_count = 0u;
    uint8_t ir_black_history_index = 0u;
    uint32_t ir_crawl_since_ms = 0u;  // timestamp when continuous CRAWL started; 0 = not crawling

    uint16_t ultra_front_cm = 0u;
    uint16_t ultra_left_cm  = 0u;
    uint16_t ultra_right_cm = 0u;

    bool     last_reading  = false;
    bool     stable_state  = false;
    uint32_t last_change_ms = 0u;

    platform_initialization();
    refresh_ui(controls_on, mode, current_speed);

    while (1)
    {
        uint32_t now = platform_millis();

        // Ramp update — no-op when RAMP_ENABLED = 0
        update_motor_ramp(now);

        // ===== BUTTON DEBOUNCE =====
        {
            bool reading = platform_button_pressed();

            if (reading != last_reading) {
                last_change_ms = now;
                last_reading   = reading;
            }

            if ((now - last_change_ms) > DEBOUNCE_MS) {
                if (stable_state != reading) {
                    stable_state = reading;

                    if (stable_state) {
                        if (controls_on && (mode == DRIVE_MODE_AUTO_IR) && auto_run_enabled && !safe.active) {
                            // Button during active IR follow → SAFE, not full power-off.
                            // Motors stop, controls stay on, LED stays on.
                            // User must send X then U to resume.
                            safe_enter(&safe, SAFE_REASON_BUTTON, now, &auto_run_enabled);
                        } else if (controls_on) {
                            system_set_off(&controls_on);
                            auto_run_enabled   = false;
                            safe.active        = false;
                            safe.reason        = SAFE_REASON_NONE;
                            safe.entered_ms    = 0u;
                            ultra_fail_streak  = 0u;
                            active_move_key    = 0;
                        } else {
                            system_set_on(&controls_on);
                            auto_run_enabled = !safe.active &&
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
                now        = platform_millis();
                last_rx_ms = now;

                {
                    int16_t speed_before = current_speed;
                    if (try_parse_arrow_speed_char(c, controls_on, mode, &current_speed)) {
                        if (current_speed != speed_before) {
                            refresh_ui(controls_on, mode, current_speed);
                        }
                        continue;
                    }
                }

                if ((c == 'M') || (c == 'm')) {
                    bool was_auto_ir = (mode == DRIVE_MODE_AUTO_IR);
                    mode              = DRIVE_MODE_MANUAL;
                    active_move_key   = 0;
                    last_cmd_ms       = now;
                    auto_run_enabled  = false;
                    ultra_fail_streak = 0u;
                    platform_motor_stop();
                    // Clear IR carry-over state so re-entering Auto IR starts fresh
                    if (was_auto_ir) {
                        last_nonzero_sum       = 0;
                        ir_debug_stream_enabled = false;
                        ir_black_history_count  = 0u;
                        ir_black_history_index  = 0u;
                        for (uint8_t idx = 0u; idx < IR_SAMPLE_HISTORY_SIZE; idx++) {
                            ir_black_history[idx] = 0u;
                        }
                    }
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'U') || (c == 'u')) {
                    mode              = DRIVE_MODE_AUTO_IR;
                    active_move_key   = 0;
                    last_cmd_ms       = 0u;
                    auto_run_enabled  = controls_on && !safe.active;
                    ultra_fail_streak = 0u;
                    // Always reset IR history on entry so stale readings don't
                    // produce a wrong first steering decision
                    last_nonzero_sum       = 0;
                    ir_debug_stream_enabled = true;
                    ir_black_history_count  = 0u;
                    ir_black_history_index  = 0u;
                    ir_crawl_since_ms       = 0u;
                    for (uint8_t idx = 0u; idx < IR_SAMPLE_HISTORY_SIZE; idx++) {
                        ir_black_history[idx] = 0u;
                    }
                    platform_motor_stop();
                    if (!controls_on) {
                        platform_usart_write_str("IR: controls OFF — press button first\r\n");
                    } else if (safe.active) {
                        platform_usart_write_str("IR: SAFE active — send X to clear\r\n");
                    } else {
                        platform_usart_write_str("IR: auto follow RUNNING with fixed recommended settings\r\n");
                        platform_usart_write_str("IR: debug stream ON so you can observe sensed values\r\n");
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

                c = to_lower(c);

                if (!controls_on) continue;

                if (c == SAFE_CLEAR_KEY) {
                    if (safe.active) {
                        safe_clear(&safe);
                        ir_crawl_since_ms = 0u;
                        refresh_ui(controls_on, mode, current_speed);
                    }
                    continue;
                }

                if (c == ' ') {
                    platform_motor_stop();
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
                    ((c == 'i') || (c == 'p') || (c == 'n') || (c == 'b') ||
                     ((c >= '1') && (c <= '3')))) {
                    platform_usart_write_str("IR: live tuning disabled; fixed recommended settings are active\r\n");
                    continue;
                }

                if (mode != DRIVE_MODE_MANUAL) continue;

                // ===== HARDWARE DIAGNOSTIC MODE =====
                if (c == 't') {
                    uint32_t test_start;
                    // Test sequence: left fwd, left rev, right fwd, right rev
                    platform_usart_write_str("MOTOR TEST: Left Forward\r\n");
                    platform_motor_set(+500, 0);   // Left motor only, full forward
                    test_start = platform_millis();
                    while ((platform_millis() - test_start) < 1000) { asm("nop"); }
                    
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Left Reverse\r\n");
                    platform_motor_set(-500, 0);   // Left motor only, full reverse
                    test_start = platform_millis();
                    while ((platform_millis() - test_start) < 1000) { asm("nop"); }
                    
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Right Forward\r\n");
                    platform_motor_set(0, +500);   // Right motor only, full forward
                    test_start = platform_millis();
                    while ((platform_millis() - test_start) < 1000) { asm("nop"); }
                    
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Right Reverse\r\n");
                    platform_motor_set(0, -500);   // Right motor only, full reverse
                    test_start = platform_millis();
                    while ((platform_millis() - test_start) < 1000) { asm("nop"); }
                    
                    platform_motor_stop();
                    platform_usart_write_str("MOTOR TEST: Complete\r\n");
                    continue;
                }

                // ===== MANUAL DRIVE =====
                switch (c)
                {
                    case 'w':
                        active_move_key = 'w';
                        last_cmd_ms = now;
                        set_motor_with_ramp(+current_speed, +current_speed);
                        break;

                    case 's':
                        active_move_key = 's';
                        last_cmd_ms = now;
                        set_motor_with_ramp(-current_speed, -current_speed);
                        break;

                    case 'a':
                        active_move_key = 'a';
                        last_cmd_ms = now;
                        apply_gentle_turn_left(current_speed);
                        break;

                    case 'd':
                        active_move_key = 'd';
                        last_cmd_ms = now;
                        apply_gentle_turn_right(current_speed);
                        break;

                    default:
                        break;
                }
            }
        }

        // ===== SAFE MODE: COMM LOSS CHECK =====
        // Comm-loss SAFE is skipped in IR auto mode — the follower is
        // fully autonomous and does not require periodic serial keep-alives.
        // It still applies in ultrasonic mode as a deadman switch.
        if (controls_on && !safe.active && auto_run_enabled &&
            (mode == DRIVE_MODE_AUTO_ULTRASONIC)) {
            if ((now - last_rx_ms) > SAFE_COMM_LOSS_MS) {
                safe_enter(&safe, SAFE_REASON_COMM_LOSS, now, &auto_run_enabled);
            }
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
                    ultra_action_t ultra_act = decide_ultrasonic_action(ultra_front_cm,
                                                                        ultra_left_cm,
                                                                        ultra_right_cm);
                    print_ultrasonic_status(ultra_front_cm, ultra_left_cm, ultra_right_cm);
                    apply_ultrasonic_action(ultra_act, current_speed, ultra_front_cm);
                    print_ultrasonic_action(ultra_act);
                } else {
                    platform_motor_stop();
                    platform_usart_write_str("US: read timeout\r\n");
                    if (ultra_fail_streak < 255u) ultra_fail_streak++;
                    if (ultra_fail_streak >= SAFE_ULTRA_FAIL_LIMIT) {
                        safe_enter(&safe, SAFE_REASON_ULTRA_TIMEOUT, now, &auto_run_enabled);
                    }
                }

                last_ultra_ms = now;
            }
        }

        // ===== IR AUTO LOOP =====
        if (controls_on && !safe.active &&
            (mode == DRIVE_MODE_AUTO_IR) && auto_run_enabled) {
            if ((now - last_auto_ms) >= AUTO_LOOP_MS) {
                uint8_t raw_mask    = platform_ir_read_mask_raw();
                uint8_t black_mask  = ir_mask_on_black_runtime(raw_mask, ir_active_on_black_high);
                uint8_t filtered_black_mask;
                int8_t  sum   = 0;
                uint8_t count = 0u;

                ir_black_history[ir_black_history_index] = black_mask;
                ir_black_history_index = (uint8_t)((ir_black_history_index + 1u) % IR_SAMPLE_HISTORY_SIZE);
                if (ir_black_history_count < IR_SAMPLE_HISTORY_SIZE) ir_black_history_count++;

                filtered_black_mask = ir_mask_or_history(ir_black_history, ir_black_history_count);

                auto_action_t act = decide_auto_action(black_mask,
                                                       filtered_black_mask,
                                                       ir_turn_threshold,
                                                       ir_min_black_count,
                                                       &sum,
                                                       &count);

                if (sum != 0) last_nonzero_sum = sum;

                // ---- crawl-timeout SAFE check ----
                // Start the clock the first time we get a CRAWL action (no line seen).
                // Reset it the moment any non-CRAWL action fires (line re-acquired).
                // If CRAWL persists uninterrupted for SAFE_IR_CRAWL_MS, enter SAFE.
                if (act == AUTO_ACT_CRAWL) {
                    if (ir_crawl_since_ms == 0u) {
                        ir_crawl_since_ms = now;   // mark when crawling started
                    } else if ((now - ir_crawl_since_ms) >= SAFE_IR_CRAWL_MS) {
                        safe_enter(&safe, SAFE_REASON_LINE_LOST, now, &auto_run_enabled);
                        ir_crawl_since_ms = 0u;
                        last_auto_ms = now;
                        continue;
                    }
                } else {
                    ir_crawl_since_ms = 0u;        // line seen — reset the timer
                }

                apply_auto_action(act, current_speed, sum, last_nonzero_sum);

                if (ir_debug_stream_enabled) {
                    print_ir_debug_status(raw_mask, black_mask, sum, count, act);
                }

                last_auto_ms = now;
            }
        }

        // ===== MANUAL CMD TIMEOUT =====
        // If no direction key has arrived within CMD_TIMEOUT_MS, the button
        // has been released — stop the motors immediately.
        if (controls_on && (mode == DRIVE_MODE_MANUAL) && (active_move_key != 0)) {
            if ((now - last_cmd_ms) >= CMD_TIMEOUT_MS) {
                platform_motor_stop();
                active_move_key = 0;
            }
        }
    }
}