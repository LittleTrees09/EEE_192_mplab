#include "platform.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS               30u
#define CMD_TIMEOUT_FIRST_MS      900u
#define CMD_TIMEOUT_REPEAT_MS     150u  // Increased from 100ms for more stable control
#define AUTO_LOOP_MS              10u

#define DEFAULT_SPEED_CMD         300
#define MIN_SPEED_CMD             0
#define MAX_SPEED_CMD             1000
#define SPEED_STEP_CMD            100

// ===== IMPROVED MANUAL CONTROL PARAMETERS =====
// Turn sensitivity: lower = gentler turns, higher = sharper turns
#define TURN_SENSITIVITY_PERCENT  40    // 40% of current_speed for outer wheel during turn
#define TURN_INNER_PERCENT        0     // 0% for inner wheel (stationary pivot point)

// Speed ramping for smoother acceleration/deceleration
#define RAMP_ENABLED              1     // Set to 0 to disable ramping
#define RAMP_STEP_PER_CYCLE       50    // Speed increase per control cycle
#define RAMP_CYCLE_MS             20    // Time between ramp steps

// Alternative turn modes (comment/uncomment to select)
// Mode 1: Gentle arc turn (outer wheel faster, inner wheel slower)
#define TURN_MODE_GENTLE_ARC      0
// Mode 2: Pivot turn (one wheel forward, other reverse) - original behavior
#define TURN_MODE_PIVOT           1

#if TURN_MODE_GENTLE_ARC
  #define TURN_OUTER_SPEED_PERCENT  100  // Outer wheel at full speed
  #define TURN_INNER_SPEED_PERCENT  30   // Inner wheel at 30% speed (creates arc)
#endif

#define TURN90_SPEED              250
#define TURN90_MS                 750u

#define IR_MASK_S1                (1u << 0)
#define IR_MASK_S2                (1u << 1)
#define IR_MASK_S3                (1u << 2)
#define IR_MASK_S4                (1u << 3)
#define IR_MASK_S5                (1u << 4)
#define IR_MASK_ALL               (IR_MASK_S1 | IR_MASK_S2 | IR_MASK_S3 | IR_MASK_S4 | IR_MASK_S5)

#define ULTRA_POLL_MS             100u
#define ULTRA_FRONT_LIMIT_CM      20u
#define ULTRA_SIDE_LIMIT_CM       15u

#define IR_TURN_THRESHOLD_MIN      1
#define IR_TURN_THRESHOLD_MAX      4
#define IR_TURN_THRESHOLD_DEFAULT  1

#define IR_MIN_COUNT_MIN           1u
#define IR_MIN_COUNT_MAX           5u
#define IR_MIN_COUNT_DEFAULT       1u

#define IR_POLICY_MOVE_IF_NONE      0u
#define IR_POLICY_MOVE_IF_DETECT    1u
#define IR_AUTO_POLICY              IR_POLICY_MOVE_IF_DETECT

#define IR_NO_DETECT_CRAWL_ON_EMPTY 1u
#define IR_ACTIVE_ON_BLACK_HIGH   0u

#define SOFT_TURN_DIV             2
#define MIN_TURN_SPEED_CMD        120

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

// ===== SPEED RAMPING STATE =====
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
"  Q = ~90 deg left\r\n"
"  E = ~90 deg right\r\n"
"  SPACE = stop\r\n"
"\r\n"
"Manual speed control:\r\n"
"  UP ARROW    = increase speed\r\n"
"  DOWN ARROW  = decrease speed\r\n"
"\r\n"
"IMPROVED FEATURES:\r\n"
"  - Reduced turn sensitivity\r\n"
"  - Smooth speed ramping\r\n"
"  - Pivot turns (opposite wheel directions)\r\n"
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
"  - Use this mode to emulate black-line hit as STOP\r\n"
#endif
"\r\n"
"IR tuning (live):\r\n"
"  I = toggle IR debug stream\r\n"
"  1..4 = TURN THRESHOLD   (1 = turn sooner, 4 = turn later)\r\n"
"  N / B = BLACK COUNT     (N = fewer sensors, B = more sensors)\r\n"
"           low count = reacts faster, high count = needs stronger hit\r\n"
"\r\n"
"SPACE stops the motors; send U again to resume auto.\r\n";

static const char UI_AUTO_ULTRA[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: AUTO ULTRASONIC AVOID\r\n"
"Baud: 38400\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic infrared follow mode\r\n"
"  O = automatic ultrasonic avoid mode\r\n"
"\r\n"
"Ultrasonic auto behavior:\r\n"
"  - No obstacle     -> stop\r\n"
"  - Front obstacle  -> turn away\r\n"
"  - Left obstacle   -> steer right\r\n"
"  - Right obstacle  -> steer left\r\n"
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

static inline void note_move_cmd(char key,
                                 uint32_t now_ms,
                                 uint32_t *last_cmd_ms,
                                 char *last_move_key,
                                 uint32_t *cmd_timeout_ms,
                                 bool *repeat_started)
{
    if (key != *last_move_key) {
        *last_move_key  = key;
        *cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
        *repeat_started = false;
    } else {
        if (!*repeat_started) {
            *cmd_timeout_ms = CMD_TIMEOUT_REPEAT_MS;
            *repeat_started = true;
        }
    }

    *last_cmd_ms = now_ms;
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
    
    // Reset ramping state
    g_ramp_state.target_left = 0;
    g_ramp_state.target_right = 0;
    g_ramp_state.current_left = 0;
    g_ramp_state.current_right = 0;
}

// ===== IMPROVED MOTOR CONTROL WITH RAMPING =====

static int16_t ramp_toward(int16_t current, int16_t target, int16_t step)
{
    int16_t diff = target - current;
    
    if (diff > step) {
        return current + step;
    } else if (diff < -step) {
        return current - step;
    } else {
        return target;
    }
}

static void set_motor_with_ramp(int16_t left, int16_t right)
{
#if RAMP_ENABLED
    g_ramp_state.target_left = left;
    g_ramp_state.target_right = right;
    // Actual ramping happens in main loop
#else
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
#endif
}

static void apply_turn_left_90(void)
{
    platform_motor_set(-TURN90_SPEED, +TURN90_SPEED);
}

static void apply_turn_right_90(void)
{
    platform_motor_set(+TURN90_SPEED, -TURN90_SPEED);
}

// ===== IMPROVED TURN COMMANDS =====

static void apply_gentle_turn_left(int16_t base_speed)
{
#if TURN_MODE_GENTLE_ARC
    // Gentle arc turn: outer wheel faster, inner wheel slower
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner_speed = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(inner_speed, outer_speed);  // left slow, right fast
#elif TURN_MODE_PIVOT
    // Pivot turn (original aggressive behavior, but reduced)
    int16_t turn_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(-turn_speed, +turn_speed);
#else
    // Stationary pivot with one wheel stopped
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(0, outer_speed);
#endif
}

static void apply_gentle_turn_right(int16_t base_speed)
{
#if TURN_MODE_GENTLE_ARC
    // Gentle arc turn: outer wheel faster, inner wheel slower
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_OUTER_SPEED_PERCENT / 100);
    int16_t inner_speed = (int16_t)((int32_t)base_speed * TURN_INNER_SPEED_PERCENT / 100);
    set_motor_with_ramp(outer_speed, inner_speed);  // left fast, right slow
#elif TURN_MODE_PIVOT
    // Pivot turn (original aggressive behavior, but reduced)
    int16_t turn_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(+turn_speed, -turn_speed);
#else
    // Stationary pivot with one wheel stopped
    int16_t outer_speed = (int16_t)((int32_t)base_speed * TURN_SENSITIVITY_PERCENT / 100);
    set_motor_with_ramp(outer_speed, 0);
#endif
}

static bool try_parse_arrow_speed_char(char c, bool on, drive_mode_t mode, int16_t *speed)
{
    static uint8_t esc_state = 0u;

    if (!on || (mode != DRIVE_MODE_MANUAL)) {
        esc_state = 0u;
        return false;
    }

    if (c == 0x1B) {
        esc_state = 1u;
        return true;
    }

    if (esc_state == 1u) {
        if (c == '[') {
            esc_state = 2u;
            return true;
        }
        esc_state = 0u;
        return false;
    }

    if (esc_state == 2u) {
        esc_state = 0u;

        if (c == 'A') {
            *speed = clamp_speed((int32_t)*speed + SPEED_STEP_CMD);
            return true;
        }

        if (c == 'B') {
            *speed = clamp_speed((int32_t)*speed - SPEED_STEP_CMD);
            return true;
        }
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

static auto_action_t decide_auto_action(uint8_t black_mask,
                                         int8_t turn_threshold,
                                         uint8_t min_black_count,
                                         int8_t *sum_out,
                                         uint8_t *count_out)
{
    int8_t sum   = 0;
    uint8_t count = 0u;

    if (black_mask & IR_MASK_S1) { sum += -2; count++; }
    if (black_mask & IR_MASK_S2) { sum += -1; count++; }
    if (black_mask & IR_MASK_S3) { sum +=  0; count++; }
    if (black_mask & IR_MASK_S4) { sum += +1; count++; }
    if (black_mask & IR_MASK_S5) { sum += +2; count++; }

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

    if (sum <= -turn_threshold) {
        return AUTO_ACT_LEFT;
    }

    if (sum >= +turn_threshold) {
        return AUTO_ACT_RIGHT;
    }

    return AUTO_ACT_FORWARD;
}

static void apply_auto_action(auto_action_t act, int16_t base_speed)
{
    int16_t turn_speed;

    switch (act)
    {
        case AUTO_ACT_STOP:
            platform_motor_stop();
            break;

        case AUTO_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case AUTO_ACT_LEFT:
            turn_speed = (int16_t)((int32_t)base_speed / SOFT_TURN_DIV);
            if (turn_speed < MIN_TURN_SPEED_CMD) {
                turn_speed = MIN_TURN_SPEED_CMD;
            }
            platform_motor_set(-turn_speed, +turn_speed);
            break;

        case AUTO_ACT_RIGHT:
            turn_speed = (int16_t)((int32_t)base_speed / SOFT_TURN_DIV);
            if (turn_speed < MIN_TURN_SPEED_CMD) {
                turn_speed = MIN_TURN_SPEED_CMD;
            }
            platform_motor_set(+turn_speed, -turn_speed);
            break;

        case AUTO_ACT_CRAWL:
            {
                int16_t crawl_speed = (int16_t)((int32_t)base_speed / 2);
                platform_motor_set(+crawl_speed, +crawl_speed);
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

    buf[i++] = 'I';
    buf[i++] = 'R';
    buf[i++] = ':';
    buf[i++] = ' ';

    buf[i++] = (raw_mask & IR_MASK_S1) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S2) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S3) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S4) ? '1' : '0';
    buf[i++] = (raw_mask & IR_MASK_S5) ? '1' : '0';

    buf[i++] = ' ';
    buf[i++] = 'B';
    buf[i++] = ':';
    buf[i++] = (black_mask & IR_MASK_S1) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S2) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S3) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S4) ? '1' : '0';
    buf[i++] = (black_mask & IR_MASK_S5) ? '1' : '0';

    buf[i++] = ' ';
    buf[i++] = 'S';
    buf[i++] = 'u';
    buf[i++] = 'm';
    buf[i++] = '=';
    if (sum < 0) {
        buf[i++] = '-';
        buf[i++] = (char)('0' + (uint8_t)(-sum));
    } else {
        buf[i++] = '+';
        buf[i++] = (char)('0' + (uint8_t)sum);
    }

    buf[i++] = ' ';
    buf[i++] = 'C';
    buf[i++] = 'n';
    buf[i++] = 't';
    buf[i++] = '=';
    buf[i++] = (char)('0' + count);

    buf[i++] = ' ';

    switch (act)
    {
        case AUTO_ACT_STOP:    { buf[i++]='S'; buf[i++]='T'; buf[i++]='O'; buf[i++]='P'; } break;
        case AUTO_ACT_FORWARD: { buf[i++]='F'; buf[i++]='W'; buf[i++]='D'; } break;
        case AUTO_ACT_LEFT:    { buf[i++]='L'; buf[i++]='E'; buf[i++]='F'; buf[i++]='T'; } break;
        case AUTO_ACT_RIGHT:   { buf[i++]='R'; buf[i++]='I'; buf[i++]='G'; buf[i++]='H'; buf[i++]='T'; } break;
        case AUTO_ACT_CRAWL:   { buf[i++]='C'; buf[i++]='R'; buf[i++]='A'; buf[i++]='W'; buf[i++]='L'; } break;
        default: break;
    }

    buf[i++] = '\r';
    buf[i++] = '\n';
    platform_usart_write_buf(buf, i);
}

static void print_ir_tuning_status(int8_t turn_threshold, uint8_t min_black_count, bool debug_enabled)
{
    char buf[80];
    uint32_t i = 0u;

    buf[i++] = 'I';
    buf[i++] = 'R';
    buf[i++] = ' ';
    buf[i++] = 'T';
    buf[i++] = 'U';
    buf[i++] = 'N';
    buf[i++] = 'I';
    buf[i++] = 'N';
    buf[i++] = 'G';
    buf[i++] = ':';
    buf[i++] = ' ';

    buf[i++] = 'T';
    buf[i++] = 'u';
    buf[i++] = 'r';
    buf[i++] = 'n';
    buf[i++] = 'T';
    buf[i++] = 'h';
    buf[i++] = 'r';
    buf[i++] = 'e';
    buf[i++] = 's';
    buf[i++] = 'h';
    buf[i++] = '=';
    buf[i++] = (char)('0' + turn_threshold);

    buf[i++] = ',';
    buf[i++] = ' ';

    buf[i++] = 'M';
    buf[i++] = 'i';
    buf[i++] = 'n';
    buf[i++] = 'B';
    buf[i++] = 'l';
    buf[i++] = 'a';
    buf[i++] = 'c';
    buf[i++] = 'k';
    buf[i++] = '=';
    buf[i++] = (char)('0' + min_black_count);

    buf[i++] = ',';
    buf[i++] = ' ';

    buf[i++] = 'D';
    buf[i++] = 'e';
    buf[i++] = 'b';
    buf[i++] = 'u';
    buf[i++] = 'g';
    buf[i++] = '=';
    buf[i++] = debug_enabled ? 'O' : 'O';
    buf[i++] = debug_enabled ? 'N' : 'F';
    buf[i++] = debug_enabled ? ' ' : 'F';

    buf[i++] = '\r';
    buf[i++] = '\n';

    platform_usart_write_buf(buf, i);
}

static ultra_action_t decide_ultrasonic_action(uint16_t front_cm, uint16_t left_cm, uint16_t right_cm)
{
    bool front_ok = (front_cm >= ULTRA_FRONT_LIMIT_CM);
    bool left_ok  = (left_cm  >= ULTRA_SIDE_LIMIT_CM);
    bool right_ok = (right_cm >= ULTRA_SIDE_LIMIT_CM);

    if (!front_ok && !left_ok && !right_ok) {
        return ULTRA_ACT_STOP;
    }

    if (!front_ok) {
        if (left_cm > right_cm) {
            return ULTRA_ACT_TURN_LEFT;
        } else {
            return ULTRA_ACT_TURN_RIGHT;
        }
    }

    if (!left_ok) {
        return ULTRA_ACT_RIGHT;
    }

    if (!right_ok) {
        return ULTRA_ACT_LEFT;
    }

    return ULTRA_ACT_FORWARD;
}

static void apply_ultrasonic_action(ultra_action_t act, int16_t base_speed)
{
    int16_t half_speed;

    switch (act)
    {
        case ULTRA_ACT_STOP:
            platform_motor_stop();
            break;

        case ULTRA_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case ULTRA_ACT_LEFT:
            half_speed = (int16_t)((int32_t)base_speed / 2);
            platform_motor_set(+half_speed, +base_speed);
            break;

        case ULTRA_ACT_RIGHT:
            half_speed = (int16_t)((int32_t)base_speed / 2);
            platform_motor_set(+base_speed, +half_speed);
            break;

        case ULTRA_ACT_TURN_LEFT:
            platform_motor_set(-base_speed, +base_speed);
            break;

        case ULTRA_ACT_TURN_RIGHT:
            platform_motor_set(+base_speed, -base_speed);
            break;

        default:
            break;
    }
}

static void print_ultrasonic_status(uint16_t front_cm, uint16_t left_cm, uint16_t right_cm)
{
    char buf[48];
    uint32_t i = 0u;

    buf[i++] = 'U';
    buf[i++] = 'S';
    buf[i++] = ':';
    buf[i++] = ' ';

    buf[i++] = 'F';
    buf[i++] = '=';
    if (front_cm >= 100u) {
        buf[i++] = (char)('0' + (front_cm / 100u));
        front_cm %= 100u;
    }
    if (front_cm >= 10u) {
        buf[i++] = (char)('0' + (front_cm / 10u));
        front_cm %= 10u;
    }
    buf[i++] = (char)('0' + front_cm);

    buf[i++] = ' ';
    buf[i++] = 'L';
    buf[i++] = '=';
    if (left_cm >= 100u) {
        buf[i++] = (char)('0' + (left_cm / 100u));
        left_cm %= 100u;
    }
    if (left_cm >= 10u) {
        buf[i++] = (char)('0' + (left_cm / 10u));
        left_cm %= 10u;
    }
    buf[i++] = (char)('0' + left_cm);

    buf[i++] = ' ';
    buf[i++] = 'R';
    buf[i++] = '=';
    if (right_cm >= 100u) {
        buf[i++] = (char)('0' + (right_cm / 100u));
        right_cm %= 100u;
    }
    if (right_cm >= 10u) {
        buf[i++] = (char)('0' + (right_cm / 10u));
        right_cm %= 10u;
    }
    buf[i++] = (char)('0' + right_cm);

    buf[i++] = ' ';
    buf[i++] = 'c';
    buf[i++] = 'm';
    buf[i++] = '\r';
    buf[i++] = '\n';

    platform_usart_write_buf(buf, i);
}

static void print_ultrasonic_action(ultra_action_t act)
{
    switch (act)
    {
        case ULTRA_ACT_STOP:
            platform_usart_write_str("Action: STOP\r\n");
            break;
        case ULTRA_ACT_FORWARD:
            platform_usart_write_str("Action: FORWARD\r\n");
            break;
        case ULTRA_ACT_LEFT:
            platform_usart_write_str("Action: STEER LEFT\r\n");
            break;
        case ULTRA_ACT_RIGHT:
            platform_usart_write_str("Action: STEER RIGHT\r\n");
            break;
        case ULTRA_ACT_TURN_LEFT:
            platform_usart_write_str("Action: TURN LEFT\r\n");
            break;
        case ULTRA_ACT_TURN_RIGHT:
            platform_usart_write_str("Action: TURN RIGHT\r\n");
            break;
        default:
            break;
    }
}

int main(void)
{
    bool controls_on = false;
    drive_mode_t mode = DRIVE_MODE_MANUAL;
    int16_t current_speed = DEFAULT_SPEED_CMD;

    bool turn_active = false;
    uint32_t turn_end_ms = 0u;
    char last_move_key = 0;
    bool repeat_started = false;
    uint32_t cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
    uint32_t last_cmd_ms = 0u;
    uint32_t last_auto_ms = 0u;
    uint32_t last_ultra_ms = 0u;

    int8_t ir_turn_threshold = IR_TURN_THRESHOLD_DEFAULT;
    uint8_t ir_min_black_count = IR_MIN_COUNT_DEFAULT;
    bool ir_debug_stream_enabled = false;
    bool auto_run_enabled = false;

    uint16_t ultra_front_cm = 0u;
    uint16_t ultra_left_cm  = 0u;
    uint16_t ultra_right_cm = 0u;

    bool last_reading = false;
    bool stable_state = false;
    uint32_t last_change_ms = 0u;

    platform_initialization();
    refresh_ui(controls_on, mode, current_speed);

    while (1)
    {
        uint32_t now = platform_millis();

        // Update ramping
        update_motor_ramp(now);

        {
            bool reading = platform_button_pressed();

            if (reading != last_reading) {
                last_change_ms = now;
                last_reading = reading;
            }

            if ((now - last_change_ms) > DEBOUNCE_MS) {
                if (stable_state != reading) {
                    stable_state = reading;

                    if (stable_state) {
                        if (controls_on) {
                            system_set_off(&controls_on);
                            turn_active = false;
                            auto_run_enabled = false;
                            last_move_key = 0;
                            repeat_started = false;
                            cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
                        } else {
                            system_set_on(&controls_on);
                            auto_run_enabled = ((mode == DRIVE_MODE_AUTO_IR) ||
                                                (mode == DRIVE_MODE_AUTO_ULTRASONIC));
                            last_cmd_ms = now;
                            turn_active = false;
                            last_move_key = 0;
                            repeat_started = false;
                            cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
                        }
                        refresh_ui(controls_on, mode, current_speed);
                    }
                }
            }
        }

        if (controls_on && (mode == DRIVE_MODE_MANUAL) && turn_active) {
            if ((int32_t)(now - turn_end_ms) >= 0) {
                platform_motor_stop();
                turn_active = false;
                last_move_key = 0;
                repeat_started = false;
                cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
                last_cmd_ms = now;
            }
        }

        {
            char c;

            while (platform_usart_read_char(&c))
            {
                now = platform_millis();

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
                    mode = DRIVE_MODE_MANUAL;
                    turn_active = false;
                    auto_run_enabled = false;
                    platform_motor_stop();
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'U') || (c == 'u')) {
                    mode = DRIVE_MODE_AUTO_IR;
                    turn_active = false;
                    auto_run_enabled = controls_on;
                    platform_motor_stop();
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                if ((c == 'O') || (c == 'o')) {
                    mode = DRIVE_MODE_AUTO_ULTRASONIC;
                    turn_active = false;
                    auto_run_enabled = controls_on;
                    platform_motor_stop();
                    refresh_ui(controls_on, mode, current_speed);
                    continue;
                }

                c = to_lower(c);

                if (!controls_on) {
                    continue;
                }

                if (c == ' ') {
                    platform_motor_stop();
                    // Reset ramping state on stop
                    g_ramp_state.target_left = 0;
                    g_ramp_state.target_right = 0;
                    g_ramp_state.current_left = 0;
                    g_ramp_state.current_right = 0;
                    
                    turn_active = false;
                    if ((mode == DRIVE_MODE_AUTO_IR) || (mode == DRIVE_MODE_AUTO_ULTRASONIC)) {
                        auto_run_enabled = false;
                    }
                    last_move_key = 0;
                    repeat_started = false;
                    cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
                    last_cmd_ms = now;
                    continue;
                }

                if ((mode == DRIVE_MODE_AUTO_IR) && (c >= '1') && (c <= '4')) {
                    ir_turn_threshold = (int8_t)(c - '0');
                    print_ir_tuning_status(ir_turn_threshold, ir_min_black_count, ir_debug_stream_enabled);
                    continue;
                }

                if ((mode == DRIVE_MODE_AUTO_IR) && (c == 'n')) {
                    if (ir_min_black_count > IR_MIN_COUNT_MIN) {
                        ir_min_black_count--;
                    }
                    print_ir_tuning_status(ir_turn_threshold, ir_min_black_count, ir_debug_stream_enabled);
                    continue;
                }

                if ((mode == DRIVE_MODE_AUTO_IR) && (c == 'b')) {
                    if (ir_min_black_count < IR_MIN_COUNT_MAX) {
                        ir_min_black_count++;
                    }
                    print_ir_tuning_status(ir_turn_threshold, ir_min_black_count, ir_debug_stream_enabled);
                    continue;
                }

                if ((mode == DRIVE_MODE_AUTO_IR) && (c == 'i')) {
                    ir_debug_stream_enabled = !ir_debug_stream_enabled;
                    print_ir_tuning_status(ir_turn_threshold, ir_min_black_count, ir_debug_stream_enabled);
                    continue;
                }

                if (mode != DRIVE_MODE_MANUAL) {
                    continue;
                }

                // ===== IMPROVED MANUAL CONTROL HANDLING =====
                switch (c)
                {
                    case 'q':
                        apply_turn_left_90();
                        turn_active = true;
                        turn_end_ms = now + TURN90_MS;
                        last_cmd_ms = now;
                        break;

                    case 'e':
                        apply_turn_right_90();
                        turn_active = true;
                        turn_end_ms = now + TURN90_MS;
                        last_cmd_ms = now;
                        break;

                    case 'w':
                        turn_active = false;
                        set_motor_with_ramp(+current_speed, +current_speed);
                        note_move_cmd('w', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 's':
                        turn_active = false;
                        set_motor_with_ramp(-current_speed, -current_speed);
                        note_move_cmd('s', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 'a':
                        turn_active = false;
                        apply_gentle_turn_left(current_speed);
                        note_move_cmd('a', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 'd':
                        turn_active = false;
                        apply_gentle_turn_right(current_speed);
                        note_move_cmd('d', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    default:
                        break;
                }
            }
        }

        if (controls_on && (mode == DRIVE_MODE_AUTO_ULTRASONIC) && auto_run_enabled) {
            if ((now - last_ultra_ms) >= ULTRA_POLL_MS) {
                bool ok_f = platform_ultrasonic_read_cm(ULTRA_FRONT, &ultra_front_cm);
                bool ok_l = platform_ultrasonic_read_cm(ULTRA_LEFT,  &ultra_left_cm);
                bool ok_r = platform_ultrasonic_read_cm(ULTRA_RIGHT, &ultra_right_cm);

                if (ok_f && ok_l && ok_r) {
                    ultra_action_t ultra_act = decide_ultrasonic_action(ultra_front_cm,
                                                                        ultra_left_cm,
                                                                        ultra_right_cm);
                    print_ultrasonic_status(ultra_front_cm, ultra_left_cm, ultra_right_cm);
                    apply_ultrasonic_action(ultra_act, current_speed);
                    print_ultrasonic_action(ultra_act);
                } else {
                    platform_motor_stop();
                    platform_usart_write_str("US: read timeout\r\n");
                }

                last_ultra_ms = now;
            }
        }

        if (controls_on && (mode == DRIVE_MODE_AUTO_IR) && auto_run_enabled) {
            if ((now - last_auto_ms) >= AUTO_LOOP_MS) {
                uint8_t raw_mask   = platform_ir_read_mask_raw();
                uint8_t black_mask = ir_mask_on_black(raw_mask);
                int8_t sum = 0;
                uint8_t count = 0u;
                auto_action_t act  = decide_auto_action(black_mask,
                                                        ir_turn_threshold,
                                                        ir_min_black_count,
                                                        &sum,
                                                        &count);

                apply_auto_action(act, current_speed);

                if (ir_debug_stream_enabled) {
                    print_ir_debug_status(raw_mask, black_mask, sum, count, act);
                }

                last_auto_ms = now;
            }
        }

        if (controls_on && (mode == DRIVE_MODE_MANUAL) && !turn_active) {
            if ((now - last_cmd_ms) > cmd_timeout_ms) {
                platform_motor_stop();
                // Reset ramping on timeout
                g_ramp_state.target_left = 0;
                g_ramp_state.target_right = 0;
                g_ramp_state.current_left = 0;
                g_ramp_state.current_right = 0;
            }
        }
    }
}