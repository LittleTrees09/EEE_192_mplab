#include "platform.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS               30u
#define CMD_TIMEOUT_FIRST_MS      900u
#define CMD_TIMEOUT_REPEAT_MS     100u
#define AUTO_LOOP_MS              10u

#define DEFAULT_SPEED_CMD         300
#define MIN_SPEED_CMD             0
#define MAX_SPEED_CMD             1000
#define SPEED_STEP_CMD            100

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
#define IR_TURN_THRESHOLD_DEFAULT  2

#define IR_MIN_COUNT_MIN           1u
#define IR_MIN_COUNT_MAX           5u
#define IR_MIN_COUNT_DEFAULT       2u   /* 2 = require a stronger/closer hit by default */

#define IR_POLICY_MOVE_IF_NONE      0u
#define IR_POLICY_MOVE_IF_DETECT    1u
/* One-line behavior switch: set to IR_POLICY_MOVE_IF_NONE or IR_POLICY_MOVE_IF_DETECT */
#define IR_AUTO_POLICY              IR_POLICY_MOVE_IF_NONE

/*
 * If no IR sensor sees black, either stop or crawl forward at half speed.
 * Set to 0 if you want a full stop instead of a search crawl.
 */
#define IR_NO_DETECT_CRAWL_ON_EMPTY 0u

/*
 * IMPORTANT:
 * Change this only if your sensor polarity is opposite.
 *
 * 0 = sensor output goes LOW when the sensor sees BLACK  <-- TCRT5000 with pull-up (default)
 * 1 = sensor output goes HIGH when the sensor sees BLACK
 *
 * gpio.c configures IR pins with pull-ups (INEN | PULLEN).
 * TCRT5000 open-collector output pulls LOW over black, so active = LOW = 0.
 */
#define IR_ACTIVE_ON_BLACK_HIGH   1u

/*
 * AUTO TURNING TUNING
 */
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
"MODE: MANUAL\r\n"
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
"  A = turn left\r\n"
"  D = turn right\r\n"
"  Q = ~90 deg left\r\n"
"  E = ~90 deg right\r\n"
"  SPACE = stop\r\n"
"\r\n"
"Manual speed control:\r\n"
"  UP ARROW    = increase speed\r\n"
"  DOWN ARROW  = decrease speed\r\n"
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
        default:  return "Speed: 0%\r\n";
    }
}

static bool try_parse_arrow_speed_char(char c,
                                       bool controls_on,
                                       drive_mode_t mode,
                                       int16_t *current_speed)
{
    typedef enum
    {
        ESC_IDLE = 0,
        ESC_SEEN,
        ESC_BRACKET
    } esc_state_t;

    static esc_state_t esc_state = ESC_IDLE;

    if (!current_speed) return false;

    switch (esc_state)
    {
        case ESC_IDLE:
            if ((uint8_t)c == 0x1Bu) {
                esc_state = ESC_SEEN;
                return true;
            }
            return false;

        case ESC_SEEN:
            if (c == '[') {
                esc_state = ESC_BRACKET;
                return true;
            }

            esc_state = ESC_IDLE;
            return false;

        case ESC_BRACKET:
            esc_state = ESC_IDLE;

            if (!controls_on || (mode != DRIVE_MODE_MANUAL)) {
                return true;
            }

            if (c == 'A') {
                *current_speed = clamp_speed((int32_t)(*current_speed) + SPEED_STEP_CMD);
                return true;
            }

            if (c == 'B') {
                *current_speed = clamp_speed((int32_t)(*current_speed) - SPEED_STEP_CMD);
                return true;
            }

            return true;
    }

    esc_state = ESC_IDLE;
    return false;
}

static uint8_t ir_mask_on_black(uint8_t raw_mask)
{
#if IR_ACTIVE_ON_BLACK_HIGH
    return (raw_mask & IR_MASK_ALL);
#else
    return ((uint8_t)(~raw_mask) & IR_MASK_ALL);
#endif
}

static auto_action_t decide_auto_action(uint8_t black_mask,
                                        int8_t turn_threshold,
                                        uint8_t min_black_count,
                                        int8_t *sum_out,
                                        uint8_t *count_out)
{
    int8_t sum = 0;
    uint8_t count = 0u;

#if (IR_AUTO_POLICY == IR_POLICY_MOVE_IF_DETECT)
    /* Classic line-follow policy: move only when black is detected. */
    if (black_mask == 0u) {
        if (sum_out)   *sum_out   = 0;
        if (count_out) *count_out = 0u;
    #if IR_NO_DETECT_CRAWL_ON_EMPTY
        return AUTO_ACT_CRAWL;
    #else
        return AUTO_ACT_STOP;
    #endif
    }

    if (black_mask & IR_MASK_S1) { sum -= 2; count++; }
    if (black_mask & IR_MASK_S2) { sum -= 1; count++; }
    if (black_mask & IR_MASK_S3) {             count++; }
    if (black_mask & IR_MASK_S4) { sum += 1; count++; }
    if (black_mask & IR_MASK_S5) { sum += 2; count++; }

    if (sum_out)   *sum_out   = sum;
    if (count_out) *count_out = count;

    /* Optional guard for full-array hit. */
    if (count >= IR_MASK_ALL) {
        return AUTO_ACT_STOP;
    }

    if (count < min_black_count) {
        return AUTO_ACT_STOP;
    }

    if (sum <= -turn_threshold) {
        return AUTO_ACT_LEFT;
    }

    if (sum >= turn_threshold) {
        return AUTO_ACT_RIGHT;
    }

    return AUTO_ACT_FORWARD;
#else
    /* Inverted policy: move when nothing is detected, stop on any detection. */
    if (black_mask == 0u) {
        if (sum_out)   *sum_out   = 0;
        if (count_out) *count_out = 0u;
        return AUTO_ACT_FORWARD;
    }

    if (black_mask & IR_MASK_S1) { sum -= 2; count++; }
    if (black_mask & IR_MASK_S2) { sum -= 1; count++; }
    if (black_mask & IR_MASK_S3) {             count++; }
    if (black_mask & IR_MASK_S4) { sum += 1; count++; }
    if (black_mask & IR_MASK_S5) { sum += 2; count++; }

    if (sum_out)   *sum_out   = sum;
    if (count_out) *count_out = count;

    (void)turn_threshold;
    (void)min_black_count;
    return AUTO_ACT_STOP;
#endif
}

static void apply_auto_action(auto_action_t action, int16_t base_speed)
{
    int16_t slow_speed;
    int16_t crawl_speed;

    if (base_speed < 0) {
        base_speed = -base_speed;
    }
    if (base_speed > MAX_SPEED_CMD) {
        base_speed = MAX_SPEED_CMD;
    }

    slow_speed = base_speed / SOFT_TURN_DIV;
    if (slow_speed < MIN_TURN_SPEED_CMD) {
        slow_speed = MIN_TURN_SPEED_CMD;
    }
    if (slow_speed > base_speed) {
        slow_speed = base_speed;
    }

    crawl_speed = base_speed / 2;
    if (crawl_speed < MIN_TURN_SPEED_CMD) {
        crawl_speed = MIN_TURN_SPEED_CMD;
    }
    if (crawl_speed > base_speed) {
        crawl_speed = base_speed;
    }

    switch (action)
    {
        case AUTO_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case AUTO_ACT_LEFT:
            // Black detected left: steer left → slow the RIGHT motor
            platform_motor_set(+base_speed, +slow_speed);
            break;

        case AUTO_ACT_RIGHT:
            // Black detected right: steer right → slow the LEFT motor
            platform_motor_set(+slow_speed, +base_speed);
            break;

        case AUTO_ACT_CRAWL:
            platform_motor_set(+crawl_speed, +crawl_speed);
            break;

        case AUTO_ACT_STOP:
        default:
            platform_motor_stop();
            break;
    }
}

static ultra_action_t decide_ultrasonic_action(uint16_t front_cm,
                                               uint16_t left_cm,
                                               uint16_t right_cm)
{
    bool front_blocked = (front_cm > 0u) && (front_cm <= ULTRA_FRONT_LIMIT_CM);
    bool left_blocked  = (left_cm  > 0u) && (left_cm  <= ULTRA_SIDE_LIMIT_CM);
    bool right_blocked = (right_cm > 0u) && (right_cm <= ULTRA_SIDE_LIMIT_CM);

    /*
     * Reactive ultrasonic mode:
     * - No obstacle detected     -> STOP
     * - Front obstacle detected  -> turn away
     * - Left obstacle detected   -> steer right
     * - Right obstacle detected  -> steer left
     *
     * This prevents the robot from driving on its own when nothing is sensed.
     */
    if (front_blocked) {
        if (left_blocked && !right_blocked) {
            return ULTRA_ACT_TURN_RIGHT;
        }
        if (right_blocked && !left_blocked) {
            return ULTRA_ACT_TURN_LEFT;
        }
        if (left_cm >= right_cm) {
            return ULTRA_ACT_TURN_LEFT;
        }
        return ULTRA_ACT_TURN_RIGHT;
    }

    if (left_blocked && right_blocked) {
        return ULTRA_ACT_STOP;
    }

    if (left_blocked) {
        return ULTRA_ACT_RIGHT;
    }

    if (right_blocked) {
        return ULTRA_ACT_LEFT;
    }

    return ULTRA_ACT_STOP;
}

static void apply_ultrasonic_action(ultra_action_t action, int16_t base_speed)
{
    int16_t slow_speed;

    if (base_speed < 0) {
        base_speed = -base_speed;
    }
    if (base_speed > MAX_SPEED_CMD) {
        base_speed = MAX_SPEED_CMD;
    }

    slow_speed = base_speed / SOFT_TURN_DIV;
    if (slow_speed < MIN_TURN_SPEED_CMD) {
        slow_speed = MIN_TURN_SPEED_CMD;
    }
    if (slow_speed > base_speed) {
        slow_speed = base_speed;
    }

    switch (action)
    {
        case ULTRA_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case ULTRA_ACT_LEFT:
            platform_motor_set(+slow_speed, +base_speed);
            break;

        case ULTRA_ACT_RIGHT:
            platform_motor_set(+base_speed, +slow_speed);
            break;

        case ULTRA_ACT_TURN_LEFT:
            platform_motor_set(-base_speed, +base_speed);
            break;

        case ULTRA_ACT_TURN_RIGHT:
            platform_motor_set(+base_speed, -base_speed);
            break;

        case ULTRA_ACT_STOP:
        default:
            platform_motor_stop();
            break;
    }
}

static void refresh_ui(bool controls_on, drive_mode_t mode, int16_t current_speed)
{
    if (!controls_on) {
        platform_usart_write_str(UI_OFF);
        return;
    }

    if (mode == DRIVE_MODE_AUTO_IR) {
        platform_usart_write_str(UI_AUTO_IR);
    } else if (mode == DRIVE_MODE_AUTO_ULTRASONIC) {
        platform_usart_write_str(UI_AUTO_ULTRA);
    } else {
        platform_usart_write_str(UI_MANUAL_HEAD);
        platform_usart_write_str(speed_percent_text(current_speed));
    }
}

static void system_set_off(bool *controls_on)
{
    platform_motor_stop();
    platform_tb6612_enable(false);
    platform_led_set(false);
    *controls_on = false;
}

static void system_set_on(bool *controls_on)
{
    platform_motor_stop();
    platform_tb6612_enable(true);
    platform_led_set(true);
    *controls_on = true;
}

static inline void apply_turn_left_90(void)  { platform_motor_set(+TURN90_SPEED, 0); }
static inline void apply_turn_right_90(void) { platform_motor_set(0, +TURN90_SPEED); }

static void usart_write_u16(uint16_t v)
{
    char buf[6];
    int i = 0;

    if (v == 0u) {
        platform_usart_write_char('0');
        return;
    }

    while ((v > 0u) && (i < (int)sizeof(buf))) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }

    while (i > 0) {
        platform_usart_write_char(buf[--i]);
    }
}

static void print_ultrasonic_status(uint16_t front_cm, uint16_t left_cm, uint16_t right_cm)
{
    platform_usart_write_str("US: F=");
    usart_write_u16(front_cm);
    platform_usart_write_str("cm L=");
    usart_write_u16(left_cm);
    platform_usart_write_str("cm R=");
    usart_write_u16(right_cm);
    platform_usart_write_str("cm\r\n");
}

static void print_ultrasonic_action(ultra_action_t act)
{
    platform_usart_write_str("US act=");

    switch (act)
    {
        case ULTRA_ACT_FORWARD:
            platform_usart_write_str("FWD");
            break;
        case ULTRA_ACT_LEFT:
            platform_usart_write_str("LEFT");
            break;
        case ULTRA_ACT_RIGHT:
            platform_usart_write_str("RIGHT");
            break;
        case ULTRA_ACT_TURN_LEFT:
            platform_usart_write_str("TURN_LEFT");
            break;
        case ULTRA_ACT_TURN_RIGHT:
            platform_usart_write_str("TURN_RIGHT");
            break;
        case ULTRA_ACT_STOP:
        default:
            platform_usart_write_str("STOP");
            break;
    }

    platform_usart_write_str("\r\n");
}

static char nibble_to_hex(uint8_t n)
{
    n &= 0x0Fu;
    if (n < 10u) {
        return (char)('0' + n);
    }
    return (char)('A' + (n - 10u));
}

static void print_ir_tuning_status(int8_t turn_threshold, uint8_t min_black_count, bool stream_on)
{
    platform_usart_write_str("IR tune: thr=");
    platform_usart_write_char((char)('0' + (uint8_t)turn_threshold));
    platform_usart_write_str(" minCnt=");
    platform_usart_write_char((char)('0' + min_black_count));
    platform_usart_write_str(" stream=");
    platform_usart_write_str(stream_on ? "ON\r\n" : "OFF\r\n");
}

static void print_ir_debug_status(uint8_t raw_mask,
                                  uint8_t black_mask,
                                  int8_t sum,
                                  uint8_t count,
                                  auto_action_t act)
{
    platform_usart_write_str("[IR] Raw=0x");
    platform_usart_write_char(nibble_to_hex(raw_mask));
    platform_usart_write_str(" Det=0x");
    platform_usart_write_char(nibble_to_hex(black_mask));
    platform_usart_write_str(" Sum=");

    if (sum < 0) {
        platform_usart_write_char('-');
        usart_write_u16((uint16_t)(-sum));
    } else if (sum > 0) {
        platform_usart_write_char('+');
        usart_write_u16((uint16_t)sum);
    } else {
        platform_usart_write_char('0');
    }

    platform_usart_write_str(" Cnt=");
    usart_write_u16(count);
    platform_usart_write_str(" Act=");

    switch (act)
    {
        case AUTO_ACT_FORWARD:
            platform_usart_write_str("FWD");
            break;
        case AUTO_ACT_LEFT:
            platform_usart_write_str("LEFT");
            break;
        case AUTO_ACT_RIGHT:
            platform_usart_write_str("RIGHT");
            break;
        case AUTO_ACT_CRAWL:
            platform_usart_write_str("CRAWL");
            break;
        case AUTO_ACT_STOP:
        default:
            platform_usart_write_str("STOP");
            break;
    }

    platform_usart_write_str("\r\n");
}

int main(void)
{
    bool controls_on = false;
    drive_mode_t mode = DRIVE_MODE_MANUAL;

    uint32_t now;
    uint32_t last_cmd_ms = 0u;
    uint32_t last_auto_ms = 0u;
    uint32_t turn_end_ms = 0u;
    uint32_t last_change_ms = 0u;
    uint32_t cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;

    int16_t current_speed = DEFAULT_SPEED_CMD;

    uint32_t last_ultra_ms = 0u;
    uint16_t ultra_front_cm = 0u;
    uint16_t ultra_left_cm  = 0u;
    uint16_t ultra_right_cm = 0u;

    bool repeat_started = false;
    bool turn_active = false;
    bool auto_run_enabled = false;
    bool ir_debug_stream_enabled = false;
    bool last_reading = false;
    bool stable_state = false;
    char last_move_key = 0;

    int8_t ir_turn_threshold = IR_TURN_THRESHOLD_DEFAULT;
    uint8_t ir_min_black_count = IR_MIN_COUNT_DEFAULT;

    platform_initialization();
    refresh_ui(false, mode, current_speed);

    for (;;)
    {
        now = platform_millis();

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
                        platform_motor_set(+current_speed, +current_speed);
                        note_move_cmd('w', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 's':
                        turn_active = false;
                        platform_motor_set(-current_speed, -current_speed);
                        note_move_cmd('s', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 'a':
                        turn_active = false;
                        platform_motor_set(+current_speed, -current_speed);
                        note_move_cmd('a', now, &last_cmd_ms, &last_move_key, &cmd_timeout_ms, &repeat_started);
                        break;

                    case 'd':
                        turn_active = false;
                        platform_motor_set(-current_speed, +current_speed);
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
            }
        }
    }
}