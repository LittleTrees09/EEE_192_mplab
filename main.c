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

/*
 * IMPORTANT:
 * Change this only if your sensor polarity is opposite.
 *
 * 0 = sensor output goes LOW when the sensor sees BLACK
 * 1 = sensor output goes HIGH when the sensor sees BLACK
 */
#define IR_ACTIVE_ON_BLACK_HIGH   1u

/*
 * AUTO TURNING TUNING
 *
 * These let you manually adjust how strongly the robot turns.
 *
 * SOFT_TURN_DIV:
 *   base_speed / SOFT_TURN_DIV is used for the slower side while turning.
 *   2 = stronger turn
 *   3 = softer turn
 *
 * MIN_TURN_SPEED_CMD:
 *   prevents the slower side from becoming too weak at low speed settings.
 */
#define SOFT_TURN_DIV             2
#define MIN_TURN_SPEED_CMD        120

typedef enum
{
    DRIVE_MODE_MANUAL = 0,
    DRIVE_MODE_AUTO_LINE
} drive_mode_t;

typedef enum
{
    AUTO_ACT_STOP = 0,
    AUTO_ACT_FORWARD,
    AUTO_ACT_LEFT,
    AUTO_ACT_RIGHT
} auto_action_t;

static const char UI_OFF[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"\r\n"
"STATUS: OFF\r\n"
"Press onboard button to enable controls.\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic black-surface follow mode\r\n";

static const char UI_MANUAL_HEAD[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: MANUAL\r\n"
"Baud: 38400\r\n"
"\r\n"
"Mode commands:\r\n"
"  M = manual mode\r\n"
"  U = automatic black-surface follow mode\r\n"
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

static const char UI_AUTO[] =
"\033[2J\033[H"
"MoBot Control\r\n"
"STATUS: ON\r\n"
"MODE: AUTO BLACK-SURFACE FOLLOW\r\n"
"Baud: 38400\r\n"
"\r\n"
"Auto behavior:\r\n"
"  - Stop if NO sensor sees black\r\n"
"  - Move toward where black is detected\r\n"
"  - Left-side black  -> turn left\r\n"
"  - Right-side black -> turn right\r\n"
"  - Center / balanced black -> forward\r\n"
"\r\n"
"SPACE stops the motors; send U again to resume auto.\r\n";

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

    /* round to nearest 10% */
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

            if (c == 'A') { /* Up Arrow */
                *current_speed = clamp_speed((int32_t)(*current_speed) + SPEED_STEP_CMD);
                return true;
            }

            if (c == 'B') { /* Down Arrow */
                *current_speed = clamp_speed((int32_t)(*current_speed) - SPEED_STEP_CMD);
                return true;
            }

            return true;
    }

    esc_state = ESC_IDLE;
    return false;
}

/*
 * Convert the raw GPIO reading into a mask where:
 * bit = 1 means "this sensor sees BLACK"
 *
 * Edit IR_ACTIVE_ON_BLACK_HIGH above if your sensors behave opposite.
 */
static uint8_t ir_mask_on_black(uint8_t raw_mask)
{
#if IR_ACTIVE_ON_BLACK_HIGH
    return (raw_mask & IR_MASK_ALL);
#else
    return ((uint8_t)(~raw_mask) & IR_MASK_ALL);
#endif
}

/*
 * AUTO DECISION LOGIC
 *
 * Goal:
 * - If NO sensor sees black -> STOP
 * - If black is more on the LEFT  -> turn LEFT
 * - If black is more on the RIGHT -> turn RIGHT
 * - If black is centered/balanced -> move FORWARD
 *
 * Sensor layout:
 *   S1 S2 S3 S4 S5
 *   L  L  C  R  R
 *
 * Weighting:
 *   S1 = -2
 *   S2 = -1
 *   S3 =  0
 *   S4 = +1
 *   S5 = +2
 *
 * Negative sum  -> black is more on the LEFT
 * Positive sum  -> black is more on the RIGHT
 * Zero sum      -> black is centered/balanced
 */
static auto_action_t decide_auto_action(uint8_t black_mask)
{
    int8_t sum = 0;
    uint8_t count = 0u;

    /* Stop if NOTHING is detected as black */
    if (black_mask == 0u) {
        return AUTO_ACT_STOP;
    }

    if (black_mask & IR_MASK_S1) { sum -= 2; count++; }
    if (black_mask & IR_MASK_S2) { sum -= 1; count++; }
    if (black_mask & IR_MASK_S3) {             count++; }
    if (black_mask & IR_MASK_S4) { sum += 1; count++; }
    if (black_mask & IR_MASK_S5) { sum += 2; count++; }

    if (count == 0u) {
        return AUTO_ACT_STOP;
    }

    /*
     * You can change these thresholds later:
     * - stricter thresholds -> more forward behavior
     * - looser thresholds   -> more turning behavior
     */
    if (sum <= -1) {
        return AUTO_ACT_LEFT;
    }

    if (sum >= 1) {
        return AUTO_ACT_RIGHT;
    }

    return AUTO_ACT_FORWARD;
}

/*
 * AUTO MOTOR RESPONSE
 *
 * FORWARD:
 *   both sides move at base speed
 *
 * LEFT:
 *   LEFT motor stops
 *   RIGHT motor moves
 *
 * RIGHT:
 *   RIGHT motor stops
 *   LEFT motor moves
 *
 * If your chassis still turns the wrong way after this,
 * swap the LEFT and RIGHT cases below.
 */
static void apply_auto_action(auto_action_t action, int16_t base_speed)
{
    if (base_speed < 0) {
        base_speed = -base_speed;
    }
    if (base_speed > MAX_SPEED_CMD) {
        base_speed = MAX_SPEED_CMD;
    }

    switch (action)
    {
        case AUTO_ACT_FORWARD:
            platform_motor_set(+base_speed, +base_speed);
            break;

        case AUTO_ACT_LEFT:
            /* Sharp left turn: left motor stops, right motor moves */
            platform_motor_set(+base_speed, 0);
            break;

        case AUTO_ACT_RIGHT:
            /* Sharp right turn: right motor stops, left motor moves */
            platform_motor_set(0, +base_speed);
            break;

        case AUTO_ACT_STOP:
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

    if (mode == DRIVE_MODE_AUTO_LINE) {
        platform_usart_write_str(UI_AUTO);
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

    bool repeat_started = false;
    bool turn_active = false;
    bool auto_run_enabled = false;
    bool last_reading = false;
    bool stable_state = false;
    char last_move_key = 0;

    platform_initialization();
    refresh_ui(false, mode, current_speed);

    for (;;)
    {
        now = platform_millis();

        /* Debounced onboard button handling */
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
                            auto_run_enabled = (mode == DRIVE_MODE_AUTO_LINE);
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

        /* End timed 90-degree turn in manual mode */
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

        /* Read UART commands */
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
                    mode = DRIVE_MODE_AUTO_LINE;
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
                    if (mode == DRIVE_MODE_AUTO_LINE) {
                        auto_run_enabled = false;
                    }
                    last_move_key = 0;
                    repeat_started = false;
                    cmd_timeout_ms = CMD_TIMEOUT_FIRST_MS;
                    last_cmd_ms = now;
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

        /* AUTO MODE LOOP:
         * Follow BLACK surface if any sensor sees it.
         * Stop only when NO sensor sees black.
         */
        if (controls_on && (mode == DRIVE_MODE_AUTO_LINE) && auto_run_enabled) {
            if ((now - last_auto_ms) >= AUTO_LOOP_MS) {
                uint8_t raw_mask   = platform_ir_read_mask_raw();
                uint8_t black_mask = ir_mask_on_black(raw_mask);
                auto_action_t act  = decide_auto_action(black_mask);

                apply_auto_action(act, current_speed);
                last_auto_ms = now;
            }
        }

        /* Manual-mode release timeout */
        if (controls_on && (mode == DRIVE_MODE_MANUAL) && !turn_active) {
            if ((now - last_cmd_ms) > cmd_timeout_ms) {
                platform_motor_stop();
            }
        }
    }
}
