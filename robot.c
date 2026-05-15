#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include "platform.h"
#include "robot.h"

#if defined(PORT_SEC_REGS)
  #define PORTX PORT_SEC_REGS
#else
  #define PORTX PORT_REGS
#endif

extern volatile uint32_t g_tick_100us;

/* -------------------------------------------------------------------------
 * Ultrasonic pin masks (PA16 shared TRIG, PA17/18/19 ECHO)
 * ---------------------------------------------------------------------- */
#define MASK_TRIG    (1u << 16u)
#define MASK_ECHO_F  (1u << 17u)
#define MASK_ECHO_L  (1u << 18u)
#define MASK_ECHO_R  (1u << 19u)

/* -------------------------------------------------------------------------
 * Motor speed constants (TB6612 via platform_motor_set, ±1000 range)
 * ---------------------------------------------------------------------- */
#define SPD_DRIVE   350
#define SPD_SLOW    200
#define SPD_NUDGE   200
#define SPD_SQUARE  150
#define SPD_STEER   290

/* -------------------------------------------------------------------------
 * Sensor data arrays — indices 2–7 (aliased per guide Section 5.2)
 * ---------------------------------------------------------------------- */
uint32_t durat[8];
uint32_t distcm[8];
uint32_t distin[8];

/* -------------------------------------------------------------------------
 * Stop flag — set by delay_ms() when button or stop key is detected.
 * Every inner loop checks this so the algorithm unwinds cleanly.
 * ---------------------------------------------------------------------- */
static volatile uint8_t g_robot_stop = 0u;

/* -------------------------------------------------------------------------
 * Navigation state
 * ---------------------------------------------------------------------- */
static int room             = 0;
static int room_corner      = 0;
static int intersection     = 0;
static int room_one_disable = 0;
static int room_two_disable = 0;
static int room_3_disable   = 0;
static int corner_obst      = 0;
static int obst_count       = 0;
static int stepcount        = 0;
static int walldist         = 600;
static int obst_delay       = 500;

/* =========================================================================
 * PRIMITIVES
 * ====================================================================== */

/*
 * delay_ms — interruptible blocking delay.
 * Polls the onboard button and serial port every ms.
 * Space, N, or B (upper or lower) trigger a stop; the button also stops.
 * Sets g_robot_stop and returns early so every caller unwinds naturally.
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = platform_millis();
    while ((platform_millis() - start) < ms)
    {
        if (g_robot_stop) { return; }

        if (platform_button_pressed()) { g_robot_stop = 1u; return; }

        char c;
        if (platform_usart_read_char(&c))
        {
            if (c == ' ' || c == 'n' || c == 'N' || c == 'b' || c == 'B')
            {
                g_robot_stop = 1u;
                return;
            }
        }

        asm("nop");
    }
}

static uint32_t get_micros(void)
{
    return g_tick_100us * 100u;
}

/*
 * motor_cmd — maps the guide's 2-byte command codes to platform_motor_set().
 * Immediately stops if g_robot_stop is set.
 */
static void motor_cmd(uint8_t a, uint8_t b)
{
    if (g_robot_stop) { platform_motor_stop(); return; }

    /* STOP */
    if (a == 64u && b == 192u) { platform_motor_stop(); return; }

    /* FORWARD */
    if (a == 47u && b == 175u) { platform_motor_set( SPD_DRIVE,  SPD_DRIVE); return; }

    /* TURN RIGHT (pivot) */
    if (a == 96u && b == 160u) { platform_motor_set( SPD_DRIVE, -SPD_DRIVE); return; }

    /* TURN LEFT (pivot) */
    if (a == 42u && b == 168u) { platform_motor_set(-SPD_DRIVE,  SPD_DRIVE); return; }

    /* NUDGE RIGHT */
    if (a == 50u && b == 140u) { platform_motor_set( SPD_DRIVE,  SPD_NUDGE); return; }

    /* NUDGE LEFT */
    if (a == 21u && b == 178u) { platform_motor_set( SPD_NUDGE,  SPD_DRIVE); return; }

    /* ALIGN LEFT (42,175) — distinct from TURN LEFT (42,168) */
    if (a == 42u && b == 175u) { platform_motor_set( SPD_NUDGE,  SPD_DRIVE); return; }

    /* STEER RIGHT */
    if (a == 47u && b == 148u) { platform_motor_set( SPD_DRIVE,  SPD_STEER); return; }

    /* STEER LEFT (20,175) */
    if (a == 20u && b == 175u) { platform_motor_set( SPD_STEER,  SPD_DRIVE); return; }

    /* SQUARE CW */
    if (a == 50u && b == 216u) { platform_motor_set( SPD_SQUARE, -SPD_SQUARE); return; }

    /* SQUARE CCW */
    if (a == 78u && b == 178u) { platform_motor_set(-SPD_SQUARE,  SPD_SQUARE); return; }

    /* FORWARD SLOW (42,210) — distinct from TURN LEFT (42,168) and ALIGN (42,175) */
    if (a == 42u && b == 210u) { platform_motor_set( SPD_SLOW,   SPD_SLOW); return; }
}

/* =========================================================================
 * PING — fire shared PA16 TRIG, concurrently time PA17/18/19 ECHO pulses.
 * Timeout per sensor: 300 ticks = 30 000 µs.
 * ====================================================================== */
static void ping(void)
{
    uint8_t  started[3] = {0u, 0u, 0u};
    uint8_t  done[3]    = {0u, 0u, 0u};
    uint32_t t_start[3] = {0u, 0u, 0u};
    uint32_t t_end[3]   = {0u, 0u, 0u};
    uint32_t dur[3];
    uint8_t  i;

    static const uint32_t echo_mask[3] = {MASK_ECHO_F, MASK_ECHO_L, MASK_ECHO_R};

    if (g_robot_stop) { return; }

    /* Fire shared trigger: guaranteed ≥100 µs pulse (2 tick minimum) */
    PORTX->GROUP[0].PORT_OUTCLR = MASK_TRIG;
    {
        uint32_t t0 = g_tick_100us;
        PORTX->GROUP[0].PORT_OUTSET = MASK_TRIG;
        while ((g_tick_100us - t0) < 2u) { asm("nop"); }
        PORTX->GROUP[0].PORT_OUTCLR = MASK_TRIG;
    }

    /* Concurrent ECHO polling — 300 ticks = 30 000 µs total timeout */
    {
        uint32_t poll_start = g_tick_100us;
        while ((g_tick_100us - poll_start) < 300u)
        {
            uint32_t port_in   = PORTX->GROUP[0].PORT_IN;
            uint32_t now_ticks = g_tick_100us;

            for (i = 0u; i < 3u; i++)
            {
                if (!done[i])
                {
                    if (!started[i] && (port_in & echo_mask[i]))
                    {
                        t_start[i] = now_ticks;
                        started[i] = 1u;
                    }
                    else if (started[i] && !(port_in & echo_mask[i]))
                    {
                        t_end[i] = now_ticks;
                        done[i]  = 1u;
                    }
                }
            }

            if (done[0] && done[1] && done[2]) { break; }
        }
    }

    for (i = 0u; i < 3u; i++)
    {
        dur[i] = (started[i] && done[i]) ? ((t_end[i] - t_start[i]) * 100u) : 0u;
    }

    /* Alias table — Section 5.2 */
    durat[4]  = dur[0]; durat[7]  = dur[0];
    distcm[4] = dur[0] / 58u; distcm[7] = dur[0] / 58u;
    distin[4] = dur[0] / 148u; distin[7] = dur[0] / 148u;

    durat[2]  = dur[1]; durat[3]  = dur[1];
    distcm[2] = dur[1] / 58u; distcm[3] = dur[1] / 58u;
    distin[2] = dur[1] / 148u; distin[3] = dur[1] / 148u;

    durat[5]  = dur[2]; durat[6]  = dur[2];
    distcm[5] = dur[2] / 58u; distcm[6] = dur[2] / 58u;
    distin[5] = dur[2] / 148u; distin[6] = dur[2] / 148u;
}

/* =========================================================================
 * SQUAREUP
 * ====================================================================== */
static void squareup(void)
{
    uint32_t d3, d5, delta;

    for (;;)
    {
        if (g_robot_stop) { return; }

        d3    = durat[3];
        d5    = durat[5];
        delta = (d3 > d5) ? (d3 - d5) : (d5 - d3);

        if (delta <= 70u) { break; }

        if (room == 3 && intersection >= 1)
        {
            motor_cmd(64u, 192u);
            delay_ms(3000u);
            if (g_robot_stop) { return; }
        }

        if (d3 > d5)
        {
            motor_cmd(50u, 216u);
            delay_ms(50u);
            motor_cmd(64u, 192u);
        }
        else
        {
            motor_cmd(78u, 178u);
            delay_ms(50u);
            motor_cmd(64u, 192u);
        }

        if (g_robot_stop) { return; }
        ping();
    }
}

/* =========================================================================
 * OBSTACLE
 * ====================================================================== */
static void obstacle(void)
{
    uint32_t obst_delta;
    uint32_t d3, d5, d4, d6;

    while (obst_count <= 1 && !g_robot_stop)
    {
        motor_cmd(64u, 192u);
        delay_ms(200u);
        if (g_robot_stop) { return; }

        ping();
        while (distcm[6] <= 15u && !g_robot_stop)
        {
            motor_cmd(42u, 168u);
            delay_ms(30u);
            motor_cmd(64u, 192u);
            ping();
        }
        if (g_robot_stop) { return; }

        if (obst_count <= 1)
        {
            motor_cmd(42u, 168u);
            delay_ms(170u);
        }
        else
        {
            do
            {
                motor_cmd(42u, 210u);
                delay_ms(80u);
                motor_cmd(64u, 192u);
                ping();
                d5 = durat[5]; d3 = durat[3];
                obst_delta = (d5 > d3) ? (d5 - d3) : (d3 - d5);
            } while (obst_delta > 50u && !g_robot_stop);
        }
        if (g_robot_stop) { return; }

        motor_cmd(64u, 192u);
        delay_ms(1000u);
        if (g_robot_stop) { return; }

        motor_cmd(42u, 210u);
        delay_ms(420u);
        motor_cmd(64u, 192u);
        if (g_robot_stop) { return; }

        do
        {
            motor_cmd(42u, 210u);
            delay_ms(80u);
            motor_cmd(64u, 192u);
            ping();
            d3 = durat[3]; d5 = durat[5];
            d4 = durat[4]; d6 = durat[6];
            if (obst_count >= 1)
                obst_delta = (d3 > d5) ? (d3 - d5) : (d5 - d3);
            else
                obst_delta = (d4 > d6) ? (d4 - d6) : (d6 - d4);
        } while (obst_delta > 50u && !g_robot_stop);
        if (g_robot_stop) { return; }

        if (distcm[3] <= 5u || (distcm[5] <= 5u && obst_count >= 1))
        {
            obst_count = 3;
            break;
        }

        if (obst_count >= 1)
        {
            ping();
            while (distcm[3] > 6u && !g_robot_stop)
            {
                motor_cmd(42u, 168u);
                delay_ms(30u);
                motor_cmd(64u, 192u);
                ping();
                if (distcm[3] <= 6u)
                {
                    delay_ms(1000u);
                    delay_ms(1000u);
                    return;
                }
            }
            if (g_robot_stop) { return; }
        }

        if (distcm[3] <= 5u || (distcm[5] <= 5u && obst_count >= 1))
        {
            obst_count = 3;
            break;
        }

        motor_cmd(42u, 175u);
        delay_ms(1100u);
        motor_cmd(64u, 192u);
        if (g_robot_stop) { return; }

        squareup();
        obst_count++;
    }

    delay_ms(1000u);
}

/* =========================================================================
 * BLINK_CAMERA
 * ====================================================================== */
static void blink_camera(int count)
{
    int i;
    for (i = 0; i < count && !g_robot_stop; i++)
    {
        PORTX->GROUP[0].PORT_OUTSET = (1u << PIN_BLINK);
        delay_ms(2000u);
        PORTX->GROUP[0].PORT_OUTCLR = (1u << PIN_BLINK);
        delay_ms(2000u);
    }
    PORTX->GROUP[0].PORT_OUTCLR = (1u << PIN_BLINK);  /* ensure pin is low if interrupted */
}

/* =========================================================================
 * DISP_GEN
 * ====================================================================== */
static void disp_gen(void)
{
#if DISPLAY_BOARD_ENABLED
    static const uint8_t room_pins[4][2] = {{0,24},{0,25},{0,27},{0,28}};
    static const uint8_t xpos_pins[4][2] = {{0,20},{0,21},{0,30},{0,31}};
    static const uint8_t ypos_pins[4][2] = {{1, 0},{1, 1},{1, 2},{1, 3}};
    static const uint8_t cond_pins[4][2] = {{1, 4},{1, 5},{1, 6},{1, 7}};

    uint8_t room_lcd  = (uint8_t)(rand() % 4 + 1);
    uint8_t x_pos     = (uint8_t)(rand() % 10);
    uint8_t y_pos     = (uint8_t)(rand() % 10);
    uint8_t condition = (uint8_t)(rand() % 3);
    uint8_t b, pin, grp;

    #define WRITE_BITS(pins_arr, val) \
        for (b = 0u; b < 4u; b++) { \
            grp = (pins_arr)[b][0]; pin = (pins_arr)[b][1]; \
            if ((val) & (1u << b)) \
                PORTX->GROUP[grp].PORT_OUTSET = (1u << pin); \
            else \
                PORTX->GROUP[grp].PORT_OUTCLR = (1u << pin); \
        }

    WRITE_BITS(room_pins, room_lcd);
    WRITE_BITS(xpos_pins, x_pos);
    WRITE_BITS(ypos_pins, y_pos);
    WRITE_BITS(cond_pins, condition);

    #undef WRITE_BITS
#endif
}

/* =========================================================================
 * ROBOT_INIT
 * ====================================================================== */
static void robot_init(void)
{
    PORTX->GROUP[0].PORT_DIRSET = (1u << PIN_BLINK);
    PORTX->GROUP[0].PORT_OUTCLR = (1u << PIN_BLINK);

    srand((unsigned int)g_tick_100us);

    disp_gen();
    delay_ms(500u);

    room             = 0;
    room_corner      = 0;
    intersection     = 0;
    room_one_disable = 0;
    room_two_disable = 0;
    room_3_disable   = 0;
    corner_obst      = 0;
    obst_count       = 0;
    stepcount        = 0;
    walldist         = 600;
    obst_delay       = 500;
}

/* =========================================================================
 * ROBOT_LOOP — exits when g_robot_stop is set
 * ====================================================================== */
static void robot_loop(void)
{
    uint32_t d3, d5, d4, d6, delta;

    for (;;)
    {
        if (g_robot_stop) { break; }

        stepcount   = 0;
        corner_obst = 0;
        ping();
        if (g_robot_stop) { break; }

        /* ── BLOCK A: Room 2 corner-based exit ──────────────────────────── */
        if (room == 2 && room_corner >= 2)
        {
            if (distcm[7] >= 110u)
            {
                room_two_disable = 1;
                intersection     = 0;
                room_corner      = 0;
                motor_cmd(64u, 192u);
                blink_camera(2);
                if (g_robot_stop) { break; }
            }
        }

        /* ── BLOCK B: Room 2 → Room 3 ───────────────────────────────────── */
        if (room == 2 && room_two_disable == 1
            && distcm[7] >= 120u && room_3_disable != 1)
        {
            room = 3;
            motor_cmd(64u, 192u);
            blink_camera(3);
            if (g_robot_stop) { break; }
        }

        /* ── BLOCK C: Side wall ≤ 7 cm — wall turn ─────────────────────── */
        if (distcm[3] <= 7u || distcm[5] <= 7u)
        {
            motor_cmd(64u, 192u);
            delay_ms(500u);
            if (g_robot_stop) { break; }
            disp_gen();
            ping();

            if (room == 1 && distcm[7] >= 80u)
            {
                room_one_disable = 1;
                intersection     = 0;
                room_corner      = 0;
                blink_camera(1);
                if (g_robot_stop) { break; }
            }

            if (room == 3 && distcm[7] >= 150u && room_3_disable == 1)
            {
                blink_camera(4);
                if (g_robot_stop) { break; }
            }

            motor_cmd(96u, 160u);
            delay_ms(500u);
            motor_cmd(64u, 192u);
            if (g_robot_stop) { break; }

            ping();
            if (distcm[4] >= 20u)
            {
                obstacle();
                if (g_robot_stop) { break; }
            }

            if (room >= 1 && obst_count == 0)
            {
                room_corner++;
            }

            obst_count = 0;
        }

        /* ── BLOCK D: No immediate side walls ───────────────────────────── */
        else
        {
            /* ── D1: All clear — open drive + room detection ─────────────── */
            if (distcm[4] >= 15u && distcm[3] >= 15u && distcm[2] >= 15u)
            {
                while (1)   /* DOAGAIN */
                {
                    if (g_robot_stop) { break; }

                    uint32_t walldelta_u = 0u;
                    motor_cmd(64u, 192u);
                    delay_ms(1000u);
                    if (g_robot_stop) { break; }

                    ping();
                    while (distcm[6] <= 15u && !g_robot_stop)
                    {
                        motor_cmd(42u, 168u);
                        delay_ms(30u);
                        motor_cmd(64u, 192u);
                        stepcount++;
                        ping();
                    }
                    if (g_robot_stop) { break; }

                    if (stepcount > 15)
                    {
                        walldist   = 1200;
                        obst_delay = 800;
                        motor_cmd(42u, 168u);
                        delay_ms(180u);
                    }
                    else
                    {
                        motor_cmd(42u, 168u);
                        delay_ms(220u);
                    }
                    if (g_robot_stop) { break; }

                    if (corner_obst >= 1)
                    {
                        motor_cmd(42u, 168u);
                        delay_ms(170u);
                        if (g_robot_stop) { break; }
                    }

                    motor_cmd(64u, 192u);
                    delay_ms(500u);
                    if (g_robot_stop) { break; }
                    ping();

                    /* Room 1 entry */
                    if (distcm[2] <= 30u && intersection >= 1
                        && corner_obst == 0 && room_one_disable != 1)
                    {
                        room             = 1;
                        room_one_disable = 1;
                        intersection     = 0;
                        room_corner      = 0;
                        blink_camera(1);
                        if (g_robot_stop) { break; }
                    }

                    /* Room 2 entry */
                    if (distcm[7] >= 140u && intersection >= 1 && room == 1)
                    {
                        room         = 2;
                        intersection = 0;
                        room_corner  = 0;
                        blink_camera(2);
                        if (g_robot_stop) { break; }
                    }

                    /* Room 3 exit */
                    if (room == 3 && distcm[7] >= 10u && distcm[4] >= 15u)
                    {
                        room_3_disable = 1;
                        intersection   = 0;
                        room_corner    = 0;
                        blink_camera(3);
                        if (g_robot_stop) { break; }
                    }

                    motor_cmd(42u, 210u);
                    delay_ms(500u);
                    motor_cmd(64u, 192u);
                    if (g_robot_stop) { break; }

                    /* Parallel-align loop */
                    do
                    {
                        motor_cmd(42u, 210u);
                        delay_ms(60u);
                        motor_cmd(64u, 192u);
                        ping();
                        d3 = durat[3]; d5 = durat[5];
                        walldelta_u = (d3 > d5) ? (d3 - d5) : (d5 - d3);
                    } while (walldelta_u >= 80u && !g_robot_stop);
                    if (g_robot_stop) { break; }

                    /* corner_obst path */
                    if (corner_obst == 1)
                    {
                        ping();
                        while (distcm[3] > 7u && !g_robot_stop)
                        {
                            motor_cmd(42u, 168u);
                            delay_ms(30u);
                            motor_cmd(64u, 192u);
                            ping();
                        }
                        break;
                    }

                    ping();
                    while (distcm[4] > 15u && !g_robot_stop)
                    {
                        motor_cmd(42u, 168u);
                        delay_ms(30u);
                        motor_cmd(64u, 192u);
                        ping();
                    }
                    if (g_robot_stop) { break; }

                    motor_cmd(42u, 175u);
                    delay_ms((uint32_t)obst_delay);
                    if (g_robot_stop) { break; }
                    intersection++;

                    if (stepcount > 15)
                    {
                        corner_obst = 1;
                        continue;
                    }

                    if (room == 1 && distcm[6] <= 15u)
                    {
                        ping();
                        while (distcm[6] > 15u && !g_robot_stop)
                        {
                            motor_cmd(42u, 168u);
                            delay_ms(30u);
                            motor_cmd(64u, 192u);
                            ping();
                        }
                        if (g_robot_stop) { break; }
                    }

                    squareup();
                    break;
                } /* end DOAGAIN */

                if (g_robot_stop) { break; }
                delay_ms(50u);
            }

            /* ── D2: Active wall correction ─────────────────────────────── */
            else
            {
                d4 = durat[4];
                d6 = durat[6];

                if (d4 < 350u)
                {
                    motor_cmd(50u, 140u);
                    delay_ms(45u);
                    motor_cmd(64u, 192u);
                }
                else if (d4 > 700u)
                {
                    motor_cmd(21u, 178u);
                    delay_ms(45u);
                    motor_cmd(64u, 192u);
                }

                if (g_robot_stop) { break; }

                delta = (d4 > d6) ? (d4 - d6) : (d6 - d4);

                if (delta <= 30u)
                {
                    motor_cmd(47u, 175u);
                }
                else if (d4 < d6)
                {
                    motor_cmd(47u, 148u);
                    delay_ms(10u);
                    motor_cmd(47u, 175u);
                }
                else
                {
                    motor_cmd(20u, 175u);
                    delay_ms(10u);
                    motor_cmd(47u, 175u);
                }
            }
        }
    } /* end for(;;) */

    platform_motor_stop();
}

/* =========================================================================
 * PUBLIC ENTRY POINT
 * ====================================================================== */
void robot_run(void)
{
    g_robot_stop = 0u;
    robot_init();
    if (!g_robot_stop)
    {
        robot_loop();
    }
    platform_motor_stop();
    platform_usart_write_str("NAV: stopped — controls restored\r\n");
}
