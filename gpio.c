#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <component/port.h>
#include "platform.h"

#include <stddef.h>

#if defined(PORT_SEC_REGS)
  #define PORTX PORT_SEC_REGS
#else
  #define PORTX PORT_REGS
#endif

#define LED_ACTIVE_LOW    0

#define PIN_PA02_BIN1     2u
#define PIN_PA03_AIN2     3u
#define PIN_PA06_AIN1     6u
#define PIN_PA07_STBY     7u
#define PIN_PA08_IR_S1    8u
#define PIN_PA09_IR_S2    9u
#define PIN_PA10_IR_S3    10u
#define PIN_PA11_IR_S4    11u
#define PIN_PA12_PWMA     12u
#define PIN_PA13_PWMB     13u
#define PIN_PA14_IR_S5    14u
#define PIN_PA15_LED      15u
#define PIN_PA22_BT_STATE 22u
#define PIN_PA23_BUTTON   23u

#define PIN_PB02_BIN2     2u

#define IR_MASK_S1        (1u << 0)
#define IR_MASK_S2        (1u << 1)
#define IR_MASK_S3        (1u << 2)
#define IR_MASK_S4        (1u << 3)
#define IR_MASK_S5        (1u << 4)

#define PIN_PA16_US_TRIG_FRONT   16u
#define PIN_PA17_US_TRIG_LEFT    17u
#define PIN_PA18_US_TRIG_RIGHT   18u
#define PIN_PA19_US_ECHO_FRONT   19u
#define PIN_PA20_US_ECHO_LEFT    20u
#define PIN_PA21_US_ECHO_RIGHT   21u

#define PWM_PERIOD_TICKS  20u

#define MOTOR_CHANNELS_SWAPPED 1

extern void platform_pwm_set_duty_raw(uint8_t duty_a, uint8_t duty_b);

static inline void pa_out_set(uint32_t m) { PORTX->GROUP[0].PORT_OUTSET = m; }
static inline void pa_out_clr(uint32_t m) { PORTX->GROUP[0].PORT_OUTCLR = m; }
static inline uint32_t pa_in(void)        { return PORTX->GROUP[0].PORT_IN; }
static inline void pb_out_set(uint32_t m) { PORTX->GROUP[1].PORT_OUTSET = m; }
static inline void pb_out_clr(uint32_t m) { PORTX->GROUP[1].PORT_OUTCLR = m; }

extern volatile uint32_t g_tick_100us;

static inline void wait_100us_ticks(uint32_t ticks)
{
    uint32_t start = g_tick_100us;
    while ((uint32_t)(g_tick_100us - start) < ticks) {
        asm("nop");
    }
}

static bool wait_echo_state(uint32_t mask, bool want_high, uint32_t timeout_ticks)
{
    uint32_t start = g_tick_100us;

    while (((pa_in() & mask) != 0u) != want_high) {
        if ((uint32_t)(g_tick_100us - start) >= timeout_ticks) {
            return false;
        }
    }

    return true;
}

static bool ultrasonic_get_masks(ultrasonic_id_t sensor, uint32_t *trig_mask, uint32_t *echo_mask)
{
    if ((trig_mask == NULL) || (echo_mask == NULL)) {
        return false;
    }

    switch (sensor)
    {
        case ULTRA_FRONT:
            *trig_mask = (1u << PIN_PA16_US_TRIG_FRONT);
            *echo_mask = (1u << PIN_PA19_US_ECHO_FRONT);
            return true;

        case ULTRA_LEFT:
            *trig_mask = (1u << PIN_PA17_US_TRIG_LEFT);
            *echo_mask = (1u << PIN_PA20_US_ECHO_LEFT);
            return true;

        case ULTRA_RIGHT:
            *trig_mask = (1u << PIN_PA18_US_TRIG_RIGHT);
            *echo_mask = (1u << PIN_PA21_US_ECHO_RIGHT);
            return true;

        default:
            return false;
    }
}

void platform_gpio_init(void)
{
    // Button PA23 input + pull-up
    PORTX->GROUP[0].PORT_DIRCLR = (1u << PIN_PA23_BUTTON);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA23_BUTTON] = (1u << 1) | (1u << 2);
    pa_out_set(1u << PIN_PA23_BUTTON);

    // HC-05 STATE input (PA22) — HIGH = connected
    PORTX->GROUP[0].PORT_DIRCLR = (1u << PIN_PA22_BT_STATE);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA22_BT_STATE] = (1u << 1); // INEN only

    // IR sensor inputs PA08, PA09, PA10, PA11, PA14
    PORTX->GROUP[0].PORT_DIRCLR =
        (1u << PIN_PA08_IR_S1) |
        (1u << PIN_PA09_IR_S2) |
        (1u << PIN_PA10_IR_S3) |
        (1u << PIN_PA11_IR_S4) |
        (1u << PIN_PA14_IR_S5);

    PORTX->GROUP[0].PORT_PINCFG[PIN_PA08_IR_S1] = (1u << 1) | (1u << 2);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA09_IR_S2] = (1u << 1) | (1u << 2);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA10_IR_S3] = (1u << 1) | (1u << 2);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA11_IR_S4] = (1u << 1) | (1u << 2);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA14_IR_S5] = (1u << 1) | (1u << 2);

    pa_out_set((1u << PIN_PA08_IR_S1) |
               (1u << PIN_PA09_IR_S2) |
               (1u << PIN_PA10_IR_S3) |
               (1u << PIN_PA11_IR_S4) |
               (1u << PIN_PA14_IR_S5));

    // Ultrasonic TRIG outputs: PA16, PA17, PA18
    PORTX->GROUP[0].PORT_DIRSET =
        (1u << PIN_PA16_US_TRIG_FRONT) |
        (1u << PIN_PA17_US_TRIG_LEFT)  |
        (1u << PIN_PA18_US_TRIG_RIGHT);

    pa_out_clr(
        (1u << PIN_PA16_US_TRIG_FRONT) |
        (1u << PIN_PA17_US_TRIG_LEFT)  |
        (1u << PIN_PA18_US_TRIG_RIGHT)
    );

    // Ultrasonic ECHO inputs: PA19, PA20, PA21
    PORTX->GROUP[0].PORT_DIRCLR =
        (1u << PIN_PA19_US_ECHO_FRONT) |
        (1u << PIN_PA20_US_ECHO_LEFT)  |
        (1u << PIN_PA21_US_ECHO_RIGHT);

    PORTX->GROUP[0].PORT_PINCFG[PIN_PA19_US_ECHO_FRONT] = (1u << 1);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA20_US_ECHO_LEFT]  = (1u << 1);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA21_US_ECHO_RIGHT] = (1u << 1);

    // Outputs on PORTA
    PORTX->GROUP[0].PORT_DIRSET =
        (1u << PIN_PA15_LED)  |
        (1u << PIN_PA07_STBY) |
        (1u << PIN_PA06_AIN1) |
        (1u << PIN_PA03_AIN2) |
        (1u << PIN_PA02_BIN1) |
        (1u << PIN_PA12_PWMA) |
        (1u << PIN_PA13_PWMB);

    // BIN2 on PORTB
    PORTX->GROUP[1].PORT_DIRSET = (1u << PIN_PB02_BIN2);

    pa_out_clr((1u << PIN_PA07_STBY) |
               (1u << PIN_PA06_AIN1) | (1u << PIN_PA03_AIN2) |
               (1u << PIN_PA02_BIN1) |
               (1u << PIN_PA12_PWMA) | (1u << PIN_PA13_PWMB));
    pb_out_clr(1u << PIN_PB02_BIN2);

#if LED_ACTIVE_LOW
    pa_out_set(1u << PIN_PA15_LED);
#else
    pa_out_clr(1u << PIN_PA15_LED);
#endif
}

void platform_ir_init(void)
{
    // Already configured in platform_gpio_init().
}

uint8_t platform_ir_read_mask_raw(void)
{
    uint32_t v = pa_in();
    uint8_t mask = 0u;

    if (v & (1u << PIN_PA08_IR_S1)) mask |= IR_MASK_S1;
    if (v & (1u << PIN_PA09_IR_S2)) mask |= IR_MASK_S2;
    if (v & (1u << PIN_PA10_IR_S3)) mask |= IR_MASK_S3;
    if (v & (1u << PIN_PA11_IR_S4)) mask |= IR_MASK_S4;
    if (v & (1u << PIN_PA14_IR_S5)) mask |= IR_MASK_S5;

    return mask;
}

bool platform_button_pressed(void)
{
    return ((pa_in() & (1u << PIN_PA23_BUTTON)) == 0u);
}

bool platform_bt_connected(void)
{
    return ((pa_in() & (1u << PIN_PA22_BT_STATE)) != 0u);
}

void platform_led_set(bool on)
{
#if LED_ACTIVE_LOW
    if (on) pa_out_clr(1u << PIN_PA15_LED);
    else    pa_out_set(1u << PIN_PA15_LED);
#else
    if (on) pa_out_set(1u << PIN_PA15_LED);
    else    pa_out_clr(1u << PIN_PA15_LED);
#endif
}

void platform_tb6612_enable(bool en)
{
    if (en) pa_out_set(1u << PIN_PA07_STBY);
    else    pa_out_clr(1u << PIN_PA07_STBY);
}

void platform_motor_stop(void)
{
    platform_pwm_set_duty_raw(0u, 0u);

    pa_out_clr((1u << PIN_PA06_AIN1) | (1u << PIN_PA03_AIN2) | (1u << PIN_PA02_BIN1));
    pb_out_clr(1u << PIN_PB02_BIN2);
    pa_out_clr((1u << PIN_PA12_PWMA) | (1u << PIN_PA13_PWMB));
}

void platform_motor_set(int16_t left, int16_t right)
{
    uint8_t duty_a = 0u;
    uint8_t duty_b = 0u;
#if MOTOR_CHANNELS_SWAPPED
    int16_t logical_left = right;
    int16_t logical_right = left;
#else
    int16_t logical_left = left;
    int16_t logical_right = right;
#endif

    if (logical_left == 0)
    {
        pa_out_clr(1u << PIN_PA06_AIN1);
        pa_out_clr(1u << PIN_PA03_AIN2);
        duty_a = 0u;
    }
    else
    {
        if (logical_left > 1000)  logical_left = 1000;
        if (logical_left < -1000) logical_left = -1000;

        if (logical_left > 0)
        {
            pa_out_clr(1u << PIN_PA03_AIN2);
            pa_out_set(1u << PIN_PA06_AIN1);
        }
        else
        {
            pa_out_clr(1u << PIN_PA06_AIN1);
            pa_out_set(1u << PIN_PA03_AIN2);
            logical_left = (int16_t)(-logical_left);
        }

        duty_a = (uint8_t)((uint32_t)logical_left * PWM_PERIOD_TICKS / 1000u);
    }

    if (logical_right == 0)
    {
        pa_out_clr(1u << PIN_PA02_BIN1);
        pb_out_clr(1u << PIN_PB02_BIN2);
        duty_b = 0u;
    }
    else
    {
        if (logical_right > 1000)  logical_right = 1000;
        if (logical_right < -1000) logical_right = -1000;

        if (logical_right > 0)
        {
            pb_out_clr(1u << PIN_PB02_BIN2);
            pa_out_set(1u << PIN_PA02_BIN1);
        }
        else
        {
            pa_out_clr(1u << PIN_PA02_BIN1);
            pb_out_set(1u << PIN_PB02_BIN2);
            logical_right = (int16_t)(-logical_right);
        }

        duty_b = (uint8_t)((uint32_t)logical_right * PWM_PERIOD_TICKS / 1000u);
    }

    platform_pwm_set_duty_raw(duty_a, duty_b);
}

void platform_ultrasonic_init(void)
{
    // Pins already configured in platform_gpio_init().
}

bool platform_ultrasonic_read_cm(ultrasonic_id_t sensor, uint16_t *distance_cm)
{
    uint32_t trig_mask;
    uint32_t echo_mask;
    uint32_t pulse_start;
    uint32_t pulse_ticks;
    uint32_t pulse_end;

    const uint32_t timeout_ticks = 300u;

    if ((distance_cm == NULL) || !ultrasonic_get_masks(sensor, &trig_mask, &echo_mask)) {
        return false;
    }

    pa_out_clr(trig_mask);
    wait_100us_ticks(1u);

    pa_out_set(trig_mask);
    wait_100us_ticks(1u);
    pa_out_clr(trig_mask);

    if (!wait_echo_state(echo_mask, true, timeout_ticks)) {
        return false;
    }

    pulse_start = g_tick_100us;

    if (!wait_echo_state(echo_mask, false, timeout_ticks)) {
        return false;
    }

    pulse_end = g_tick_100us;
    pulse_ticks = (uint32_t)(pulse_end - pulse_start);

    *distance_cm = (uint16_t)((pulse_ticks * 1715u + 500u) / 1000u);

    return true;
}