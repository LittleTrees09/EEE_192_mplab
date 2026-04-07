#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <component/port.h>
#include "platform.h"

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
#define PIN_PA23_BUTTON   23u

#define PIN_PB02_BIN2     2u

#define IR_MASK_S1        (1u << 0)
#define IR_MASK_S2        (1u << 1)
#define IR_MASK_S3        (1u << 2)
#define IR_MASK_S4        (1u << 3)
#define IR_MASK_S5        (1u << 4)

#define PWM_PERIOD_TICKS  20u

extern void platform_pwm_set_duty_raw(uint8_t duty_a, uint8_t duty_b);

static inline void pa_out_set(uint32_t m) { PORTX->GROUP[0].PORT_OUTSET = m; }
static inline void pa_out_clr(uint32_t m) { PORTX->GROUP[0].PORT_OUTCLR = m; }
static inline uint32_t pa_in(void)        { return PORTX->GROUP[0].PORT_IN; }
static inline void pb_out_set(uint32_t m) { PORTX->GROUP[1].PORT_OUTSET = m; }
static inline void pb_out_clr(uint32_t m) { PORTX->GROUP[1].PORT_OUTCLR = m; }

void platform_gpio_init(void)
{
    // Button PA23 input + pull-up
    PORTX->GROUP[0].PORT_DIRCLR = (1u << PIN_PA23_BUTTON);
    PORTX->GROUP[0].PORT_PINCFG[PIN_PA23_BUTTON] = (1u << 1) | (1u << 2); // INEN | PULLEN
    pa_out_set(1u << PIN_PA23_BUTTON);

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
    // Already configured in platform_gpio_init(). Kept separate on purpose.
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

    // coast stop instead of active brake
    pa_out_clr((1u << PIN_PA06_AIN1) | (1u << PIN_PA03_AIN2) | (1u << PIN_PA02_BIN1));
    pb_out_clr(1u << PIN_PB02_BIN2);
    pa_out_clr((1u << PIN_PA12_PWMA) | (1u << PIN_PA13_PWMB));
}

void platform_motor_set(int16_t left, int16_t right)
{
    uint8_t duty_a = 0u;
    uint8_t duty_b = 0u;

    // LEFT side = channel A
    if (left == 0)
    {
        // coast
        pa_out_clr(1u << PIN_PA06_AIN1);
        pa_out_clr(1u << PIN_PA03_AIN2);
        duty_a = 0u;
    }
    else
    {
        if (left > 1000)  left = 1000;
        if (left < -1000) left = -1000;

        if (left > 0)
        {
            // forward
            pa_out_set(1u << PIN_PA06_AIN1);
            pa_out_clr(1u << PIN_PA03_AIN2);
        }
        else
        {
            // reverse
            pa_out_clr(1u << PIN_PA06_AIN1);
            pa_out_set(1u << PIN_PA03_AIN2);
            left = (int16_t)(-left);
        }

        duty_a = (uint8_t)((uint32_t)left * PWM_PERIOD_TICKS / 1000u);
    }

    // RIGHT side = channel B
    if (right == 0)
    {
        // coast
        pa_out_clr(1u << PIN_PA02_BIN1);
        pb_out_clr(1u << PIN_PB02_BIN2);
        duty_b = 0u;
    }
    else
    {
        if (right > 1000)  right = 1000;
        if (right < -1000) right = -1000;

        if (right > 0)
        {
            // forward
            pa_out_set(1u << PIN_PA02_BIN1);
            pb_out_clr(1u << PIN_PB02_BIN2);
        }
        else
        {
            // reverse
            pa_out_clr(1u << PIN_PA02_BIN1);
            pb_out_set(1u << PIN_PB02_BIN2);
            right = (int16_t)(-right);
        }

        duty_b = (uint8_t)((uint32_t)right * PWM_PERIOD_TICKS / 1000u);
    }

    platform_pwm_set_duty_raw(duty_a, duty_b);
}
