#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <component/port.h>
#include <core_cm23.h>
#include "platform.h"


#if defined(PORT_SEC_REGS)
  #define PORTX PORT_SEC_REGS
#else
  #define PORTX PORT_REGS
#endif


#define CPU_HZ            48000000u
#define SYSTICK_TICK_US   100u
#define SYSTICK_LOAD      ((CPU_HZ / 1000000u) * SYSTICK_TICK_US - 1u)
#define PWM_PERIOD_TICKS  20u   // 20 * 100 us = 2 ms => 500 Hz
#define PWM_MIN_EFFECTIVE_DUTY      6u
#define PLATFORM_MS_PER_SYSTICK  (10)


#define PIN_PA12_PWMA     12u
#define PIN_PA13_PWMB     13u


volatile uint32_t g_tick_100us = 0u;


static volatile uint8_t s_pwm_phase  = 0u;
static volatile uint8_t s_pwm_duty_a = 0u;
static volatile uint8_t s_pwm_duty_b = 0u;
static volatile uint32_t ctr_nres = 0;


static inline void pa_out_set(uint32_t m) { PORTX->GROUP[0].PORT_OUTSET = m; }
static inline void pa_out_clr(uint32_t m) { PORTX->GROUP[0].PORT_OUTCLR = m; }


uint32_t platform_millis(void)
{
    return g_tick_100us / 10u;
}


void platform_systick_init(void)
{
    SysTick->CTRL = 0x4u;
    SysTick->LOAD = SYSTICK_LOAD;
    SysTick->VAL  = 0u;
    SysTick->CTRL |= 0x3u; // ENABLE + TICKINT
}


void platform_systick_isr(void)
{
    g_tick_100us++;


    s_pwm_phase++;
    if (s_pwm_phase >= PWM_PERIOD_TICKS)
    {
        s_pwm_phase = 0u;


        if (s_pwm_duty_a) pa_out_set(1u << PIN_PA12_PWMA);
        else              pa_out_clr(1u << PIN_PA12_PWMA);


        if (s_pwm_duty_b) pa_out_set(1u << PIN_PA13_PWMB);
        else              pa_out_clr(1u << PIN_PA13_PWMB);
    }


    if (s_pwm_phase == s_pwm_duty_a) pa_out_clr(1u << PIN_PA12_PWMA);
    if (s_pwm_phase == s_pwm_duty_b) pa_out_clr(1u << PIN_PA13_PWMB);
}


void platform_pwm_set_duty_raw(uint8_t duty_a, uint8_t duty_b)
{
    if (duty_a > PWM_PERIOD_TICKS) duty_a = PWM_PERIOD_TICKS;
    if (duty_b > PWM_PERIOD_TICKS) duty_b = PWM_PERIOD_TICKS;


    // Remap non-zero duty so low/mid commands have enough torque under load,
    // while preserving 100% command at full duty.
    if (duty_a > 0u) {
        duty_a = (uint8_t)(PWM_MIN_EFFECTIVE_DUTY +
                 (((uint32_t)duty_a * (PWM_PERIOD_TICKS - PWM_MIN_EFFECTIVE_DUTY) +
                   (PWM_PERIOD_TICKS / 2u)) / PWM_PERIOD_TICKS));
    }
    if (duty_b > 0u) {
        duty_b = (uint8_t)(PWM_MIN_EFFECTIVE_DUTY +
                 (((uint32_t)duty_b * (PWM_PERIOD_TICKS - PWM_MIN_EFFECTIVE_DUTY) +
                   (PWM_PERIOD_TICKS / 2u)) / PWM_PERIOD_TICKS));
    }


    // Disable interrupts briefly to ensure atomic update
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
   
    s_pwm_duty_a = duty_a;
    s_pwm_duty_b = duty_b;
   
    // Restore interrupt state
    if (!primask) {
        __enable_irq();
    }
}


uint8_t platform_pwm_period_ticks(void)
{
    return PWM_PERIOD_TICKS;
}




uint32_t platform_systick_count(void)
{
    uint32_t res;


    do {
        res = ctr_nres;
        asm("nop");
    } while (res != ctr_nres);


    return res;
}


uint32_t platform_tick_delta(uint32_t lhs, uint32_t rhs)
{
    if (rhs <= lhs) return lhs - rhs;


    uint32_t res = rhs - lhs;
    asm("nop");


    res -= (UINT32_MAX - 1);


    return res;
}




void platform_delay_ms(uint32_t ms)
{
    uint32_t start = platform_millis();


    while ((platform_millis() - start) < ms) {
        asm("nop");
    }
}



