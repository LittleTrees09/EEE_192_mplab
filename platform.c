#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

static void raise_perf(void)
{
    uint32_t tmp_reg;

    PM_REGS->PM_INTFLAG = 0x01;
    PM_REGS->PM_PLCFG  = 0x02;
    while ((PM_REGS->PM_INTFLAG & 0x01u) == 0u) { asm("nop"); }
    PM_REGS->PM_INTFLAG = 0x01;

    NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2u << 1);

    SUPC_REGS->SUPC_VREGPLL = 0x00000202;
    while ((SUPC_REGS->SUPC_STATUS & (1u << 18)) == 0u) { asm("nop"); }

    OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0000;
    while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1u << 24)) == 0u) { asm("nop"); }

    tmp_reg  = (*(uint32_t*)0x00806020u) & 0x000000FFu;
    tmp_reg |= (*(uint32_t*)0x00806024u) & 0x0000FF00u;
    OSCCTRL_REGS->OSCCTRL_DFLLVAL = tmp_reg;

    OSCCTRL_REGS->OSCCTRL_DFLLMUL  = 0x0000BB80;
    OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0002;
    while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1u << 24)) == 0u) { asm("nop"); }

    GCLK_REGS->GCLK_GENCTRL[2] = 0x00010105;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1u << 4)) != 0u) { asm("nop"); }

    GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1u << 2)) != 0u) { asm("nop"); }
}

void platform_initialization(void)
{
    raise_perf();
    platform_gpio_init();
    platform_ir_init();
    platform_systick_init();
    platform_usart_init();

    platform_motor_stop();
    platform_tb6612_enable(false);
    platform_led_set(false);
}
