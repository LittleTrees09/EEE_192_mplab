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

#define SERCOM_CDC_REGS (&(SERCOM3_REGS->USART_INT))

void platform_usart_init(void)
{
    GCLK_REGS->GCLK_PCHCTRL[20] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[20] & 0x00000040u) == 0u) {
        asm("nop");
    }

    SERCOM_CDC_REGS->SERCOM_CTRLA = 0x00000001u;
    while ((SERCOM_CDC_REGS->SERCOM_SYNCBUSY & 0x00000001u) != 0u) {
        asm("nop");
    }

    SERCOM_CDC_REGS->SERCOM_CTRLA  = 0x00000004u;
    SERCOM_CDC_REGS->SERCOM_CTRLA |= 0x40000000u;
    SERCOM_CDC_REGS->SERCOM_CTRLA |= 0x00100000u;
    SERCOM_CDC_REGS->SERCOM_CTRLB  = 0x00000000u;
    SERCOM_CDC_REGS->SERCOM_CTRLC  = 0x08000000u;

    SERCOM_CDC_REGS->SERCOM_BAUD = 55470u; // 38400 baud with current GCLK setup

    SERCOM_CDC_REGS->SERCOM_CTRLB |= 0x00C30000u;
    while ((SERCOM_CDC_REGS->SERCOM_SYNCBUSY & 0x00000004u) != 0u) {
        asm("nop");
    }

    PORTX->GROUP[1].PORT_OUTCLR = (1u << 9);
    PORTX->GROUP[1].PORT_DIRCLR = (1u << 9);

    PORTX->GROUP[1].PORT_OUTSET = (1u << 8);
    PORTX->GROUP[1].PORT_DIRSET = (1u << 8);

    PORTX->GROUP[1].PORT_PMUX[(8u >> 1)] = 0x33u;
    PORTX->GROUP[1].PORT_PINCFG[9] = 0x03u; // PMUXEN | INEN
    PORTX->GROUP[1].PORT_PINCFG[8] = 0x41u; // PMUXEN | DRVSTR

    SERCOM_CDC_REGS->SERCOM_CTRLA |= 0x00000002u;
    while ((SERCOM_CDC_REGS->SERCOM_SYNCBUSY & 0x00000006u) != 0u) {
        asm("nop");
    }
}

bool platform_usart_read_char(char *out)
{
    volatile uint16_t sc;
    uint32_t dt;

    if (!out) return false;

    if ((SERCOM_CDC_REGS->SERCOM_INTFLAG & 0x04u) == 0u) {
        return false;
    }

    sc = SERCOM_CDC_REGS->SERCOM_STATUS;
    (void)sc;

    dt = SERCOM_CDC_REGS->SERCOM_DATA;
    *out = (char)(dt & 0xFFu);
    return true;
}

void platform_usart_write_char(char c)
{
    while ((SERCOM_CDC_REGS->SERCOM_INTFLAG & 0x01u) == 0u) {
        asm("nop");
    }
    SERCOM_CDC_REGS->SERCOM_DATA = (uint16_t)c;
}

void platform_usart_write_buf(const char *s, uint32_t n)
{
    uint32_t i;

    if (!s) return;

    for (i = 0u; i < n; i++) {
        platform_usart_write_char(s[i]);
    }
}

void platform_usart_write_str(const char *s)
{
    if (!s) return;
    while (*s) {
        platform_usart_write_char(*s++);
    }
}
