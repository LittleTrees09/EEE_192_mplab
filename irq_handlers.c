#include <xc.h>
#include "platform.h"

void __attribute__((interrupt)) SysTick_Handler(void)
{
    platform_systick_isr();
}
