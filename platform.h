#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_tick_100us;

// ----- top-level init / timebase -----
void     platform_initialization(void);
uint32_t platform_millis(void);
void     platform_systick_init(void);
void     platform_systick_isr(void);

// ----- GPIO / basic board IO -----
void     platform_gpio_init(void);
bool     platform_button_pressed(void);      // PA23, pressed = LOW
void     platform_led_set(bool on);          // PA15

// ----- TB6612 motor driver -----
void     platform_tb6612_enable(bool en);    // PA07 STBY
void     platform_motor_set(int16_t left, int16_t right); // -1000..+1000
void     platform_motor_stop(void);

// ----- IR line array (digital inputs) -----
void     platform_ir_init(void);
uint8_t  platform_ir_read_mask_raw(void);    // bit0=S1 ... bit4=S5

// ----- USART over SERCOM3 on PB08/PB09 -----
void     platform_usart_init(void);
bool     platform_usart_read_char(char *out);     // non-blocking
void     platform_usart_write_char(char c);
void     platform_usart_write_buf(const char *s, uint32_t n);
void     platform_usart_write_str(const char *s);

#ifdef __cplusplus
}
#endif

#endif
