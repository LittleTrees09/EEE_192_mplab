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
uint32_t platform_systick_count(void);
uint32_t platform_tick_delta(uint32_t lhs, uint32_t rhs);
void     platform_delay_ms(uint32_t ms);




// ----- GPIO / basic board IO -----
void     platform_gpio_init(void);
bool     platform_button_pressed(void);
void     platform_led_set(bool on);


// ----- TB6612 motor driver -----
void     platform_tb6612_enable(bool en);
void     platform_motor_set(int16_t left, int16_t right);
void     platform_motor_stop(void);


// ----- IR line array (digital inputs) -----
void     platform_ir_init(void);
uint8_t  platform_ir_read_mask_raw(void);


// ----- HC-SR04 ultrasonic sensors -----
typedef enum
{
    ULTRA_FRONT = 0,
    ULTRA_LEFT  = 1,
    ULTRA_RIGHT = 2
} ultrasonic_id_t;


void platform_ultrasonic_init(void);
bool platform_ultrasonic_read_cm(ultrasonic_id_t sensor, uint16_t *distance_cm);


// ----- USART over SERCOM3 on PB08/PB09 -----
void     platform_usart_init(void);
bool     platform_usart_read_char(char *out);
void     platform_usart_write_char(char c);
void     platform_usart_write_buf(const char *s, uint32_t n);
void     platform_usart_write_str(const char *s);


// ----- HC-05 Bluetooth -----
bool     platform_bt_connected(void);   // PA22, HIGH = paired & connected


#ifdef __cplusplus
}
#endif


#endif

