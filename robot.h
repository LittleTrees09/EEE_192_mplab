#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Sensor data arrays — indices 2–7 per the alias table in the integration guide.
 *   Front : distcm[4], distcm[7], durat[4], durat[7]
 *   Left  : distcm[2], distcm[3], durat[2], durat[3]
 *   Right : distcm[5], distcm[6], durat[5], durat[6]
 * durat[] values are in microseconds (100 µs resolution via g_tick_100us).
 */
extern uint32_t durat[8];
extern uint32_t distcm[8];
extern uint32_t distin[8];

/*
 * Camera/indicator blink pin — adjust to match your PCB.
 * Must be a PORTA pin; set PIN_BLINK to the bit number (0–31).
 */
#define PIN_BLINK  28u   /* PA28 — change if wired differently */

/*
 * Display board enable — set to 1 if the GPIO display board is connected.
 * When 0, disp_gen() compiles as a no-op so the nav algorithm still runs.
 */
#define DISPLAY_BOARD_ENABLED  0

/* Entry points */
void robot_run(void);   /* calls robot_init() then robot_loop() — never returns */

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
