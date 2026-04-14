#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DRIVE_MODE_MANUAL = 0,
    DRIVE_MODE_AUTO_IR = 1,
    DRIVE_MODE_AUTO_ULTRASONIC = 2
} drive_mode_t;

#ifdef __cplusplus
}
#endif

#endif