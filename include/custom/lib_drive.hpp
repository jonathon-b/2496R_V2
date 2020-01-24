#include "config.hpp"
#include "../classes/PID.hpp"
#ifndef LIB_DRIVE_H
#define LIB_DRIVE_H

extern void drive_chassis();
extern void drive_lift();
extern void drive_intake();
extern void drive_tray();
extern double tilt_offset;
extern void pid_lift();

extern PID autolift;
extern double preset_heights[3];
extern double lift_value;
extern int index_heights;
#endif
