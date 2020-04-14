#ifndef _CONFIG_H
#define _CONFIG_H

#include <Arduino.h>
// Select appropriate display type

#define DISPLAY_0802
//#define DISPLAY_1602_I2C

#ifdef DISPLAY_1602_I2C
#include "dspl_1602_i2c.h"
#endif

#ifdef DISPLAY_0802
#include "dspl_0802.h"
#endif

/*
 * Maximum supported custom tips for controller, see iron_tips.h and cfg.h
 * Each custom tip data requires 12 bytes in EEPROM, 504 bytes per 42 tips are required.
 * 
 */
#define MAX_CUSTOM_TIPS (42)

#endif
