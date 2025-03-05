#ifndef RIBERRY_CONFIG_H
#define RIBERRY_CONFIG_H

#include <Arduino.h>

#ifdef RIBERRY_VERSION
constexpr const char VERSION[] = RIBERRY_VERSION;
#else
constexpr const char VERSION[] = "v0.0.0";  // OK
#endif

#ifdef LCD_ROTATION
constexpr int INIT_LCD_ROTATION = LCD_ROTATION; /**< Current rotation of the LCD. */
#else
constexpr int INIT_LCD_ROTATION = 1; /**< Current rotation of the LCD. */
#endif

#ifdef USE_GROVE
constexpr bool USE_GROVE = 1;
#else
constexpr bool USE_GROVE = 0;
#endif

#endif  // RIBERRY_CONFIG_H
