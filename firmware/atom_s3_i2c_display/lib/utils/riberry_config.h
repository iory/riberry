#ifndef RIBERRY_CONFIG_H
#define RIBERRY_CONFIG_H

#include <Arduino.h>

#ifdef RIBERRY_VERSION
constexpr const char VERSION[] = RIBERRY_VERSION;
#else
constexpr const char VERSION[] = "v0.0.0";  // OK
#endif

#endif  // RIBERRY_CONFIG_H