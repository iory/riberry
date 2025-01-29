#ifndef COLOR_H
#define COLOR_H

#include <Arduino.h>

namespace Color {
// ANSI Escape Codes
const String CSI = "\033[";

// Foreground Colors
namespace Foreground {
const String BLACK = CSI + "30m";
const String RED = CSI + "31m";
const String GREEN = CSI + "32m";
const String YELLOW = CSI + "33m";
const String BLUE = CSI + "34m";
const String MAGENTA = CSI + "35m";
const String CYAN = CSI + "36m";
const String WHITE = CSI + "37m";
const String RESET = CSI + "39m";

const String LIGHTBLACK = CSI + "90m";
const String LIGHTRED = CSI + "91m";
const String LIGHTGREEN = CSI + "92m";
const String LIGHTYELLOW = CSI + "93m";
const String LIGHTBLUE = CSI + "94m";
const String LIGHTMAGENTA = CSI + "95m";
const String LIGHTCYAN = CSI + "96m";
const String LIGHTWHITE = CSI + "97m";
}  // namespace Foreground

// Background Colors
namespace Background {
const String BLACK = CSI + "40m";
const String RED = CSI + "41m";
const String GREEN = CSI + "42m";
const String YELLOW = CSI + "43m";
const String BLUE = CSI + "44m";
const String MAGENTA = CSI + "45m";
const String CYAN = CSI + "46m";
const String WHITE = CSI + "47m";
const String RESET = CSI + "49m";

const String LIGHTBLACK = CSI + "100m";
const String LIGHTRED = CSI + "101m";
const String LIGHTGREEN = CSI + "102m";
const String LIGHTYELLOW = CSI + "103m";
const String LIGHTBLUE = CSI + "104m";
const String LIGHTMAGENTA = CSI + "105m";
const String LIGHTCYAN = CSI + "106m";
const String LIGHTWHITE = CSI + "107m";
}  // namespace Background

// Text Styles
namespace Style {
const String RESET = CSI + "0m";
const String BOLD = CSI + "1m";
const String DIM = CSI + "2m";
const String UNDERLINE = CSI + "4m";
const String BLINK = CSI + "5m";
const String REVERSE = CSI + "7m";
const String HIDDEN = CSI + "8m";
}  // namespace Style
}  // namespace Color

#endif  // COLOR_H
