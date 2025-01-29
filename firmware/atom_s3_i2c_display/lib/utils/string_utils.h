#ifndef STRING_UTILS_H
#define STRING_UTILS_H


inline bool compareIgnoringEscapeSequences(const String &str1, const String &str2) {
  int i1 = 0, i2 = 0;
  while (i1 < str1.length() && i2 < str2.length()) {
    if (i1 >= str1.length() || i2 >= str2.length() || str1.charAt(i1) != str2.charAt(i2)) {
      return false;
    }
    i1++;
    i2++;
  }
  return i1 == str1.length() && i2 == str2.length();
}


#endif
