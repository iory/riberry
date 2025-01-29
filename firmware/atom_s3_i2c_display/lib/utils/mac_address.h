#ifndef MAC_ADDRESS_H
#define MAC_ADDRESS_H

#include <color.h>
#include <english.h>
#include <Crypto.h>

const int num_words = sizeof(english_table) / sizeof(english_table[0]);
const String colors[] = { Color::Foreground::RED, Color::Foreground::GREEN, Color::Foreground::BLUE, Color::Foreground::YELLOW, Color::Foreground::MAGENTA, Color::Foreground::CYAN };
const int num_colors = sizeof(colors) / sizeof(colors[0]);

String fancyMacAddress(const char* seed) {
  SHA256 hasher;
  byte hash[SHA256_SIZE];

  hasher.doUpdate(seed, strlen(seed));
  hasher.doFinal(hash);

  uint32_t adjective_index = ((uint32_t)hash[0] << 24 | (uint32_t)hash[1] << 16 | (uint32_t)hash[2] << 8 | (uint32_t)hash[3]) % num_words;
  uint32_t noun_index = ((uint32_t)hash[4] << 24 | (uint32_t)hash[5] << 16 | (uint32_t)hash[6] << 8 | (uint32_t)hash[7]) % num_words;
  uint32_t color_index = ((uint32_t)hash[8] << 24 | (uint32_t)hash[9] << 16 | (uint32_t)hash[10] << 8 | (uint32_t)hash[11]) % num_colors;

  String str = colors[color_index] + String(english_table[adjective_index]) + "-" + String(english_table[noun_index]) + Color::Foreground::RESET;
  return str;
}

#endif  // MAC_ADDRESS_H