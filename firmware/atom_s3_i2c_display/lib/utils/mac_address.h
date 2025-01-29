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

  uint64_t adjective_index = ((uint64_t)hash[0] << 56 | (uint64_t)hash[1] << 48 | (uint64_t)hash[2] << 40 |
                               (uint64_t)hash[3] << 32 | (uint64_t)hash[4] << 24 | (uint64_t)hash[5] << 16 |
                               (uint64_t)hash[6] << 8  | (uint64_t)hash[7]) ^
                              ((uint64_t)hash[8] << 32 | (uint64_t)hash[9] << 24 | (uint64_t)hash[10] << 16 | (uint64_t)hash[11] << 8);
  adjective_index %= num_words;

  uint64_t noun_index = ((uint64_t)hash[12] << 56 | (uint64_t)hash[13] << 48 | (uint64_t)hash[14] << 40 |
                          (uint64_t)hash[15] << 32 | (uint64_t)hash[16] << 24 | (uint64_t)hash[17] << 16 |
                          (uint64_t)hash[18] << 8  | (uint64_t)hash[19]) ^
                         ((uint64_t)hash[20] << 32 | (uint64_t)hash[21] << 24 | (uint64_t)hash[22] << 16 | (uint64_t)hash[23] << 8);
  noun_index %= num_words;

  uint64_t color_index = ((uint64_t)hash[24] << 56 | (uint64_t)hash[25] << 48 | (uint64_t)hash[26] << 40 |
                           (uint64_t)hash[27] << 32 | (uint64_t)hash[28] << 24 | (uint64_t)hash[29] << 16 |
                           (uint64_t)hash[30] << 8  | (uint64_t)hash[31]);
  color_index %= num_colors;


  String str = colors[color_index] + String(english_table[adjective_index]) + "-" + String(english_table[noun_index]) + Color::Foreground::RESET;
  return str;
}

#endif  // MAC_ADDRESS_H
