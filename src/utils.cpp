//
// Created by Kurosu Chan on 2023/5/22.
//
#include "utils.h"

int utils::rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

/// won't add trailing `LF` or `CRLF` and caller should decide whether to add one.
void utils::printWithSize(const char *str, size_t size, bool hex) {
  for (size_t i = 0; i < size; i++) {
    if (hex) {
      // https://stackoverflow.com/questions/61518810/print-the-value-of-a-pointer-in-hex-format-without-printf
      uint8_t hi = (str[i] >> 4) & 0xf;
      uint8_t lo = str[i] & 0xf;
      uint8_t tmp[2] = {hi, lo};

      tmp[0] += hi < 10 ? '0' : 'a' - 10;
      tmp[1] += lo < 10 ? '0' : 'a' - 10;
      putchar(tmp[0]);
      putchar(tmp[1]);
    } else {
      putchar(str[i]);
    }
  }
};
