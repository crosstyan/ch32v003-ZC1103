//
// Created by Kurosu Chan on 2023/5/22.
//

#ifndef SIMPLE_UTILS_H
#define SIMPLE_UTILS_H

#include "ch32v003fun.h"
#include <cstdio>
#include <cstdlib>
#include <printf.h>

namespace utils {
  int rand_range(int min, int max);
  void printWithSize(const char *str, size_t size, bool hex = false);
}


#endif //SIMPLE_UTILS_H
