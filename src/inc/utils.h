//
// Created by Kurosu Chan on 2023/5/22.
//

#ifndef SIMPLE_UTILS_H
#define SIMPLE_UTILS_H

#include "ch32v003fun.h"
#include <cstdio>
#include <cstdlib>
#include <printf.h>
#include <etl/vector.h>

namespace utils {
  /**
   * @brief random number generator
   * @deprecated You bastard! used 6kb of FLASH! use `etl::random` instead!
   */
  int rand_range(int min, int max);
  void printWithSize(const uint8_t *str, size_t size, bool hex = false);
  void printWithSize(const etl::ivector<uint8_t> &vec, bool hex = false);
}


#endif //SIMPLE_UTILS_H
