//
// Created by Kurosu Chan on 2023/5/22.
//
#include "utils.h"

int utils::rand_range(int min, int max) {
  return min + (std::rand() % (max - min + 1));
}

