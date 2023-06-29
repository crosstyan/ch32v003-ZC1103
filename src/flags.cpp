//
// Created by Kurosu Chan on 2023/6/25.
//

#include "flags.h"

namespace Flags {
bool flag = false;

void setFlag(bool value) {
  flag = value;
}

bool getFlag() {
  return flag;
}
}
