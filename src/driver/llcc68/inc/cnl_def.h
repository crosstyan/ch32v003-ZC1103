//
// Created by Kurosu Chan on 2023/6/25.
//

#ifndef SIMPLE_CNL_DEF_H
#define SIMPLE_CNL_DEF_H

#include <cnl/scaled_integer.h>

using fixed_16_16 = cnl::scaled_integer<uint32_t, cnl::power<-16>>;
using fixed_24_8 = cnl::scaled_integer<uint32_t, cnl::power<-8>>;

#endif // SIMPLE_CNL_DEF_H
