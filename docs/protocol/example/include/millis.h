//
// Created by Kurosu Chan on 2023/6/30.
//

#ifndef SE_DE_TEST_MILLIS_H
#define SE_DE_TEST_MILLIS_H

#include <chrono>

inline auto millis() -> uint64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

#endif //SE_DE_TEST_MILLIS_H
