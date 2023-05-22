//
// Created by Kurosu Chan on 2023/5/22.
//

#ifndef SIMPLE_INSTANT_H
#define SIMPLE_INSTANT_H

#include "system_tick.h"
#include <chrono>
#ifdef FUNCTIONAL
#include <functional>
#else
#include <utility>
#endif

class Instant {
  uint64_t time;
public:
  Instant() {
    this->time = millis();
  }

  auto elapsed() {
    auto now = millis();
    auto diff = now - this->time;
    auto duration = std::chrono::duration<uint64_t, std::milli>(diff);
    return duration;
  }

  void reset() {
    auto now = millis();
    this->time = std::move(now);
  }

  auto elapsed_and_reset() {
    auto now = millis();
    auto diff = now - this->time;
    auto duration = std::chrono::duration<uint64_t, std::milli>(diff);
    this->time = now;
    return duration;
  }

  /// for some reason this function always returns 0
  [[nodiscard]] uint64_t count() const {
    return this->time;
  }

  #ifdef FUNCTIONAL
  /// region `FLASH' overflowed by 3934 bytes
  template<typename T>
  void try_run(std::function<T> f, std::chrono::duration<uint64_t, std::milli> d) {
    if (this->elapsed() > d) {
      f();
      this->reset();
    }
  }
  #endif
};

#endif //SIMPLE_INSTANT_H
