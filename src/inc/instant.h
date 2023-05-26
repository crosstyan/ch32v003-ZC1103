//
// Created by Kurosu Chan on 2023/5/22.
//

#ifndef SIMPLE_INSTANT_H
#define SIMPLE_INSTANT_H

#include "system_tick.h"
#include <etl/delegate.h>
#include <chrono>

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

  [[nodiscard]] uint64_t count() const {
    return this->time;
  }

  template<typename T>
  void try_run(etl::delegate<T> f, std::chrono::duration<uint64_t, std::milli> d) {
    if (this->elapsed() > d) {
      f();
      this->reset();
    }
  }
};

#endif //SIMPLE_INSTANT_H
