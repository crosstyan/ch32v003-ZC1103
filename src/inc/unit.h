//
// Created by Kurosu Chan on 2023/5/24.
//

#ifndef SIMPLE_UNIT_H
#define SIMPLE_UNIT_H


/// equivalent to Rust's `()`
/// or Scala's `Unit`
/// std::monostate should be used?
///
/// should be used with optional
struct Unit {};
// operator copy and paste from std::monostate
constexpr bool operator==(Unit, Unit) noexcept { return true; }
constexpr bool operator!=(Unit, Unit) noexcept { return false; }
constexpr bool operator<(Unit, Unit) noexcept { return false; }
constexpr bool operator>(Unit, Unit) noexcept { return false; }
constexpr bool operator<=(Unit, Unit) noexcept { return true; }
constexpr bool operator>=(Unit, Unit) noexcept { return true; }

/// as for either monad, `std::variant` should be a good choice

#endif //SIMPLE_UNIT_H
