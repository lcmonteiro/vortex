#pragma once

#include <cmath>
#include <functional>

#include "dual/operations/base.hpp"

namespace vortex::dual {
/// @brief atan2(y, x) with n1 == y and n2 == x.
struct atan2 : binary_operation<atan2> {
  template <class T>
  auto value(const T& y, const T& x) const {
    return std::atan2(y, x);
  }
  template <class T>
  auto dvalue(const duo<T>& y, const duo<T>& x) const {
    const T denom = y.v * y.v + x.v * x.v;
    return (x.v * y.d - y.v * x.d) / denom;
  }
  template <class T>
  auto dvalue(const duo<T>& y, const T& x) const {
    const T denom = y.v * y.v + x * x;
    return (x * y.d) / denom;
  }
  template <class T>
  auto dvalue(const T& y, const duo<T>& x) const {
    const T denom = y * y + x.v * x.v;
    return (-y * x.d) / denom;
  }
};

template <class T, class U, atan2::enable_t<T, U> = 0>
inline auto atan2_impl(const T& y, const U& x) {
  return std::invoke(atan2{}, y, x);
}
}  // namespace vortex::dual

namespace std {
template <class T, class U, vortex::dual::atan2::enable_t<T, U> = 0>
inline auto atan2(const T& y, const U& x) {
  return std::invoke(vortex::dual::atan2{}, y, x);
}
}  // namespace std
