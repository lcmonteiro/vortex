#pragma once

#include <cmath>
#include <functional>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct cos : unary_operation<cos> {
  template <class T>
  auto value(const T& v) const {
    return std::cos(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -std::sin(n.v) * n.d;
  }
};
}  // namespace g2o_dual::dual

namespace std {
template <class T, g2o_dual::dual::cos::enable_t<T> = 0>
inline auto cos(const T& n) {
  return std::invoke(g2o_dual::dual::cos{}, n);
}
}  // namespace std
