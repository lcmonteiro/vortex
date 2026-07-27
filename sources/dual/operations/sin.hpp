#pragma once

#include <cmath>
#include <functional>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct sin : unary_operation<sin> {
  template <class T>
  auto value(const T& v) const {
    return std::sin(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return std::cos(n.v) * n.d;
  }
};
}  // namespace g2o_dual::dual

namespace std {
template <class T, g2o_dual::dual::sin::enable_t<T> = 0>
inline auto sin(const T& n) {
  return std::invoke(g2o_dual::dual::sin{}, n);
}
}  // namespace std
