#pragma once

#include <cmath>
#include <functional>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct sqrt : unary_operation<sqrt> {
  template <class T>
  auto value(const T& v) const {
    return std::sqrt(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return n.d / (T{2} * std::sqrt(n.v));
  }
};
}  // namespace g2o_dual::dual

namespace std {
template <class T, g2o_dual::dual::sqrt::enable_t<T> = 0>
inline auto sqrt(const T& n) {
  return std::invoke(g2o_dual::dual::sqrt{}, n);
}
}  // namespace std
