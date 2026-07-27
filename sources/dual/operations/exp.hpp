#pragma once

#include <cmath>
#include <functional>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct exp : unary_operation<exp> {
  template <class T>
  auto value(const T& v) const {
    return std::exp(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return std::exp(n.v) * n.d;
  }
};
}  // namespace g2o_dual::dual

namespace std {
template <class T, g2o_dual::dual::exp::enable_t<T> = 0>
inline auto exp(const T& n) {
  return std::invoke(g2o_dual::dual::exp{}, n);
}
}  // namespace std
