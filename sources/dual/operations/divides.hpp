#pragma once

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct divides : binary_operation<divides> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 / v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n2.v * n1.d - n1.v * n2.d) / (n2.v * n2.v);
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (-v1 * n2.d) / (n2.v * n2.v);
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (v2 * n1.d) / (v2 * v2);
  }
};

template <class T, class U, divides::enable_t<T, U> = 0>
inline auto operator/(const T& n1, const U& n2) {
  return std::invoke(divides{}, n1, n2);
}
}  // namespace g2o_dual::dual
