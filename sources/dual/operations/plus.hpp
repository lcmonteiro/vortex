
#pragma once

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct plus : binary_operation<plus> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 + v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return n1.d + n2.d;
  }
  template <class T>
  auto dvalue(const T&, const duo<T>& n2) const {
    return n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T&) const {
    return n1.d;
  }
};

template <class T, class U, plus::enable_t<T, U> = 0>
inline auto operator+(const T& n1, const U& n2) {
  return std::invoke(plus{}, n1, n2);
}
}  // namespace g2o_dual::dual
