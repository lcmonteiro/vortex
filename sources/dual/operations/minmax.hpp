#pragma once

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct min : binary_operation<min> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return (v1 <= v2) ? v1 : v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n1.v <= n2.v) ? n1.d : n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (n1.v <= v2) ? n1.d : T{0};
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (v1 <= n2.v) ? T{0} : n2.d;
  }
};
struct max : binary_operation<max> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return (v1 >= v2) ? v1 : v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n1.v >= n2.v) ? n1.d : n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (n1.v >= v2) ? n1.d : T{0};
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (v1 >= n2.v) ? T{0} : n2.d;
  }
};
}  // namespace g2o_dual::dual

namespace std {
template <class T,  //
          class U,  //
          g2o_dual::dual::min::enable_t<T, U> = 0>
inline auto min(const T& n1, const U& n2) {
  return std::invoke(g2o_dual::dual::min{}, n1, n2);
}
template <class T,  //
          class U,  //
          g2o_dual::dual::max::enable_t<T, U> = 0>
inline auto max(const T& n1, const U& n2) {
  return std::invoke(g2o_dual::dual::max{}, n1, n2);
}
}  // namespace std
