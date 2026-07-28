#pragma once

#include "dual/operations/base.hpp"

namespace vortex::dual {
struct negate : unary_operation<negate> {
  template <class T>
  auto value(const T& v) const {
    return -v;
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -n.d;
  }
};

template <class T, negate::enable_t<T> = 0>
inline auto operator-(const T& n) {
  return std::invoke(negate{}, n);
}
}  // namespace vortex::dual
