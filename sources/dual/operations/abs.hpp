#pragma once

#include <cmath>

#include "dual/operations/base.hpp"

namespace vortex::dual {

struct abs : unary_operation<abs> {
  template <class T>
  auto value(const T& v) const {
    return std::abs(v);
  }

  template <class T>
  auto dvalue(const duo<T>& n) const {
    return n.v < T{0} ? -n.d : (n.v > T{0} ? n.d : T{0});
  }
};

}  // namespace vortex::dual

namespace std {
template <class T, vortex::dual::abs::enable_t<T> = 0>
inline auto abs(const T& n) {
  return std::invoke(vortex::dual::abs{}, n);
}
}  // namespace std
