/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.sin component
/// ===========================================================================
#ifndef VORTEX_DUAL_OPERATIONS_SIN_HPP
#define VORTEX_DUAL_OPERATIONS_SIN_HPP

#include <cmath>
#include <functional>

#include "dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Sine operation.
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
}  // namespace vortex::dual

namespace std {
/// @brief Computes the sine of a dual number.
template <class T, vortex::dual::sin::enable_t<T> = 0>
inline auto sin(const T& n) {
  return std::invoke(vortex::dual::sin{}, n);
}
}  // namespace std

#endif  // VORTEX_DUAL_OPERATIONS_SIN_HPP
