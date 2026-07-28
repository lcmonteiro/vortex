/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.sqrt component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP

#include <cmath>
#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Square-root operation.
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
}  // namespace vortex::dual

namespace std {
/// @brief Computes the square root of a dual number.
template <class T, vortex::dual::sqrt::enable_t<T> = 0>
inline auto sqrt(const T& n) {
  return std::invoke(vortex::dual::sqrt{}, n);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP
