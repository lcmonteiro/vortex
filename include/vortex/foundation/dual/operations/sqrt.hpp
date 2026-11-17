/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.sqrt component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Square-root operation.
struct sqrt_operation : unary_operation<sqrt_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::sqrt(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return n.d / (T{2} * std::sqrt(n.v));
  }
};
/// @brief Computes the square root of a dual number.
template <class T>
inline auto sqrt(const number<T>& n) {
  return sqrt_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_SQRT_HPP
