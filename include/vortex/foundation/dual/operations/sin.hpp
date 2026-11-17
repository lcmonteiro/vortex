/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.sin component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_SIN_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_SIN_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Sine operation.
struct sin_operation : unary_operation<sin_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::sin(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return std::cos(n.v) * n.d;
  }
};
/// @brief Computes the sine of a dual number.
template <class T>
inline auto sin(const number<T>& n) {
  return sin_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_SIN_HPP
