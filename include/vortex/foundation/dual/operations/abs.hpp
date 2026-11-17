/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.abs component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {

/// @brief Absolute-value operation.
struct abs_operation : unary_operation<abs_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::abs(v);
  }

  template <class T>
  auto dvalue(const duo<T>& n) const {
    return n.v < T{0} ? -n.d : (n.v > T{0} ? n.d : T{0});
  }
};

/// @brief Computes the absolute value of a dual number.
template <class T>
inline auto abs(const number<T>& n) {
  return abs_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP
