/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.exp component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Exponential operation.
struct exp_operation : unary_operation<exp_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::exp(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return std::exp(n.v) * n.d;
  }
};
/// @brief Computes the exponential of a dual number.
template <class T>
inline auto exp(const number<T>& n) {
  return exp_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP
