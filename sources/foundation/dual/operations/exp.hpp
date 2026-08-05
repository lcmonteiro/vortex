/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.exp component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP

#include <cmath>
#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Exponential operation.
struct exp : unary_operation<exp> {
  template <class T>
  auto value(const T& v) const {
    return std::exp(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return std::exp(n.v) * n.d;
  }
};
}  // namespace vortex::dual

namespace std {
/// @brief Computes the exponential of a dual number.
template <class T>
requires vortex::dual::exp::enable<T>
inline auto exp(const T& n) {
  return std::invoke(vortex::dual::exp{}, n);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_EXP_HPP
