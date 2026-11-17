/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.cos component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Cosine operation.
struct cos_operation : unary_operation<cos_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::cos(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -std::sin(n.v) * n.d;
  }
};
/// @brief Computes the cosine of a dual number.
template <class T>
inline auto cos(const number<T>& n) {
  return cos_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP
