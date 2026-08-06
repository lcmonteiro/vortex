/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.cos component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP

#include <cmath>
#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Cosine operation.
struct cos : unary_operation<cos> {
  template <class T>
  auto value(const T& v) const {
    return std::cos(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -std::sin(n.v) * n.d;
  }
};
}  // namespace vortex::dual

namespace std {
/// @brief Computes the cosine of a dual number.
template <class T>
requires vortex::dual::cos::enable<T>
inline auto cos(const T& n) {
  return std::invoke(vortex::dual::cos{}, n);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_COS_HPP
