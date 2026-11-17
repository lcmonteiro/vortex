/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.atan2 component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_ATAN2_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_ATAN2_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief atan2(y, x) with n1 == y and n2 == x.
struct atan2_operation : binary_operation<atan2_operation> {
  template <class T>
  auto value(const T& y, const T& x) const {
    return std::atan2(y, x);
  }
  template <class T>
  auto dvalue(const duo<T>& y, const duo<T>& x) const {
    const T denom = y.v * y.v + x.v * x.v;
    return (x.v * y.d - y.v * x.d) / denom;
  }
  template <class T>
  auto dvalue(const duo<T>& y, const T& x) const {
    const T denom = y.v * y.v + x * x;
    return (x * y.d) / denom;
  }
  template <class T>
  auto dvalue(const T& y, const duo<T>& x) const {
    const T denom = y * y + x.v * x.v;
    return (-y * x.d) / denom;
  }
};

/// @brief Computes atan2(y, x), propagating derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto atan2(const number<T>& n1, const number<T>& n2) {
  return atan2_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto atan2(const number<T>& n1, const S& v2) {
  return atan2_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto atan2(const S& v1, const number<T>& n2) {
  return atan2_operation{}(static_cast<T>(v1), n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_ATAN2_HPP
