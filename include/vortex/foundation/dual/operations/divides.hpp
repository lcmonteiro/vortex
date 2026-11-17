/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.divides component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_DIVIDES_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_DIVIDES_HPP

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Division operation.
struct divides_operation : binary_operation<divides_operation> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 / v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n2.v * n1.d - n1.v * n2.d) / (n2.v * n2.v);
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (-v1 * n2.d) / (n2.v * n2.v);
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (v2 * n1.d) / (v2 * v2);
  }
};

/// @brief Divides two operands, propagating derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto operator/(const number<T>& n1, const number<T>& n2) {
  return divides_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto operator/(const number<T>& n1, const S& v2) {
  return divides_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto operator/(const S& v1, const number<T>& n2) {
  return divides_operation{}(static_cast<T>(v1), n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_DIVIDES_HPP
