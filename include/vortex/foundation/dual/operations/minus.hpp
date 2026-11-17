/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.minus component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Subtraction operation.
struct minus_operation : binary_operation<minus_operation> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 - v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return n1.d - n2.d;
  }
  template <class T>
  auto dvalue(const T&, const duo<T>& n2) const {
    return -n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T&) const {
    return n1.d;
  }
};

/// @brief Subtracts two operands, propagating derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto operator-(const number<T>& n1, const number<T>& n2) {
  return minus_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto operator-(const number<T>& n1, const S& v2) {
  return minus_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto operator-(const S& v1, const number<T>& n2) {
  return minus_operation{}(static_cast<T>(v1), n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP
