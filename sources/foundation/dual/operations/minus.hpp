/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.minus component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Subtraction operation.
struct minus : binary_operation<minus> {
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
template <class T, class U>
requires minus::enable<T, U>
inline auto operator-(const T& n1, const U& n2) {
  return std::invoke(minus{}, n1, n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_MINUS_HPP
