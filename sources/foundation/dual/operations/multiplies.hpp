/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.multiplies component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_MULTIPLIES_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_MULTIPLIES_HPP

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Multiplication operation.
struct multiplies : binary_operation<multiplies> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 * v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return n1.v * n2.d + n2.v * n1.d;
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return v1 * n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return v2 * n1.d;
  }
};

/// @brief Multiplies two operands, propagating derivatives.
template <class T, class U>
requires multiplies::enable<T, U>
inline auto operator*(const T& n1, const U& n2) {
  return std::invoke(multiplies{}, n1, n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_MULTIPLIES_HPP
