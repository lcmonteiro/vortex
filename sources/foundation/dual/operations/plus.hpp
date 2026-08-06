/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.plus component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_PLUS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_PLUS_HPP

#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Addition operation.
struct plus : binary_operation<plus> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return v1 + v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return n1.d + n2.d;
  }
  template <class T>
  auto dvalue(const T&, const duo<T>& n2) const {
    return n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T&) const {
    return n1.d;
  }
};

/// @brief Adds two operands, propagating derivatives.
template <class T, class U>
requires plus::enable<T, U>
inline auto operator+(const T& n1, const U& n2) {
  return std::invoke(plus{}, n1, n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_PLUS_HPP
