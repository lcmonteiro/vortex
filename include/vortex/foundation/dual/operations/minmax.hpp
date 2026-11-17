/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.minmax component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_MINMAX_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_MINMAX_HPP

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Minimum operation.
struct min_operation : binary_operation<min_operation> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return (v1 <= v2) ? v1 : v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n1.v <= n2.v) ? n1.d : n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (n1.v <= v2) ? n1.d : T{0};
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (v1 <= n2.v) ? T{0} : n2.d;
  }
};
/// @brief Maximum operation.
struct max_operation : binary_operation<max_operation> {
  template <class T>
  auto value(const T& v1, const T& v2) const {
    return (v1 >= v2) ? v1 : v2;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const duo<T>& n2) const {
    return (n1.v >= n2.v) ? n1.d : n2.d;
  }
  template <class T>
  auto dvalue(const duo<T>& n1, const T& v2) const {
    return (n1.v >= v2) ? n1.d : T{0};
  }
  template <class T>
  auto dvalue(const T& v1, const duo<T>& n2) const {
    return (v1 >= n2.v) ? T{0} : n2.d;
  }
};
/// @brief Returns the smaller operand, propagating its derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto min(const number<T>& n1, const number<T>& n2) {
  return min_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto min(const number<T>& n1, const S& v2) {
  return min_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto min(const S& v1, const number<T>& n2) {
  return min_operation{}(static_cast<T>(v1), n2);
}
/// @brief Returns the larger operand, propagating its derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto max(const number<T>& n1, const number<T>& n2) {
  return max_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto max(const number<T>& n1, const S& v2) {
  return max_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto max(const S& v1, const number<T>& n2) {
  return max_operation{}(static_cast<T>(v1), n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_MINMAX_HPP
