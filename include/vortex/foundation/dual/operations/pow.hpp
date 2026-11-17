/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.pow component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief pow(base, exponent) with n1 == base and n2 == exponent.
struct pow_operation : binary_operation<pow_operation> {
  template <class T>
  auto value(const T& base, const T& exponent) const {
    return std::pow(base, exponent);
  }
  template <class T>
  auto dvalue(const duo<T>& base, const duo<T>& exponent) const {
    return exponent.v * std::pow(base.v, exponent.v - T{1}) * base.d +
           std::pow(base.v, exponent.v) * std::log(base.v) * exponent.d;
  }
  template <class T>
  auto dvalue(const duo<T>& base, const T& exponent) const {
    return exponent * std::pow(base.v, exponent - T{1}) * base.d;
  }
  template <class T>
  auto dvalue(const T& base, const duo<T>& exponent) const {
    return std::pow(base, exponent.v) * std::log(base) * exponent.d;
  }
};
/// @brief Raises base to exponent, propagating derivatives.
/// @note A scalar operand is converted to the number's scalar type first.
template <class T>
inline auto pow(const number<T>& n1, const number<T>& n2) {
  return pow_operation{}(n1, n2);
}
template <class T, scalar S>
inline auto pow(const number<T>& n1, const S& v2) {
  return pow_operation{}(n1, static_cast<T>(v2));
}
template <class T, scalar S>
inline auto pow(const S& v1, const number<T>& n2) {
  return pow_operation{}(static_cast<T>(v1), n2);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP
