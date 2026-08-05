/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.pow component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP

#include <cmath>
#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief pow(base, exponent) with n1 == base and n2 == exponent.
struct power : binary_operation<power> {
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
}  // namespace vortex::dual

namespace std {
/// @brief Computes base raised to exponent for dual operands.
template <class T, class U>
requires vortex::dual::power::enable<T, U>
inline auto pow(const T& base, const U& exponent) {
  return std::invoke(vortex::dual::power{}, base, exponent);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_POW_HPP
