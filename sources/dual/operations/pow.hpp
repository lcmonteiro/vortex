#pragma once

#include <cmath>
#include <functional>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
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
}  // namespace g2o_dual::dual

namespace std {
template <class T, class U, g2o_dual::dual::power::enable_t<T, U> = 0>
inline auto pow(const T& base, const U& exponent) {
  return std::invoke(g2o_dual::dual::power{}, base, exponent);
}
}  // namespace std
