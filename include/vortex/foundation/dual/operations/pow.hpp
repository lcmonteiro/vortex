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
///
/// The two terms of the derivative are kept apart and each answers zero wherever its contribution
/// is zero, rather than evaluating a product with an infinite or undefined factor in it:
/// - through the base, e b^(e-1) db is zero when e or db is, though 0^-1 is infinite;
/// - through the exponent, b^e ln(b) de is zero when de is, though ln(b) is infinite at b = 0 and
///   undefined for b < 0, and zero at b = 0 (0^e is constant for e > 0).
struct pow_operation : binary_operation<pow_operation> {
  template <class T>
  auto value(const T& base, const T& exponent) const {
    return std::pow(base, exponent);
  }
  template <class T>
  auto dvalue(const duo<T>& base, const duo<T>& exponent) const {
    return through_base(base.v, base.d, exponent.v) +
           through_exponent(base.v, exponent.v, exponent.d);
  }
  template <class T>
  auto dvalue(const duo<T>& base, const T& exponent) const {
    return through_base(base.v, base.d, exponent);
  }
  template <class T>
  auto dvalue(const T& base, const duo<T>& exponent) const {
    return through_exponent(base, exponent.v, exponent.d);
  }

 private:
  template <class T>
  static auto through_base(const T& b, const T& db, const T& e) -> T {
    if (e == T{0} or db == T{0}) {
      return T{0};
    }
    return e * std::pow(b, e - T{1}) * db;
  }
  template <class T>
  static auto through_exponent(const T& b, const T& e, const T& de) -> T {
    if (de == T{0} or b == T{0}) {
      return T{0};
    }
    return std::pow(b, e) * std::log(b) * de;
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
