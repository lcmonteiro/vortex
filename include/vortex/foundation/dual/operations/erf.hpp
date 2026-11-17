/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.erf component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Error-function operation.
struct erf_operation : unary_operation<erf_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::erf(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return two_over_sqrt_pi<T> * std::exp(-n.v * n.v) * n.d;
  }

 private:
  template <class T>
  static constexpr T two_over_sqrt_pi = T{1.12837916709551257389615890312154517L};
};

/// @brief Complementary error-function operation.
struct erfc_operation : unary_operation<erfc_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::erfc(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -two_over_sqrt_pi<T> * std::exp(-n.v * n.v) * n.d;
  }

 private:
  template <class T>
  static constexpr T two_over_sqrt_pi = T{1.12837916709551257389615890312154517L};
};

/// @brief Computes the error function of a dual number.
template <class T>
inline auto erf(const number<T>& n) {
  return erf_operation{}(n);
}
/// @brief Computes the complementary error function of a dual number.
template <class T>
inline auto erfc(const number<T>& n) {
  return erfc_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
