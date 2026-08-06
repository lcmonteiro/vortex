/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.erf component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
#include <cmath>
#include <functional>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Error-function operation.
struct erf : unary_operation<erf> {
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
struct erfc : unary_operation<erfc> {
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

}  // namespace vortex::dual

namespace std {
/// @brief Computes the error function of a dual number.
template <class T>
requires vortex::dual::erf::enable<T>
inline auto erf(const T& n) {
  return std::invoke(vortex::dual::erf{}, n);
}
/// @brief Computes the complementary error function of a dual number.
template <class T>
requires vortex::dual::erfc::enable<T>
inline auto erfc(const T& n) {
  return std::invoke(vortex::dual::erfc{}, n);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_ERF_HPP
