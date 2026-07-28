/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.log component
/// ===========================================================================
#ifndef VORTEX_DUAL_OPERATIONS_LOG_HPP
#define VORTEX_DUAL_OPERATIONS_LOG_HPP

#include <cmath>

#include "dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Natural-logarithm operation.
struct log : unary_operation<log> {
  template <class T>
  auto value(const T& v) const {
    return std::log(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    assert(n.v > T{0});
    return n.d / n.v;
  }
};

/// @brief Logarithm of (1 + x) operation.
struct log1p : unary_operation<log1p> {
  template <class T>
  auto value(const T& v) const {
    return std::log1p(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    assert(n.v > T{-1});
    return n.d / (T{1} + n.v);
  }
};
}  // namespace vortex::dual

namespace std {
/// @brief Computes the natural logarithm of a dual number.
template <class T, vortex::dual::log::enable_t<T> = 0>
inline auto log(const T& n) {
  return std::invoke(vortex::dual::log{}, n);
}
/// @brief Computes log(1 + x) of a dual number.
template <class T, vortex::dual::log1p::enable_t<T> = 0>
inline auto log1p(const T& n) {
  return std::invoke(vortex::dual::log1p{}, n);
}
}  // namespace std

#endif  // VORTEX_DUAL_OPERATIONS_LOG_HPP
