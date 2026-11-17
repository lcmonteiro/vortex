/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.log component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_LOG_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_LOG_HPP

#include <cmath>

#include "vortex/foundation/dual/operations/base.hpp"
#include "vortex/helpers/contracts.hpp"

namespace vortex::dual {
/// @brief Natural-logarithm operation.
struct log_operation : unary_operation<log_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::log(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    VORTEX_ASSERT(n.v > T{0}, "log derivative requires a positive value");
    return n.d / n.v;
  }
};

/// @brief Logarithm of (1 + x) operation.
struct log1p_operation : unary_operation<log1p_operation> {
  template <class T>
  auto value(const T& v) const {
    return std::log1p(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    VORTEX_ASSERT(n.v > T{-1}, "log1p derivative requires a value greater than -1");
    return n.d / (T{1} + n.v);
  }
};
/// @brief Computes the natural logarithm of a dual number.
template <class T>
inline auto log(const number<T>& n) {
  return log_operation{}(n);
}
/// @brief Computes log(1 + x) of a dual number.
template <class T>
inline auto log1p(const number<T>& n) {
  return log1p_operation{}(n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_LOG_HPP
