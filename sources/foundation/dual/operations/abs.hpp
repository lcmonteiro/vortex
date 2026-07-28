/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.abs component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP

#include <cmath>

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {

/// @brief Absolute-value operation.
struct abs : unary_operation<abs> {
  template <class T>
  auto value(const T& v) const {
    return std::abs(v);
  }

  template <class T>
  auto dvalue(const duo<T>& n) const {
    return n.v < T{0} ? -n.d : (n.v > T{0} ? n.d : T{0});
  }
};

}  // namespace vortex::dual

namespace std {
/// @brief Computes the absolute value of a dual number.
template <class T, vortex::dual::abs::enable_t<T> = 0>
inline auto abs(const T& n) {
  return std::invoke(vortex::dual::abs{}, n);
}
}  // namespace std

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_ABS_HPP
