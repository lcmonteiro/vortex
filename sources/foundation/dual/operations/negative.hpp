/// ===========================================================================
/// @file
///
/// @brief vortex.dual.operations.negative component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_NEGATIVE_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_NEGATIVE_HPP

#include "foundation/dual/operations/base.hpp"

namespace vortex::dual {
/// @brief Negation operation.
struct negate : unary_operation<negate> {
  template <class T>
  auto value(const T& v) const {
    return -v;
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -n.d;
  }
};

/// @brief Negates a dual number, propagating derivatives.
template <class T>
requires negate::enable<T>
inline auto operator-(const T& n) {
  return std::invoke(negate{}, n);
}
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_NEGATIVE_HPP
