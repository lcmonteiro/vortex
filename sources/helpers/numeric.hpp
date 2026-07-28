/// ===========================================================================
/// @file
/// @brief vortex.helpers.numeric component
///
/// Small standard-library based numeric utilities.
/// ===========================================================================
#ifndef VORTEX_HELPERS_NUMERIC_HPP
#define VORTEX_HELPERS_NUMERIC_HPP

#include <utility>

namespace vortex::helpers {

/// @brief Narrowing cast helper.
/// @tparam To   Target type.
/// @tparam From Source type.
/// @param value The value to convert.
/// @return The value converted to the target type.
template <class To, class From>
constexpr auto narrow_cast(From&& value) noexcept -> To {
  return static_cast<To>(std::forward<From>(value));
}

/// @brief Compile-time integer power.
/// @tparam Exponent The integer exponent.
/// @tparam T        The base type.
/// @param base The base value.
/// @return base raised to the given exponent.
template <unsigned Exponent, class T>
constexpr auto int_pow(T base) -> T {
  T result{1};
  for (unsigned i = 0; i < Exponent; ++i) {
    result *= base;
  }
  return result;
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_NUMERIC_HPP
