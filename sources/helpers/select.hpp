/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.select component
/// ===========================================================================
#ifndef VORTEX_HELPERS_SELECT_HPP
#define VORTEX_HELPERS_SELECT_HPP

#include <array>
#include <functional>
#include <type_traits>

namespace vortex::helpers {

/// @brief Defines options for selecting between different configurations.
enum class option {
  kAll,   ///< Select both right and left configurations.
  kLeft,  ///< Select only the left configuration.
  kRight  ///< Select only the right configuration.
};

/// @brief This provides a compile-time constant for a given Option value.
/// @tparam value The option value to be wrapped as a compile-time constant.
template <option value>
using option_constant = std::integral_constant<option, value>;

/// @brief Selects between right and left configurations based on the given option.
/// @tparam value The option specifying which configuration(s) to select.
/// @tparam T The type of the `left` and `right` parameters (deduced
/// automatically).
/// @param right Reference to the "right" configuration.
/// @param left Reference to the "left" configuration.
/// @return A `std::array` of references to the selected configurations:
///         - If `value` is `Option::kAll`, returns both `left` and `right`.
///         - If `value` is `Option::kLeft`, returns only `left`.
///         - If `value` is `Option::kRight`, returns only `right`.
/* LCOV_EXCL_START : Constexpr */
template <class T, option value>
auto select(T& left, T& right, const option_constant<value>) {
  if constexpr (value == option::kAll) {
    return std::array{std::ref(left), std::ref(right)};
  }
  if constexpr (value == option::kLeft) {
    return std::array{std::ref(left)};
  }
  if constexpr (value == option::kRight) {
    return std::array{std::ref(right)};
  }
}
/* LCOV_EXCL_STOP : Constexpr */
/* LCOV_EXCL_START : Constexpr */
template <class T, option value>
auto cselect(T& left, T& right, const option_constant<value>) {
  if constexpr (value == option::kAll) {
    return std::array{std::cref(left), std::cref(right)};
  }
  if constexpr (value == option::kLeft) {
    return std::array{std::cref(left)};
  }
  if constexpr (value == option::kRight) {
    return std::array{std::cref(right)};
  }
}
/* LCOV_EXCL_STOP : Constexpr */
}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_SELECT_HPP
