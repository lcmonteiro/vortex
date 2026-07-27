/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.select component
/// ===========================================================================
#ifndef VORTEX_HELPERS_SELECT_HPP
#define VORTEX_HELPERS_SELECT_HPP

#include <array>

namespace graph {
namespace detail {

/// @enum Option
/// @brief Defines options for selecting between different configurations.
enum class Option {
  kAll,   ///< Select both right and left configurations.
  kLeft,  ///< Select only the left configuration.
  kRight  ///< Select only the right configuration.
};

/// @brief  This provides a compile-time constant for a given Option value.
///
/// @tparam value The option value to be wrapped as a compile-time constant.
template <Option value>
using OptionConstant = std::integral_constant<Option, value>;

/// @brief Selects between rightd and leftd configurations based on the
/// given option.
///
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
template <class T, Option value>
auto select(T& left, T& right, const OptionConstant<value>) {
  if constexpr (value == Option::kAll) {
    return std::array{ref(left), ref(right)};
  }
  if constexpr (value == Option::kLeft) {
    return std::array{ref(left)};
  }
  if constexpr (value == Option::kRight) {
    return std::array{ref(right)};
  }
}
/* LCOV_EXCL_STOP : Constexpr */
/* LCOV_EXCL_START : Constexpr */
template <class T, Option value>
auto cselect(T& left, T& right, const OptionConstant<value>) {
  if constexpr (value == Option::kAll) {
    return std::array{cref(left), cref(right)};
  }
  if constexpr (value == Option::kLeft) {
    return std::array{cref(left)};
  }
  if constexpr (value == Option::kRight) {
    return std::array{cref(right)};
  }
}
/* LCOV_EXCL_STOP : Constexpr */
}  // namespace detail
}  // namespace graph

#endif  // VORTEX_HELPERS_SELECT_HPP
