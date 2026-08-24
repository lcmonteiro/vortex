/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.number component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP
#define VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP

#include <array>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include "helpers/indices.hpp"
#include "helpers/memory.hpp"

namespace vortex::dual {

using helpers::indices;
using helpers::make_indices;

/// @brief Gets the current memory resource for dual number allocations.
/// @return The current memory resource.
inline auto memory() noexcept -> std::pmr::memory_resource* {
  return helpers::memory_scope::get_resource();
}

/// @brief Forward-mode dual number carrying a scalar value and its partial derivatives.
/// @tparam T Underlying scalar type.
template <class T>
struct number {
  /// @brief Type aliases for the underlying scalar type, index type, and derivative value type.
  using index_t = std::size_t;
  using value_t = T;
  struct dvalue_t {
    index_t index{};
    value_t value{};
  };
  using dvalues_t = std::pmr::vector<dvalue_t>;

  /// @brief Default constructor: zero value, one derivative of value zero at index 0.
  /// @note The derivative is zero, not one. A default-constructed number exists so that blaze can
  /// lay out a `static_vector<number, N>` before assigning its elements, and so that every number
  /// carries at least one derivative; it is not a variable. Seeding it at one would make an
  /// unassigned residual component read back as depending on the first tangent direction, and the
  /// scatter would write that into the first node's jacobian block instead of a zero row.
  number() : value_{}, dvalues_{{dvalue_t{0, value_t{0}}}, memory()} {}

  /// @brief Copies into storage from the scope active now. A defaulted copy would not:
  /// `std::pmr::polymorphic_allocator` does not propagate on copy construction, so it would draw
  /// from `std::pmr::get_default_resource()` and bypass the arena the caller installed.
  number(const number& n) : value_{n.value_}, dvalues_{n.dvalues_, memory()} {}

  /// @brief Unlike the copy above, a move keeps the source's resource along with its buffer.
  number(number&&) = default;

  /// @brief Constructs an independent variable seeded at the given index.
  /// @param value The scalar value.
  /// @param index Derivative index assigned to this variable (derivative 1).
  explicit number(const value_t& value, index_t index)
      : value_{value}, dvalues_{{dvalue_t{index, value_t{1}}}, memory()} {}

  /// @brief Assignment operators.
  auto operator=(const number&) -> number<T>& = default;
  auto operator=(number&&) -> number<T>& = default;

  /// @brief Implicit conversion to the underlying scalar type.
  operator const value_t&() const { return value_; }

  /// @brief Sets the scalar value.
  /// @param v The new scalar value.
  auto value(const value_t& v) -> void { value_ = v; }

  /// @brief Gets the scalar value.
  /// @return The scalar value.
  auto value() const -> const value_t& { return value_; }

  /// @brief Gets the active partial derivatives, sorted by index.
  /// @return The list of `(index, value)` entries.
  auto dvalues() const -> const dvalues_t& { return dvalues_; }

  /// @brief Gets the number of active partial derivatives.
  /// @note It is never zero.
  /// @return The size of the derivative storage.
  auto size() const -> std::size_t { return std::size(dvalues_); }

 protected:
  /// @brief Builds from existing derivative storage, for the operation bases.
  number(const value_t& value, dvalues_t&& dvalues) : value_{value}, dvalues_{std::move(dvalues)} {}

  template <class Derived>
  friend struct unary_operation;
  template <class Derived>
  friend struct binary_operation;

 private:
  value_t value_{};
  dvalues_t dvalues_{memory()};
};

/// @brief Compares two dual numbers by their scalar value.
template <class T>
inline auto operator<(const number<T>& n1, const number<T>& n2) -> bool {
  return n1.value() < n2.value();
}
/// @brief Compares a dual number and a scalar by value.
template <class T>
inline auto operator<(const number<T>& n1, const T& n2) -> bool {
  return n1.value() < n2;
}
/// @brief Compares a scalar and a dual number by value.
template <class T>
inline auto operator<(const T& n1, const number<T>& n2) -> bool {
  return n1 < n2.value();
}

/// @brief Creates a dual number with a zero derivative vector of size D.
/// @tparam U The underlying scalar type.
/// @tparam D The size of the derivative vector.
/// @tparam O The starting offset for the derivative indices (default is 0).
template <class U, std::size_t D, std::size_t O = 0>
constexpr auto zeros() {
  return []<std::size_t... Is>(indices<Is...>) {
    return std::array{number<U>{U{0}, O + Is}...};
  }(make_indices<D>{});
}

/// @brief Type trait to check if a type is a dual number.
template <class T>
struct is_number : std::false_type {};
template <class T>
struct is_number<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_v = is_number<T>::value;

/// @brief Type trait to check if a type is a dual number or a scalar.
template <class T>
struct is_number_like : std::is_arithmetic<T> {};
template <class T>
struct is_number_like<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_like_v = is_number_like<T>::value;

}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP
