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

#include "helpers/contracts.hpp"
#include "helpers/memory.hpp"

namespace vortex::dual {

/// @brief Creates a compile-time sequence of integers from 0 to N-1.
template <std::size_t N>
using make_sequence = std::make_index_sequence<N>;
template <std::size_t... I>
using sequence = std::index_sequence<I...>;

/// @brief Gets the current memory resource for dual number allocations.
/// @return The current memory resource.
inline auto memory() noexcept -> std::pmr::memory_resource* {
  return helpers::memory_scope::get_resource();
}

/// @brief Forward-mode dual number carrying a scalar value and its partial
/// derivatives.
///
/// The derivative components are stored sparsely: `dindex_` lists the active
/// derivative indices and `dvalue_` holds the corresponding derivative values.
///
/// @tparam T Underlying scalar type.
template <class T>
struct number {
  using index_t = std::size_t;
  using value_t = T;
  using dindex_t = std::pmr::vector<index_t>;
  using dvalue_t = std::pmr::vector<value_t>;

  /// @brief Default constructor creates a constant dual number with zero value and no derivatives.
  number() = default;

  /// @brief Copies a dual number, drawing the copy's storage from the memory scope active now.
  ///
  /// A defaulted copy would not do that: `std::pmr::polymorphic_allocator` does not propagate on
  /// copy construction -- `select_on_container_copy_construction` hands back a default-constructed
  /// allocator -- so the copy would come from `std::pmr::get_default_resource()` and silently
  /// bypass the arena the caller installed, however carefully the source was placed.
  number(const number& other)
      : value_{other.value_},
        dindex_{other.dindex_, memory()},
        dvalue_{other.dvalue_, memory()} {}

  /// @brief Moving steals the storage, so the result keeps the source's resource as-is.
  number(number&&) = default;

  /// @brief Constructs a constant dual number with a zero derivative.
  /// @param value The scalar value.
  explicit number(const value_t& value)  //
      : value_{value}, dindex_{memory()}, dvalue_{memory()} {}

  /// @brief Constructs an independent variable seeded at the given index.
  /// @param value The scalar value.
  /// @param index Derivative index assigned to this variable (derivative 1).
  explicit number(const value_t& value, index_t index)
      : value_{value}, dindex_({index}, memory()), dvalue_(index + 1, value_t{0}, memory()) {
    dvalue_.back() = value_t{1};
  }

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

  /// @brief Gets the active derivative indices.
  /// @return The list of active derivative indices.
  auto dindex() const -> const dindex_t& { return dindex_; }

  /// @brief Gets the derivative values.
  /// @return The derivative value components.
  auto dvalue() const -> const dvalue_t& { return dvalue_; }

  /// @brief Gets a single derivative component.
  /// @param i Derivative index.
  /// @return The derivative value at index @p i.
  auto dvalue(index_t i) const -> const value_t& {
    VORTEX_ASSERT(i < std::size(dvalue_), "derivative index out of range");
    return dvalue_[i];
  }

  /// @brief Gets the number of derivative components.
  /// @return The size of the derivative storage.
  auto size() const -> std::size_t { return std::size(dvalue_); }

 protected:
  number(const value_t& value, const dindex_t& dindex, const dvalue_t& dvalue)
      : value_{value}, dindex_{dindex, memory()}, dvalue_{dvalue, memory()} {}

  /// @brief Adopts already-built derivative storage instead of copying it. The operation bases
  /// construct these from `memory()`, so moving preserves both their contents and their resource.
  number(const value_t& value, dindex_t&& dindex, dvalue_t&& dvalue)
      : value_{value}, dindex_{std::move(dindex)}, dvalue_{std::move(dvalue)} {}

  template <class Derived>
  friend struct unary_operation;
  template <class Derived>
  friend struct binary_operation;

 private:
  value_t value_{};
  dindex_t dindex_{memory()};
  dvalue_t dvalue_{memory()};
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
  return []<std::size_t... Is>(sequence<Is...>) {
    return std::array{number<U>{U{0}, O + Is}...};
  }(make_sequence<D>{});
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
