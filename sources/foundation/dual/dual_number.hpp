/// ===========================================================================
/// @file
///
/// @brief vortex.dual.number component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP
#define VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP

#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include "helpers/memory.hpp"

namespace vortex::dual {

template <std::size_t N>
using make_sequence = std::make_index_sequence<N>;
template <std::size_t... I>
using sequence = std::index_sequence<I...>;
template <std::size_t I>
using index = std::integral_constant<std::size_t, I>;

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

  static constexpr auto kMaxIndex = 100000;

  number() = default;
  number(const number& other)
      : value_{other.value_},
        dindex_{other.dindex_, resource()},
        dvalue_{other.dvalue_, resource()} {}
  number(number&&) = default;

  /// @brief Constructs a constant dual number with a zero derivative.
  /// @param value The scalar value.
  explicit number(const value_t& value)
      : value_{value},  //
        dindex_{resource()},
        dvalue_{resource()} {}

  /// @brief Constructs an independent variable seeded at the given index.
  /// @param value The scalar value.
  /// @param index Derivative index assigned to this variable (derivative 1).
  explicit number(const value_t& value, index_t index)
      : value_{value},  //
        dindex_({index}, resource()),
        dvalue_(index + 1, value_t{0}, resource()) {
    assert(index < kMaxIndex);
    dvalue_.back() = value_t{1};
  }

  auto operator=(const number&) -> number<T>& = default;
  auto operator=(number&&) -> number<T>& = default;

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
    assert(i < dvalue_.size());
    return dvalue_[i];
  }

  /// @brief Gets the number of derivative components.
  /// @return The size of the derivative storage.
  auto size() const -> std::size_t { return dvalue_.size(); }

 protected:
  number(const value_t& value,    //< The scalar value.
         const dindex_t& dindex,  //< The active derivative indices.
         const dvalue_t& dvalue   //< The derivative values.
         )
      : value_{value}, dindex_{dindex, resource()}, dvalue_{dvalue, resource()} {}
  template <class Derived>
  friend struct unary_operation;
  template <class Derived>
  friend struct binary_operation;

 private:
  static auto resource() noexcept -> std::pmr::memory_resource* {
    return helpers::MemoryScope::GetResource();
  }

  value_t value_{};
  dindex_t dindex_{resource()};
  dvalue_t dvalue_{resource()};
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

/// @brief Creates a dual number seeded as an independent variable.
/// @tparam U Scalar type.
/// @param init Initial scalar value.
/// @param i Derivative index assigned to the variable.
/// @return The constructed dual number.
template <class U>
inline auto make_number_(const U& init, std::size_t i = 0) {
  return number{init, i};
}

/// @brief Creates an array of dual numbers, each seeded at its own index with an optional offset.
/// @tparam N Number of elements to create.
/// @tparam U Scalar type of the initial value.
/// @tparam O Index offset applied to each variable's seed (default is 0).
/// @param init Initial scalar value for every dual element.
/// @param offset Compile-time index offset wrapper (default is index<0>).
/// @return An array of `N` independent dual variables.
template <std::size_t N, std::size_t O = 0, class U, std::size_t... I>
constexpr auto array(const U& init, sequence<I...> = {}) {
  if constexpr (sizeof...(I) != N) {
    return array<N, O>(init, make_sequence<N>{});
  } else {
    return std::array{number{init, I + O}...};
  }
}
template <class U, std::size_t N, std::size_t O = 0>
constexpr auto zeros() {
  return array<N, O>(U{0});
}

template <class T>
struct is_number : std::false_type {};
template <class T>
struct is_number<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_v = is_number<T>::value;

template <class T>
struct is_number_like : std::is_arithmetic<T> {};
template <class T>
struct is_number_like<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_like_v = is_number_like<T>::value;

}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_DUAL_NUMBER_HPP
