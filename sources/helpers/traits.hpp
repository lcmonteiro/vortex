/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helpers.traits component
/// ===========================================================================
#ifndef VORTEX_HELPERS_TRAITS_HPP
#define VORTEX_HELPERS_TRAITS_HPP
#include <type_traits>
#include <utility>

namespace graph {
namespace trait {
///
/// @brief Identity metafunction that returns the type T unchanged.
///
template <class T>
struct Identity {
  using type = T;
};

///
/// @brief Valid metafunction that checks if a set of types is valid (i.e., can be instantiated).
///
template <class...>
struct Valid : Identity<int> {};

template <class... Ts>
using if_valid = typename Valid<Ts...>::type;

///
/// @brief utils traits for conditionally removing functions
///
template <class T>
inline constexpr bool is_lvalue_reference = std::is_lvalue_reference<T>::value;
template <class T, class U>
inline constexpr bool is_same = std::is_same<T, U>::value;
template <class T>
inline constexpr bool is_const = std::is_const<T>::value;

template <class T>
using if_map_like = typename Valid<           //
    decltype(std::begin(std::declval<T>())),  //
    decltype(std::end(std::declval<T>())),    //
    std::enable_if_t<not is_same<typename T::key_type, typename T::value_type>>>::type;

template <class T>
using if_set_like = typename Valid<           //
    decltype(std::begin(std::declval<T>())),  //
    decltype(std::end(std::declval<T>())),    //
    std::enable_if_t<is_same<typename T::key_type, typename T::value_type>>>::type;

///
/// @brief Adjusts the reference type (to a lvalue reference) of an object based
/// on the traits of a specified type.
///
template <class T>
constexpr T& lreference(T&& x) noexcept {
  return static_cast<T&>(x);
}

}  // namespace trait
}  // namespace graph

#endif  // VORTEX_HELPERS_TRAITS_HPP
