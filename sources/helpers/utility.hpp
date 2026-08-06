/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.traits component
/// ===========================================================================
#ifndef VORTEX_HELPERS_TRAITS_HPP
#define VORTEX_HELPERS_TRAITS_HPP

namespace vortex::helpers {

/// @brief Concept satisfied by types with distinct key/value types (map-like containers).
template <class T>
concept map_like = requires(T t) {
  typename T::key_type;
  typename T::value_type;
  std::begin(t);
  std::end(t);
  requires not std::is_same_v<typename T::key_type, typename T::value_type>;
};

/// @brief Concept satisfied by types with matching key/value types (set-like containers).
template <class T>
concept set_like = requires(T t) {
  typename T::key_type;
  typename T::value_type;
  std::begin(t);
  std::end(t);
  requires std::is_same_v<typename T::key_type, typename T::value_type>;
};

/// @brief Adjusts the reference type (to an lvalue reference) of an object based
/// on the traits of a specified type.
template <class T>
constexpr auto lreference(T&& x) noexcept -> T& {
  return static_cast<T&>(x);
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_TRAITS_HPP
