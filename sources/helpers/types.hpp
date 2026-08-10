/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.types component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_TYPES_HPP
#define VORTEX_HELPERS_TYPES_HPP
#include <tuple>

namespace vortex::helpers {

/// @brief Container that stores and manipulates a compile-time list of types.
///
/// This template class `types` stores a list of types passed as template
/// arguments (`Ts...`). It provides several helper aliases to operate on the
/// type list, such as concatenation, applying templates, and accessing
/// individual types.
template <class... Ts>
class types {
  template <class>
  struct concat_impl;
  template <class... Tn>
  struct concat_impl<types<Tn...>> {
    using type = types<Ts..., Tn...>;
  };

 public:
  template <template <class...> class T>
  using apply_t = T<Ts...>;

  template <template <class...> class T, template <class> class W>
  using apply_wrapped_t = T<W<Ts>...>;

  template <std::size_t index>
  using element_t = std::tuple_element_t<index, apply_t<std::tuple>>;

  template <class T>
  using concat_t = typename concat_impl<T>::type;
};

/// @brief Builds a type `B<Ts...>` from the types stored in a `types` list.
template <template <class...> class B, class Ts>
using types_build_t = typename Ts::template apply_t<B>;

/// @brief Builds a type `B<W<Ts>...>` by wrapping each stored type with `W` first.
template <template <class...> class B, template <class> class W, class Ts>
using types_wrap_build_t = typename Ts::template apply_wrapped_t<B, W>;

/// @brief Helper alias to access the type at a specific index in a `types` list.
template <std::size_t index, class Ts>
using types_element_build_t = typename Ts::template element_t<index>;

}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_TYPES_HPP