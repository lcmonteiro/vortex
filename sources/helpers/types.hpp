/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.types component
/// ===========================================================================
#ifndef VORTEX_HELPERS_TYPES_HPP
#define VORTEX_HELPERS_TYPES_HPP
#include <tuple>

namespace graph {
namespace detail {

/// @brief Types Container
/// This template class `Types` is used to store and manipulate a list of types
/// passed as template arguments (`Ts...`). It provides several helper methods
/// to operate on the type list, such as concatenation, applying templates,
/// and accessing individual types.
template <class... Ts>
class Types {
  template <class>
  struct concat;
  template <class... Tn>
  struct concat<Types<Tn...>> {
    using type = Types<Ts..., Tn...>;
  };

 public:
  template <template <class...> class T>
  using Type = T<Ts...>;

  template <template <class...> class T, template <class> class W>
  using WrapType = T<W<Ts>...>;

  template <size_t index>
  using ElementType = std::tuple_element_t<index, Type<std::tuple>>;

  template <class T>
  using Concat = typename concat<T>::type;
};

template <template <class...> class B, class Ts>
using TypesBuild = typename Ts::template Type<B>;

template <template <class...> class B, template <class> class W, class Ts>
using TypesWrapBuild = typename Ts::template WrapType<B, W>;

template <size_t index, class Ts>
using TypesElementBuild = typename Ts::template ElementType<index>;

}  // namespace detail
}  // namespace graph
#endif  // VORTEX_HELPERS_TYPES_HPP
