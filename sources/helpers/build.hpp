/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helpers.build component
/// ===========================================================================
#ifndef VORTEX_HELPERS_BUILD_HPP
#define VORTEX_HELPERS_BUILD_HPP

#include "helpers/traits.hpp"

namespace vortex::graph {
namespace detail {

namespace impl {
template <class Init, template <class...> class Container, class... Types>
inline auto build(Init&& init, Container<Types...>) {
  auto& ref_init = trait::lreference<Init>(std::forward<Init>(init));
  return Container<Types...>{Types{ref_init}...};
}
}  // namespace impl

template <class Container, class Init>
inline auto build(Init&& init) {
  return impl::build(std::forward<Init>(init), Container{});
}

}  // namespace detail
}  // namespace vortex::graph

#endif  // VORTEX_HELPERS_BUILD_HPP
