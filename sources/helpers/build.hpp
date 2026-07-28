/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.build component
/// ===========================================================================
#ifndef VORTEX_HELPERS_BUILD_HPP
#define VORTEX_HELPERS_BUILD_HPP

#include "helpers/traits.hpp"

namespace vortex::helpers {

namespace details {
template <class Init, template <class...> class Container, class... Types>
inline auto build(Init&& init, Container<Types...>) {
  auto& ref_init = lreference<Init>(std::forward<Init>(init));
  return Container<Types...>{Types{ref_init}...};
}
}  // namespace details

template <class Container, class Init>
inline auto build(Init&& init) {
  return details::build(std::forward<Init>(init), Container{});
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_BUILD_HPP
