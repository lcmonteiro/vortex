/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.build component
/// ===========================================================================
#ifndef VORTEX_HELPERS_BUILD_HPP
#define VORTEX_HELPERS_BUILD_HPP

#include "helpers/utility.hpp"

namespace vortex::helpers {

namespace details {
template <class Init, template <class...> class Container, class... Types>
inline auto build(Init&& init, Container<Types...>) {
  auto& ref_init = lreference<Init>(std::forward<Init>(init));
  return Container<Types...>{Types{ref_init}...};
}
}  // namespace details

/// @brief Builds a container of types using a provided initialization function.
/// @tparam Container The container template to build (e.g., std::tuple, std::array).
/// @tparam Init The type of the initialization function.
/// @param init The initialization function to be applied to each type in the container.
/// @return A container of types initialized using the provided function.
template <class Container, class Init>
inline auto build(Init&& init) {
  return details::build(std::forward<Init>(init), Container{});
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_BUILD_HPP
