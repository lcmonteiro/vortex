/// ===========================================================================
/// @file
///
/// @brief vortex.helper.invoke component
/// ===========================================================================
#ifndef VORTEX_HELPERS_INVOKE_HPP
#define VORTEX_HELPERS_INVOKE_HPP

#include <cstddef>
#include <utility>

#include "helpers/traits.hpp"

namespace vortex::helpers {

namespace details {
template <class Fcall, class Fargs, size_t... Is>
constexpr decltype(auto) invoke(Fcall&& call, Fargs&& args, std::index_sequence<Is...>) {
  auto& ref_args = lreference<Fargs>(std::forward<Fargs>(args));
  return std::forward<Fcall>(call)(ref_args.template operator()<Is>()...);
}
}  // namespace details

template <size_t N>
struct Expand {
  static constexpr auto value = std::make_index_sequence<N>{};
};

template <size_t N, class Fcall, class Fargs>
decltype(auto) invoke(Fcall&& call, Fargs args, Expand<N> expand) {
  return details::invoke(std::forward<Fcall>(call), std::forward<Fargs>(args), expand.value);
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_INVOKE_HPP
