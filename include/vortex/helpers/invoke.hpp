/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.invoke component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_INVOKE_HPP
#define VORTEX_HELPERS_INVOKE_HPP

#include <cstddef>
#include <utility>

#include "vortex/helpers/indices.hpp"
#include "vortex/helpers/utility.hpp"

namespace vortex::helpers {

namespace details {
template <class Fcall, class Fargs, std::size_t... Is>
constexpr auto invoke(Fcall&& call, Fargs&& args, indices<Is...>) -> decltype(auto) {
  auto& ref_args = lreference<Fargs>(std::forward<Fargs>(args));
  return std::forward<Fcall>(call)(ref_args.template operator()<Is>()...);
}
}  // namespace details

/// @brief Compile-time carrier of an indices<0..N-1> used to expand an indexed argument
/// pack into a call.
/// @tparam N Number of indices to expand.
template <std::size_t N>
struct expand {
  static constexpr auto value = make_indices<N>{};
};

template <std::size_t N, class Fcall, class Fargs>
auto invoke(Fcall&& call, Fargs args, expand<N> indices) -> decltype(auto) {
  return details::invoke(std::forward<Fcall>(call), std::forward<Fargs>(args), indices.value);
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_INVOKE_HPP
