/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.unroll component
/// ===========================================================================
#ifndef VORTEX_HELPERS_UNROLL_HPP
#define VORTEX_HELPERS_UNROLL_HPP

#include <cstddef>
#include <type_traits>
#include <utility>

#include "helpers/traits.hpp"

namespace vortex::helpers {

namespace details {
template <size_t... Es, class F, size_t... Is>
auto unroll(F&& fn, std::index_sequence<Is...>) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  if constexpr (std::is_invocable_v<F, size_t>) {
    (ref_func.operator()(Es..., Is), ...);
  } else {
    (ref_func.template operator()<Es..., Is>(), ...);
  }
}
template <class F, size_t... Is>
auto unroll_pair(F&& fn, std::index_sequence<Is...>) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  (unroll<Is>(ref_func, std::make_index_sequence<Is + 1>{}), ...);
}
}  // namespace details

/// @brief Compile-time carrier of a std::index_sequence<0..N-1>.
/// @tparam N Number of indices in the sequence.
template <size_t N>
struct Indexes {
  static constexpr auto value = std::make_index_sequence<N>{};
};

template <size_t N, class F>
auto unroll(Indexes<N>, F&& fn) -> void {
  details::unroll(std::forward<F>(fn), Indexes<N>::value);
}

template <size_t N, class F>
auto unroll_pair(Indexes<N>, F&& fn) -> void {
  details::unroll_pair(std::forward<F>(fn), Indexes<N>::value);
}

}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_UNROLL_HPP
