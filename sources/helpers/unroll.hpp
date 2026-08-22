/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.unroll component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_UNROLL_HPP
#define VORTEX_HELPERS_UNROLL_HPP

#include <cstddef>
#include <type_traits>
#include <utility>

#include "helpers/indices.hpp"
#include "helpers/utility.hpp"

namespace vortex::helpers {

namespace details {
template <std::size_t... Es, class F, std::size_t... Is>
auto unroll(F&& fn, indices<Is...>) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  if constexpr (std::is_invocable_v<F, std::size_t>) {
    (ref_func.operator()(Es..., Is), ...);
  } else {
    (ref_func.template operator()<Es..., Is>(), ...);
  }
}
template <class F, std::size_t... Is>
auto unroll_pair(F&& fn, indices<Is...>) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  (unroll<Is>(ref_func, make_indices<Is + 1>{}), ...);
}
}  // namespace details

/// @brief Compile-time carrier of an indices<0..N-1>.
/// @tparam N Number of indices in the sequence.
template <std::size_t N>
struct range {
  static constexpr auto value = make_indices<N>{};
};

template <std::size_t N, class F>
auto unroll(range<N>, F&& fn) -> void {
  details::unroll(std::forward<F>(fn), range<N>::value);
}

template <std::size_t N, class F>
auto unroll_pair(range<N>, F&& fn) -> void {
  details::unroll_pair(std::forward<F>(fn), range<N>::value);
}

}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_UNROLL_HPP
