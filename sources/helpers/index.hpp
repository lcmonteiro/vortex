/// ===========================================================================
/// @file
///
/// @brief vortex.helper.index component
/// ===========================================================================
#ifndef VORTEX_HELPERS_INDEX_HPP
#define VORTEX_HELPERS_INDEX_HPP

#include <cstddef>
#include <type_traits>
#include <utility>

#include "helpers/traits.hpp"

namespace vortex::helpers {

namespace details {
template <size_t... Es, class F, size_t... Is>
void for_each(F&& fn, std::index_sequence<Is...>) {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  if constexpr (std::is_invocable_v<F, size_t>) {
    (ref_func.operator()(Es..., Is), ...);
  } else {
    (ref_func.template operator()<Es..., Is>(), ...);
  }
}
template <class F, size_t... Is>
void for_each_pair(F&& fn, std::index_sequence<Is...>) {
  auto& ref_func = lreference<F>(std::forward<F>(fn));
  (for_each<Is>(ref_func, std::make_index_sequence<Is + 1>{}), ...);
}
}  // namespace details

template <size_t N>
struct Indexes {
  static constexpr auto value = std::make_index_sequence<N>{};
};

template <size_t N, class F>
void for_each(Indexes<N>, F&& fn) {
  details::for_each(std::forward<F>(fn), Indexes<N>::value);
}

template <size_t N, class F>
void for_each_pair(Indexes<N>, F&& fn) {
  details::for_each_pair(std::forward<F>(fn), Indexes<N>::value);
}

}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_INDEX_HPP
