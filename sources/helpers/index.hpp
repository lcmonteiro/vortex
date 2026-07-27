/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.index component
/// ===========================================================================
#ifndef VORTEX_HELPERS_INDEX_HPP
#define VORTEX_HELPERS_INDEX_HPP

#include <cstddef>
#include <type_traits>
#include <utility>

#include "sources/helpers/traits.hpp"

namespace graph {
namespace detail {

namespace impl {
template <size_t... Es, class F, size_t... Is>
void for_each(F&& fn, std::index_sequence<Is...>) {
  auto& ref_func = trait::lreference<F>(std::forward<F>(fn));
  if constexpr (std::is_invocable_v<F, size_t>) {
    (ref_func.operator()(Es..., Is), ...);
  } else {
    (ref_func.template operator()<Es..., Is>(), ...);
  }
}
template <class F, size_t... Is>
void for_each_pair(F&& fn, std::index_sequence<Is...>) {
  auto& ref_func = trait::lreference<F>(std::forward<F>(fn));
  (for_each<Is>(ref_func, std::make_index_sequence<Is + 1>{}), ...);
}
}  // namespace impl

template <size_t N>
struct Indexes {
  static constexpr auto value = std::make_index_sequence<N>{};
};

template <size_t N, class F>
void for_each(Indexes<N>, F&& fn) {
  impl::for_each(std::forward<F>(fn), Indexes<N>::value);
}

template <size_t N, class F>
void for_each_pair(Indexes<N>, F&& fn) {
  impl::for_each_pair(std::forward<F>(fn), Indexes<N>::value);
}

}  // namespace detail
}  // namespace graph
#endif  // VORTEX_HELPERS_INDEX_HPP
