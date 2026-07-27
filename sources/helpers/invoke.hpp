/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief g2o_dual.helper.invoke component
/// ===========================================================================
#ifndef G2O_DUAL_HELPERS_INVOKE_HPP
#define G2O_DUAL_HELPERS_INVOKE_HPP

#include <cstddef>
#include <utility>

#include "sources/helpers/traits.hpp"

namespace graph {
namespace detail {

namespace impl {
template <class Fcall, class Fargs, size_t... Is>
constexpr decltype(auto) invoke(Fcall&& call, Fargs&& args, std::index_sequence<Is...>) {
  auto& ref_args = trait::lreference<Fargs>(std::forward<Fargs>(args));
  return std::forward<Fcall>(call)(ref_args.template operator()<Is>()...);
}
}  // namespace impl

template <size_t N>
struct Expand {
  static constexpr auto value = std::make_index_sequence<N>{};
};

template <size_t N, class Fcall, class Fargs>
decltype(auto) invoke(Fcall&& call, Fargs args, Expand<N> expand) {
  return impl::invoke(std::forward<Fcall>(call), std::forward<Fargs>(args), expand.value);
}

}  // namespace detail
}  // namespace graph

#endif  // G2O_DUAL_HELPERS_INVOKE_HPP
