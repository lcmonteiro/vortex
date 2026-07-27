/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.apply component
/// ===========================================================================
#ifndef VORTEX_HELPERS_APPLY_HPP
#define VORTEX_HELPERS_APPLY_HPP

#include <tuple>
#include <utility>

#include "sources/helpers/traits.hpp"

namespace vortex::graph {
namespace detail {
namespace impl {

template <class F, class T, size_t... Is>
inline void apply_tuple(F&& func, T&& tuple, std::index_sequence<Is...>) {
  auto& ref_func = trait::lreference<F>(std::forward<F>(func));
  auto& ref_tuple = trait::lreference<T>(std::forward<T>(tuple));
  (ref_func(std::get<Is>(ref_tuple)), ...);
}

template <class F, class T>
inline void apply_set(F&& func, T&& set) {
  auto& ref_func = trait::lreference<F>(std::forward<F>(func));
  auto& ref_set = trait::lreference<T>(std::forward<T>(set));
  for (auto& value : ref_set) {
    ref_func(value);
  }
}

template <class F, class T>
inline void apply_map(F&& func, T&& map) {
  auto& ref_func = trait::lreference<F>(std::forward<F>(func));
  auto& ref_map = trait::lreference<T>(std::forward<T>(map));
  for (auto& value_pair : ref_map) {
    ref_func(std::get<1>(value_pair));
  }
}

}  // namespace impl

template <class F, class... Ts>
inline void apply(F&& func, std::tuple<Ts...>& data) {
  impl::apply_tuple(std::forward<F>(func), data, std::make_index_sequence<sizeof...(Ts)>{});
}

template <class F, class... Ts>
inline void apply(F&& func, const std::tuple<Ts...>& data) {
  impl::apply_tuple(std::forward<F>(func), data, std::make_index_sequence<sizeof...(Ts)>{});
}

template <class F, class T, trait::if_set_like<T> = 0>
inline void apply(F&& func, T& data) {
  impl::apply_set(std::forward<F>(func), data);
}

template <class F, class T, trait::if_set_like<T> = 0>
inline void apply(F&& func, const T& data) {
  impl::apply_set(std::forward<F>(func), data);
}

template <class F, class T, trait::if_map_like<T> = 0>
inline void apply(F&& func, T& data) {
  impl::apply_map(std::forward<F>(func), data);
}

template <class F, class T, trait::if_map_like<T> = 0>
inline void apply(F&& func, const T& data) {
  impl::apply_map(std::forward<F>(func), data);
}

}  // namespace detail
}  // namespace vortex::graph

#endif  // VORTEX_HELPERS_APPLY_HPP
