/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.apply component
/// ===========================================================================
#ifndef VORTEX_HELPERS_APPLY_HPP
#define VORTEX_HELPERS_APPLY_HPP

#include <cstddef>
#include <tuple>
#include <utility>

#include "helpers/traits.hpp"

namespace vortex::helpers {
namespace details {

template <class F, class T, std::size_t... Is>
inline auto apply_tuple(F&& func, T&& tuple, std::index_sequence<Is...>) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(func));
  auto& ref_tuple = lreference<T>(std::forward<T>(tuple));
  (ref_func(std::get<Is>(ref_tuple)), ...);
}

template <class F, class T>
inline auto apply_set(F&& func, T&& set) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(func));
  auto& ref_set = lreference<T>(std::forward<T>(set));
  for (auto& value : ref_set) {
    ref_func(value);
  }
}

template <class F, class T>
inline auto apply_map(F&& func, T&& map) -> void {
  auto& ref_func = lreference<F>(std::forward<F>(func));
  auto& ref_map = lreference<T>(std::forward<T>(map));
  for (auto& value_pair : ref_map) {
    ref_func(std::get<1>(value_pair));
  }
}

}  // namespace details

template <class F, class... Ts>
inline auto apply(F&& func, std::tuple<Ts...>& data) -> void {
  details::apply_tuple(std::forward<F>(func), data, std::make_index_sequence<sizeof...(Ts)>{});
}

template <class F, class... Ts>
inline auto apply(F&& func, const std::tuple<Ts...>& data) -> void {
  details::apply_tuple(std::forward<F>(func), data, std::make_index_sequence<sizeof...(Ts)>{});
}

template <class F, class T, if_set_like<T> = 0>
inline auto apply(F&& func, T& data) -> void {
  details::apply_set(std::forward<F>(func), data);
}

template <class F, class T, if_set_like<T> = 0>
inline auto apply(F&& func, const T& data) -> void {
  details::apply_set(std::forward<F>(func), data);
}

template <class F, class T, if_map_like<T> = 0>
inline auto apply(F&& func, T& data) -> void {
  details::apply_map(std::forward<F>(func), data);
}

template <class F, class T, if_map_like<T> = 0>
inline auto apply(F&& func, const T& data) -> void {
  details::apply_map(std::forward<F>(func), data);
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_APPLY_HPP
