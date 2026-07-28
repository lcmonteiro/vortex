/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.move component
/// ===========================================================================
#ifndef VORTEX_HELPERS_MOVE_HPP
#define VORTEX_HELPERS_MOVE_HPP
#include <iterator>
#include <utility>

#include "helpers/traits.hpp"

namespace vortex::helpers {

template <class K, class V>
inline auto get_value(const std::pair<K, V>& pair) -> decltype(auto) {
  return std::get<V>(pair);
}

template <class V>
inline auto get_value(const V& value) -> decltype(auto) {
  return value;
}

/// @brief Moves all elements of the specified type.
template <class T, class Fn>
inline auto move_all(T& source, T& destination, Fn&& execute) -> void {
  apply(std::forward<Fn>(execute), source);
  if (source.size() > destination.size()) {
    source.insert(std::make_move_iterator(std::begin(destination)),
                  std::make_move_iterator(std::end(destination)));
    destination.clear();
    destination.swap(source);
  } else {
    destination.insert(std::make_move_iterator(std::begin(source)),
                       std::make_move_iterator(std::end(source)));
    source.clear();
  }
}

/// @brief Moves all elements matching a condition.
template <class T, class Fn1, class Fn2>
inline auto move_if(T& source, T& destination, Fn1&& evaluate, Fn2&& execute) -> void {
  auto& ref_evaluate = lreference<Fn1>(std::forward<Fn1>(evaluate));
  auto& ref_execute = lreference<Fn2>(std::forward<Fn2>(execute));
  for (auto iter = source.begin(); iter != source.end();) {
    if (ref_evaluate(get_value(*iter))) {
      std::ignore = destination.emplace(*iter);
      ref_execute(get_value(*iter));
      iter = source.erase(iter);
    } else {
      ++iter;
    }
  }
}

/// @brief Moves an element by key.
template <class T, class Key, class Fn>
inline auto move(T& source, T& destination, const Key& key, Fn&& execute) -> void {
  auto& ref_execute = lreference<Fn>(std::forward<Fn>(execute));
  if (auto iter = source.find(key); iter != std::end(source)) {
    std::ignore = destination.emplace(*iter);
    ref_execute(get_value(*iter));
    std::ignore = source.erase(iter);
    return;
  }
}

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_MOVE_HPP
