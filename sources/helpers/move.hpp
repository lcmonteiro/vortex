/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.move component
/// ===========================================================================
#ifndef VORTEX_HELPERS_MOVE_HPP
#define VORTEX_HELPERS_MOVE_HPP
#include <iterator>
#include <utility>

#include "sources/helpers/traits.hpp"

namespace graph {
namespace detail {

template <class K, class V>
inline decltype(auto) get_value(const std::pair<K, V>& pair) {
  return std::get<V>(pair);
}

template <class V>
inline decltype(auto) get_value(const V& value) {
  return value;
}

/// @brief move all elements of specified type
template <class T, class Fn>
inline void move_all(T& source, T& destination, Fn&& execute) {
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

/// @brief move all elements via condition
template <class T, class Fn1, class Fn2>
inline void move_if(T& source, T& destination, Fn1&& evaluate, Fn2&& execute) {
  auto& ref_evaluate = trait::lreference<Fn1>(std::forward<Fn1>(evaluate));
  auto& ref_execute = trait::lreference<Fn2>(std::forward<Fn2>(execute));
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

/// @brief move elements by key
template <class T, class Key, class Fn>
inline void move(T& source, T& destination, const Key& key, Fn&& execute) {
  auto& ref_execute = trait::lreference<Fn>(std::forward<Fn>(execute));
  if (auto iter = source.find(key); iter != std::end(source)) {
    std::ignore = destination.emplace(*iter);
    ref_execute(get_value(*iter));
    std::ignore = source.erase(iter);
    return;
  }
}

}  // namespace detail
}  // namespace graph

#endif  // VORTEX_HELPERS_MOVE_HPP
