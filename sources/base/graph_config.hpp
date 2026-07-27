/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph_engine.common component
/// ===========================================================================
#ifndef VORTEX_BASE_GRAPH_CONFIG_HPP
#define VORTEX_BASE_GRAPH_CONFIG_HPP
#include <map>
#include <set>
namespace graph {

/// ===========================================================================
/// Default Graph Config
/// @brief this is a default configuration use on Graph
/// ===========================================================================
struct DefaultConfig {
  /// @brief Initial size of the cache in bytes.
  /// @details Set to 0x80000 (524,288 bytes).
  static constexpr size_t CacheInitSize = 0x80000U;

  /// @brief Maximum size of a single cache block in bytes.
  /// @details Set to 0x8000 (32,768 bytes).
  static constexpr size_t CacheBlockMaxSize = 0x8000;

  /// @brief Data type for node keys.
  using Key = size_t;

  ///@brief Data type for node containers.
  template <class K, class V>
  using Map = std::pmr::map<K, V>;

  /// @brief Data type for edge containers.
  template <class T>
  using Set = std::pmr::set<T>;
};

}  // namespace graph
#endif  // VORTEX_BASE_GRAPH_CONFIG_HPP
