/// ===========================================================================
/// @file
///
/// @brief vortex.graph.config component
/// ===========================================================================
#ifndef VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
#define VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
#include <cstddef>
#include <map>
#include <set>
namespace vortex::graph {

/// ===========================================================================
/// @brief Default configuration used on Graph.
/// ===========================================================================
struct DefaultConfig {
  /// @brief Initial size of the cache in bytes.
  /// @details Set to 0x80000 (524,288 bytes).
  static constexpr std::size_t CacheInitSize = 0x80000U;

  /// @brief Maximum size of a single cache block in bytes.
  /// @details Set to 0x8000 (32,768 bytes).
  static constexpr std::size_t CacheBlockMaxSize = 0x8000;

  /// @brief Data type for node keys.
  using Key = std::size_t;

  /// @brief Data type for node containers.
  template <class K, class V>
  using Map = std::pmr::map<K, V>;

  /// @brief Data type for edge containers.
  template <class T>
  using Set = std::pmr::set<T>;
};

}  // namespace vortex::graph
#endif  // VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
