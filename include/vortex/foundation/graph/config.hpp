/// ===============================================================================================
/// @file
///
/// @brief vortex.graph.config component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
#define VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
#include <cstddef>
#include <map>
#include <set>
namespace vortex::graph {

/// ===============================================================================================
/// @brief Default configuration used on Graph.
/// ===============================================================================================
struct default_config {
  /// @brief Initial size of the cache in bytes.
  /// @details Set to 0x1000000 (16,777,216 bytes).
  static constexpr std::size_t cache_init_size = 0x1000000;

  /// @brief Maximum size of a big cache block in bytes.
  /// @details Set to 0x100000 (1,048,576 bytes).
  static constexpr std::size_t cache_big_block_max_size = 0x100000;

  /// @brief Maximum size of a small cache block in bytes.
  /// @details Set to 0x1000 (4,096 bytes).
  static constexpr std::size_t cache_small_block_max_size = 0x1000;

  /// @brief Data type for node keys.
  using key_type = std::size_t;

  /// @brief Data type for node containers.
  template <class K, class V>
  using map_type = std::pmr::map<K, V>;

  /// @brief Data type for edge containers.
  template <class T>
  using set_type = std::pmr::set<T>;
};

}  // namespace vortex::graph
#endif  // VORTEX_FOUNDATION_GRAPH_GRAPH_CONFIG_HPP
