/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
#define VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP

#include <expected>

namespace vortex::graph {
namespace optimization {

/// ============================================================================
/// @brief
/// ============================================================================
enum class AlgorithmError { FAIL, NUMERIC_LIMIT };
template <class Graph, class GraphSolver>
class Algorithm {
 public:
  explicit Algorithm(Graph& graph) : graph_{graph}, solver_(graph) {}

  Algorithm(const Algorithm&) = delete;
  Algorithm& operator=(const Algorithm&) = delete;
  Algorithm(Algorithm&&) = default;
  Algorithm& operator=(Algorithm&&) = delete;

  ~Algorithm() = default;

  /// @brief Algorithm initialization.
  /// @return Nothing or an algorithm error.
  std::expected<void, AlgorithmError> init(bool reset) {
    auto result = solver_.init(reset);
    if (not result) {
      return std::unexpected(AlgorithmError::FAIL);
    }
    return {};
  }
  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  template <bool first_iteration, bool last_iteration>
  std::expected<bool, AlgorithmError> solve();

 protected:
  Graph& graph_;
  GraphSolver solver_;
};

}  // namespace optimization
}  // namespace vortex::graph

#endif  // VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
