/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
#define VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP

#include <expected>

namespace vortex::graph::optimization {

/// ============================================================================
/// @brief
/// ============================================================================
enum class AlgorithmError { FAIL, NUMERIC_LIMIT };
template <class Graph, class GraphSolver>
class Algorithm {
 public:
  explicit Algorithm(Graph& graph) : graph_{graph}, solver_(graph) {}

  Algorithm(const Algorithm&) = delete;
  auto operator=(const Algorithm&) -> Algorithm& = delete;
  Algorithm(Algorithm&&) = default;
  auto operator=(Algorithm&&) -> Algorithm& = delete;

  ~Algorithm() = default;

  /// @brief Algorithm initialization.
  /// @return Nothing or an algorithm error.
  auto init(bool reset) -> std::expected<void, AlgorithmError> {
    auto result = solver_.init(reset);
    if (not result) {
      return std::unexpected(AlgorithmError::FAIL);
    }
    return {};
  }
  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  template <bool first_iteration, bool last_iteration>
  auto solve() -> std::expected<bool, AlgorithmError>;

 protected:
  Graph& graph_;
  GraphSolver solver_;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
