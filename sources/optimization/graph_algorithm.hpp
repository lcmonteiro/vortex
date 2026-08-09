/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
#define VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP

#include "helpers/expected.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Error codes reported by the optimization algorithm.
/// ===============================================================================================
enum class algorithm_error { kFail, kNumericLimit };

/// ===============================================================================================
/// @brief Base class for optimization algorithms operating on a graph.
///
/// Owns the graph and its solver and provides the common initialization shared
/// by concrete algorithms (e.g. LevenbergAlgorithm).
///
/// @tparam Graph       The graph type to optimize.
/// @tparam GraphSolver The graph solver type used to build and solve the system.
/// ===============================================================================================
template <class Graph, class GraphSolver>
class algorithm {
  using graph_type = Graph;
  using graph_solver_type = GraphSolver;

 public:
  explicit algorithm(Graph& graph) : graph_{graph}, solver_(graph) {}

  algorithm(const algorithm&) = delete;
  algorithm(algorithm&&) = default;
  ~algorithm() = default;

  auto operator=(const algorithm&) -> algorithm& = delete;
  auto operator=(algorithm&&) -> algorithm& = delete;

  /// @brief Algorithm initialization.
  /// @return Nothing or an algorithm error.
  auto init(bool reset) -> helpers::expected<void, algorithm_error> {
    auto result = solver_.init(reset);
    if (not result) {
      return helpers::unexpected(algorithm_error::kFail);
    }
    return {};
  }

  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  template <bool first_iteration, bool last_iteration>
  auto solve() -> helpers::expected<bool, algorithm_error>;

 protected:
  graph_type& graph_;
  graph_solver_type solver_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_HPP
