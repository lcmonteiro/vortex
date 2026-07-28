/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP
#define VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP

namespace vortex::graph::optimization {
/// ============================================================================
/// @class GraphSolver
/// @brief A templated class that provides solver functionality for a graph
/// using a linear solver.
///
/// @tparam Graph The graph type to be solved (can be any graph type, as long as
/// it is compatible).
/// @tparam LinearSolver The linear solver type used to perform solving
/// operations on the graph.
/// ============================================================================
template <class Graph, class LinearSolver>
class GraphSolver {
 public:
  explicit GraphSolver(Graph& graph) : graph_{graph} {}

  ~GraphSolver() = default;
  GraphSolver(const GraphSolver&) = delete;
  GraphSolver(GraphSolver&&) = default;

  GraphSolver& operator=(const GraphSolver&) = delete;
  GraphSolver& operator=(GraphSolver&&) = delete;

  /// @brief Initializes the GraphSolver.
  ///
  /// @param bool A boolean parameter, which could be used to specify additional
  /// initialization options (optional usage depending on the solver
  /// implementation).
  /// @return Returns a boolean indicating the success (true) or failure (false)
  /// of the initialization.
  bool init(bool) { return lsolver_.init(); }

 protected:
  Graph& graph_;
  LinearSolver lsolver_;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP
